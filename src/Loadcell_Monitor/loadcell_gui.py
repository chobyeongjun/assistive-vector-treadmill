"""
================================================================
 Loadcell Monitor GUI
 기준무게 대비 실시간 체중 비교 디스플레이
================================================================

 BLE (Nordic UART Service) 로 Teensy 4.1에서 전송된
 L/R 로드셀 힘 데이터를 수신하여 기준무게 대비 시각화

 패킷 포맷: "SL2c<L_force*100>n<R_force*100>n"

 사용법:
   python loadcell_gui.py

 의존성:
   pip install bleak PyQt6
================================================================
"""

import sys
import asyncio
import re

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QDoubleSpinBox, QSizePolicy, QLineEdit
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject, QThread, QPointF
from PyQt6.QtGui import QFont, QColor, QPainter, QPen, QBrush

import bleak
from bleak import BleakClient, BleakScanner

# ================================================================
# BLE 설정 (Nordic UART Service)
# ================================================================

NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID      = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Notify (데이터 수신)
NUS_RX_UUID      = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # Write  (명령 전송)

DEVICE_NAME = "Walker"  # BLE 디바이스 이름

N_TO_KG = 1.0 / 9.80665  # N → kgf 변환


# ================================================================
# BLE 데이터 파서
# ================================================================

class LoadcellParser:
    """BLE 수신 패킷 파싱
       - 스트림 데이터:  "SL2c<L>n<R>n"             → (L_force, R_force)
       - 응답 메시지:   "SR:<msg>\\n"                → "<msg>"
         예) "SR:LOG_START:LC_00.CSV\\n", "SR:TARE_OK:L=3.2,R=1.1\\n"
    """

    _PATTERN = re.compile(r"SL(\d+)c(-?\d+)n(-?\d+)n|SR:([^\n]*)\n")

    def __init__(self):
        self.buffer = ""

    def feed(self, data: bytes):
        """바이트 데이터 추가 후 (loadcell_pairs, response_msgs) 반환"""
        self.buffer += data.decode("utf-8", errors="ignore")

        loadcell = []
        responses = []
        last_end = 0

        for m in self._PATTERN.finditer(self.buffer):
            if m.group(1) is not None:  # SL 패킷
                l_raw = int(m.group(2))
                r_raw = int(m.group(3))
                loadcell.append((l_raw / 100.0, r_raw / 100.0))
            else:  # SR 응답
                responses.append(m.group(4))
            last_end = m.end()

        # 소비된 부분 제거, 미완성 꼬리만 유지
        self.buffer = self.buffer[last_end:]

        # 오버플로 방지
        if len(self.buffer) > 256:
            self.buffer = self.buffer[-64:]

        return loadcell, responses


# ================================================================
# BLE Worker (별도 스레드)
# ================================================================

class BleSignals(QObject):
    data_received = pyqtSignal(float, float)   # (l_force, r_force)
    status_changed = pyqtSignal(str)           # 상태 메시지
    response_received = pyqtSignal(str)        # 펌웨어 응답 (SR: 메시지)
    connected = pyqtSignal()
    disconnected = pyqtSignal()


class BleWorker(QThread):
    def __init__(self, signals: BleSignals):
        super().__init__()
        self.signals = signals
        self.parser = LoadcellParser()
        self.client = None
        self._running = True
        self._should_connect = False
        self._should_disconnect = False
        self._send_cmd = None

    def request_connect(self):
        self._should_connect = True

    def request_disconnect(self):
        self._should_disconnect = True

    def send_command(self, cmd: str):
        self._send_cmd = cmd

    def stop(self):
        self._running = False

    def run(self):
        asyncio.run(self._main_loop())

    async def _main_loop(self):
        while self._running:
            if self._should_connect:
                self._should_connect = False
                await self._do_connect()

            if self._should_disconnect:
                self._should_disconnect = False
                await self._do_disconnect()

            if self._send_cmd and self.client and self.client.is_connected:
                cmd = self._send_cmd
                self._send_cmd = None
                try:
                    await self.client.write_gatt_char(
                        NUS_RX_UUID, (cmd + "\n").encode(), response=False
                    )
                except Exception as e:
                    self.signals.status_changed.emit(f"Send error: {e}")

            await asyncio.sleep(0.05)

    async def _do_connect(self):
        self.signals.status_changed.emit("Scanning...")

        try:
            devices = await BleakScanner.discover(timeout=5.0)
            target = None
            for d in devices:
                if d.name and DEVICE_NAME in d.name:
                    target = d
                    break

            if not target:
                self.signals.status_changed.emit(f"'{DEVICE_NAME}' not found. Retry.")
                return

            self.signals.status_changed.emit(f"Connecting to {target.name}...")
            self.client = BleakClient(target.address)
            await self.client.connect()

            # Notify 구독
            await self.client.start_notify(NUS_TX_UUID, self._notification_handler)

            # "start" 명령 전송 (BLE 스트림 활성화)
            await self.client.write_gatt_char(
                NUS_RX_UUID, b"start\n", response=False
            )

            # 현재 로깅 상태 쿼리 → LOG_ACTIVE:<name> 또는 LOG_IDLE 응답 유발
            await asyncio.sleep(0.1)
            await self.client.write_gatt_char(
                NUS_RX_UUID, b"status\n", response=False
            )

            self.signals.status_changed.emit(f"Connected: {target.name}")
            self.signals.connected.emit()

        except Exception as e:
            self.signals.status_changed.emit(f"Connection failed: {e}")

    async def _do_disconnect(self):
        if self.client and self.client.is_connected:
            try:
                await self.client.write_gatt_char(
                    NUS_RX_UUID, b"stop\n", response=False
                )
                await asyncio.sleep(0.1)
                await self.client.disconnect()
            except Exception:
                pass
        self.client = None
        self.signals.status_changed.emit("Disconnected")
        self.signals.disconnected.emit()

    def _notification_handler(self, sender, data: bytearray):
        loadcell, responses = self.parser.feed(bytes(data))
        for l_force, r_force in loadcell:
            self.signals.data_received.emit(l_force, r_force)
        for resp in responses:
            self.signals.response_received.emit(resp)


# ================================================================
# 기준무게 비교 위젯 (QPainter 직접 렌더링 — 최소 지연)
# ================================================================

class WeightComparisonWidget(QWidget):
    """기준무게 대비 L+R 합산 무게를 점과 선으로 실시간 표시"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._ref = 0.0
        self._sum = 0.0
        self._lf = 0.0
        self._rf = 0.0
        self._range = 10.0  # 기준선 위/아래 표시 범위 (kg)

        self.setMinimumSize(400, 300)
        self.setSizePolicy(QSizePolicy.Policy.Expanding,
                           QSizePolicy.Policy.Expanding)

        # ── 렌더링 리소스 사전 할당 (paintEvent 내 객체 생성 최소화) ──
        self._bg = QColor(26, 26, 46)
        self._ref_pen = QPen(QColor(255, 215, 0), 3, Qt.PenStyle.DashLine)
        self._grid_pen = QPen(QColor(50, 50, 70), 1, Qt.PenStyle.DotLine)
        self._grid_text_color = QColor(80, 80, 100)
        self._ref_label_color = QColor(255, 215, 0)
        self._white = QColor(255, 255, 255)
        self._left_color = QColor(0, 212, 255)
        self._right_color = QColor(255, 107, 53)
        self._shadow_color = QColor(0, 0, 0, 60)

        self._dot_above = QColor(0, 255, 136)   # 초록 — 기준 초과
        self._dot_below = QColor(255, 100, 107)  # 빨강 — 기준 미달
        self._dot_on    = QColor(255, 215, 0)    # 금색 — 기준 일치

        self._sum_font  = QFont("Courier New", 32, QFont.Weight.Bold)
        self._diff_font = QFont("Courier New", 16)
        self._ref_font  = QFont("Courier New", 14, QFont.Weight.Bold)
        self._lr_font   = QFont("Courier New", 16)
        self._grid_font = QFont("Courier New", 10)

    # ── 외부 인터페이스 ──

    def set_data(self, lf: float, rf: float):
        self._lf = lf
        self._rf = rf
        self._sum = lf + rf
        self.update()  # repaint 예약 (Qt가 자동 coalesce)

    def set_reference(self, v: float):
        self._ref = v
        self.update()

    def set_range(self, v: float):
        self._range = max(v, 1.0)
        self.update()

    # ── 렌더링 ──

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        mx, my = 70, 30

        # 배경
        p.fillRect(0, 0, w, h, self._bg)

        # 그리기 영역 계산
        dx = mx
        dw = w - 2 * mx
        dt = my
        db = h - my - 40  # 하단 L/R 텍스트 공간
        dh = db - dt
        ref_y = dt + dh * 0.5
        half_h = dh * 0.5

        # ── 그리드 (스케일 눈금) ──
        step = self._grid_step()
        n = int(self._range / step)
        p.setFont(self._grid_font)

        for i in range(1, n + 1):
            for sign in (1, -1):
                offset_n = i * step * sign
                py = ref_y - (offset_n / self._range) * half_h
                if dt <= py <= db:
                    p.setPen(self._grid_pen)
                    p.drawLine(dx, int(py), dx + dw, int(py))
                    p.setPen(QPen(self._grid_text_color))
                    val = self._ref + offset_n
                    fmt = f"{val:.1f}" if step < 1 else f"{val:.0f}"
                    p.drawText(4, int(py + 4), fmt)

        # ── ±10% 보조선 ──
        if self._ref > 0 and self._range > 0:
            for pct, label in ((0.20, "+20%"), (-0.20, "-20%")):
                off_kg = self._ref * pct
                py = ref_y - (off_kg / self._range) * half_h
                if dt <= py <= db:
                    p.setPen(QPen(QColor(255, 100, 100, 160), 2, Qt.PenStyle.DashLine))
                    p.drawLine(dx, int(py), dx + dw, int(py))
                    p.setPen(QPen(QColor(255, 130, 130, 220)))
                    p.setFont(self._grid_font)
                    p.drawText(dx + dw + 4, int(py + 4), label)

        # ── 기준선 ──
        p.setPen(self._ref_pen)
        p.drawLine(dx, int(ref_y), dx + dw, int(ref_y))

        # 기준선 라벨
        p.setPen(QPen(self._ref_label_color))
        p.setFont(self._ref_font)
        ref_text = f"{self._ref:.0f} kg"
        p.drawText(4, int(ref_y - 8), ref_text)

        # ── 점 위치 계산 ──
        diff = self._sum - self._ref
        if self._range > 0:
            px_off = (diff / self._range) * half_h
        else:
            px_off = 0.0
        px_off = max(-half_h, min(half_h, px_off))

        dot_x = dx + dw * 0.5
        dot_y = ref_y - px_off  # 화면 Y 반전
        dot_r = 22.0

        # 점 색상 결정
        thr = max(self._ref * 0.03, 2.0)
        if abs(diff) < thr:
            dc = self._dot_on
        elif diff > 0:
            dc = self._dot_above
        else:
            dc = self._dot_below

        # 글로우 (저비용 반투명 원)
        glow = QColor(dc)
        glow.setAlpha(40)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(glow))
        p.drawEllipse(QPointF(dot_x, dot_y), dot_r + 14, dot_r + 14)

        # 그림자
        p.setBrush(QBrush(self._shadow_color))
        p.drawEllipse(QPointF(dot_x + 2, dot_y + 2), dot_r, dot_r)

        # 점 본체
        p.setBrush(QBrush(dc))
        p.drawEllipse(QPointF(dot_x, dot_y), dot_r, dot_r)

        # ── 합산 무게 텍스트 (점 우측) ──
        p.setPen(QPen(self._white))
        p.setFont(self._sum_font)
        p.drawText(int(dot_x + dot_r + 25), int(dot_y + 12), f"{self._sum:.1f} kg")

        # 기준 대비 차이
        if self._ref > 0:
            p.setPen(QPen(dc))
            p.setFont(self._diff_font)
            sign_s = "+" if diff >= 0 else ""
            p.drawText(int(dot_x + dot_r + 25), int(dot_y + 38),
                       f"({sign_s}{diff:.1f})")

        # ── L/R 개별 값 (하단) ──
        by = h - 12
        p.setFont(self._lr_font)

        p.setPen(QPen(self._left_color))
        p.drawText(dx, by, f"L: {self._lf:.1f} kg")

        p.setPen(QPen(self._right_color))
        rt = f"R: {self._rf:.1f} kg"
        rw_px = p.fontMetrics().horizontalAdvance(rt)
        p.drawText(dx + dw - rw_px, by, rt)

        p.end()

    def _grid_step(self):
        r = self._range
        if r <= 2:   return 0.5
        if r <= 5:   return 1
        if r <= 10:  return 2
        if r <= 20:  return 5
        if r <= 50:  return 10
        if r <= 100: return 20
        return 50


# ================================================================
# GUI 메인 윈도우
# ================================================================

BTN_STYLE = """
    QPushButton {
        background-color: #16213e;
        color: white;
        font-size: 15px;
        font-weight: bold;
        padding: 7px 16px;
        border: 2px solid #0f3460;
        border-radius: 8px;
    }
    QPushButton:hover { background-color: #0f3460; }
    QPushButton:pressed { background-color: #533483; }
    QPushButton:disabled { background-color: #333; color: #666; }
"""

SPIN_STYLE = """
    QDoubleSpinBox {
        background-color: #16213e;
        color: white;
        font-size: 18px;
        font-weight: bold;
        padding: 4px 8px;
        border: 2px solid #0f3460;
        border-radius: 8px;
        min-width: 110px;
    }
    QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
        width: 22px;
        border: none;
        background-color: #0f3460;
    }
    QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover {
        background-color: #533483;
    }
"""

EDIT_STYLE = """
    QLineEdit {
        background-color: #16213e;
        color: white;
        font-size: 15px;
        padding: 6px 10px;
        border: 2px solid #0f3460;
        border-radius: 8px;
        selection-background-color: #533483;
    }
    QLineEdit:focus { border: 2px solid #533483; }
"""


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Loadcell Monitor - Weight Comparison")
        self.setMinimumSize(900, 600)
        self.resize(1100, 750)

        self.setStyleSheet("""
            QMainWindow { background-color: #0f0f23; }
            QWidget { background-color: #0f0f23; }
        """)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(15, 10, 15, 10)
        main_layout.setSpacing(8)

        # ── 상단: 상태 + 연결 버튼 ──
        top = QHBoxLayout()

        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet(
            "color: #888; font-size: 16px; background: transparent;")
        top.addWidget(self.status_label, stretch=1)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setStyleSheet(BTN_STYLE)
        self.connect_btn.clicked.connect(self.on_connect)
        top.addWidget(self.connect_btn)

        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.setStyleSheet(BTN_STYLE)
        self.disconnect_btn.clicked.connect(self.on_disconnect)
        self.disconnect_btn.setEnabled(False)
        top.addWidget(self.disconnect_btn)

        self.tare_btn = QPushButton("Calibration")
        self.tare_btn.setStyleSheet(BTN_STYLE)
        self.tare_btn.clicked.connect(self.on_tare)
        self.tare_btn.setEnabled(False)
        top.addWidget(self.tare_btn)

        self.log_btn = QPushButton("Log Start")
        self.log_btn.setStyleSheet(BTN_STYLE)
        self.log_btn.clicked.connect(self.on_log_toggle)
        self.log_btn.setEnabled(False)
        top.addWidget(self.log_btn)

        self._is_logging = False
        main_layout.addLayout(top)

        # ── 설정: 기준무게 + 표시범위 ──
        settings = QHBoxLayout()

        lbl_ref = QLabel("기준무게:")
        lbl_ref.setStyleSheet(
            "color: #FFD700; font-size: 16px; font-weight: bold; background: transparent;")
        settings.addWidget(lbl_ref)

        self.ref_spin = QDoubleSpinBox()
        self.ref_spin.setRange(0, 200)
        self.ref_spin.setValue(0)
        self.ref_spin.setSingleStep(1)
        self.ref_spin.setSuffix(" kg")
        self.ref_spin.setDecimals(1)
        self.ref_spin.setStyleSheet(SPIN_STYLE)
        self.ref_spin.valueChanged.connect(self._on_ref_changed)
        settings.addWidget(self.ref_spin)

        settings.addStretch()

        lbl_range = QLabel("표시범위:")
        lbl_range.setStyleSheet(
            "color: #AAA; font-size: 14px; background: transparent;")
        settings.addWidget(lbl_range)

        self.range_spin = QDoubleSpinBox()
        self.range_spin.setRange(1, 100)
        self.range_spin.setValue(10)
        self.range_spin.setSingleStep(1)
        self.range_spin.setSuffix(" kg")
        self.range_spin.setDecimals(0)
        self.range_spin.setStyleSheet(SPIN_STYLE)
        self.range_spin.valueChanged.connect(self._on_range_changed)
        settings.addWidget(self.range_spin)

        main_layout.addLayout(settings)

        # ── 로그 파일 설정 ──
        log_row = QHBoxLayout()

        lbl_log = QLabel("Log file:")
        lbl_log.setStyleSheet(
            "color: #AAA; font-size: 14px; background: transparent;")
        log_row.addWidget(lbl_log)

        self.name_edit = QLineEdit()
        self.name_edit.setPlaceholderText("파일명 (비우면 자동 번호, .CSV 자동 부여)")
        self.name_edit.setStyleSheet(EDIT_STYLE)
        self.name_edit.returnPressed.connect(self.on_apply_name)
        self.name_edit.setEnabled(False)
        log_row.addWidget(self.name_edit, stretch=1)

        self.apply_name_btn = QPushButton("Apply Name")
        self.apply_name_btn.setStyleSheet(BTN_STYLE)
        self.apply_name_btn.clicked.connect(self.on_apply_name)
        self.apply_name_btn.setEnabled(False)
        log_row.addWidget(self.apply_name_btn)

        self.current_log_label = QLabel("Log: —")
        self.current_log_label.setStyleSheet(
            "color: #7FD8BE; font-size: 13px; font-family: 'Courier New';"
            " background: transparent; padding: 0 8px;")
        log_row.addWidget(self.current_log_label)

        main_layout.addLayout(log_row)

        # ── 중앙: 비교 위젯 ──
        self.comparison = WeightComparisonWidget()
        main_layout.addWidget(self.comparison, stretch=1)

        # ── 하단: Hz ──
        self.hz_label = QLabel("0 Hz")
        self.hz_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.hz_label.setStyleSheet(
            "color: #555; font-size: 14px; background: transparent;")
        main_layout.addWidget(self.hz_label)

        # ── BLE Worker ──
        self.ble_signals = BleSignals()
        self.ble_signals.data_received.connect(self._on_data)
        self.ble_signals.status_changed.connect(self._on_status)
        self.ble_signals.response_received.connect(self._on_response)
        self.ble_signals.connected.connect(self._on_connected)
        self.ble_signals.disconnected.connect(self._on_disconnected)

        self.ble_worker = BleWorker(self.ble_signals)
        self.ble_worker.start()

        # Hz 카운터
        self._pkt_count = 0
        self._hz_timer = QTimer()
        self._hz_timer.timeout.connect(self._update_hz)
        self._hz_timer.start(1000)

    # ── 설정 변경 ──

    def _on_ref_changed(self, v):
        self.comparison.set_reference(v)

    def _on_range_changed(self, v):
        self.comparison.set_range(v)

    # ── BLE 이벤트 ──

    def _on_data(self, lf: float, rf: float):
        self.comparison.set_data(lf * N_TO_KG, rf * N_TO_KG)
        self._pkt_count += 1

    def _on_status(self, msg: str):
        self.status_label.setText(msg)
        if "failed" in msg or "not found" in msg:
            self.connect_btn.setEnabled(True)

    def _on_connected(self):
        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(True)
        self.tare_btn.setEnabled(True)
        self.log_btn.setEnabled(True)
        self.name_edit.setEnabled(True)
        self.apply_name_btn.setEnabled(True)
        # 실제 로깅 상태는 LOG_ACTIVE/LOG_IDLE 응답 수신 시 _on_response에서 갱신

    def _on_disconnected(self):
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.tare_btn.setEnabled(False)
        self.log_btn.setEnabled(False)
        self.name_edit.setEnabled(False)
        self.apply_name_btn.setEnabled(False)
        self._is_logging = False
        self._set_log_btn_state(False)
        self.current_log_label.setText("Log: —")
        self.comparison.set_data(0.0, 0.0)

    # ── 펌웨어 응답 핸들러 (SR: 메시지) ──

    def _on_response(self, msg: str):
        """SR:<msg> 응답 처리 — 로깅 상태/파일명 UI 동기화"""
        if msg.startswith("LOG_START:"):
            fname = msg[len("LOG_START:"):]
            self._is_logging = True
            self._set_log_btn_state(True)
            self.current_log_label.setText(f"Log: {fname}  ●REC")
            self.status_label.setText(f"Logging → {fname}")
        elif msg.startswith("LOG_STOP:"):
            fname = msg[len("LOG_STOP:"):]
            self._is_logging = False
            self._set_log_btn_state(False)
            self.current_log_label.setText(f"Log: {fname} (stopped)")
            self.status_label.setText(f"Log stopped: {fname}")
        elif msg.startswith("LOG_ACTIVE:"):
            fname = msg[len("LOG_ACTIVE:"):]
            self._is_logging = True
            self._set_log_btn_state(True)
            self.current_log_label.setText(f"Log: {fname}  ●REC")
        elif msg == "LOG_IDLE":
            self._is_logging = False
            self._set_log_btn_state(False)
            self.current_log_label.setText("Log: (idle)")
        elif msg.startswith("LOG_FAIL:"):
            reason = msg[len("LOG_FAIL:"):]
            self._is_logging = False
            self._set_log_btn_state(False)
            self.current_log_label.setText(f"Log FAIL: {reason}")
            self.status_label.setText(f"Log failed: {reason}")
        elif msg == "LOG_NAME_CLEARED":
            self.current_log_label.setText("Log: (auto-increment)")
        # STREAM_ON/OFF, TARE_OK 는 별도 시각 피드백 없이 무시 (status_label은 BLE 워커가 이미 설정)

    def _set_log_btn_state(self, logging: bool):
        if logging:
            self.log_btn.setText("Log Stop")
            self.log_btn.setStyleSheet(BTN_STYLE.replace("#16213e", "#8b0000"))
        else:
            self.log_btn.setText("Log Start")
            self.log_btn.setStyleSheet(BTN_STYLE)

    # ── 버튼 핸들러 ──

    def on_connect(self):
        self.connect_btn.setEnabled(False)
        self.ble_worker.request_connect()

    def on_disconnect(self):
        self.ble_worker.request_disconnect()

    def on_tare(self):
        self.ble_worker.send_command("tare")

    def on_apply_name(self):
        """파일명 입력창의 내용으로 로그 파일 전환
           - 내용이 있으면 'logname:<name>' → 펌웨어가 stop→새 파일 start
           - 비어있으면 'logname:' (빈 값) → auto-increment로 복귀
        """
        name = self.name_edit.text().strip()
        self.ble_worker.send_command(f"logname:{name}")

    def on_log_toggle(self):
        # 정지는 logstop, 시작은 파일명 필드가 채워져 있으면 logname:, 아니면 log
        if self._is_logging:
            self.ble_worker.send_command("logstop")
        else:
            name = self.name_edit.text().strip()
            if name:
                self.ble_worker.send_command(f"logname:{name}")
            else:
                self.ble_worker.send_command("log")
        # UI 상태는 LOG_START/LOG_STOP 응답 수신 시 갱신

    def _update_hz(self):
        self.hz_label.setText(f"{self._pkt_count} Hz")
        self._pkt_count = 0

    def closeEvent(self, event):
        self.ble_worker.stop()
        self.ble_worker.wait(2000)
        event.accept()


# ================================================================
# 엔트리 포인트
# ================================================================

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
