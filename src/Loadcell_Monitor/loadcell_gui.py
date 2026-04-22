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
    QLabel, QPushButton, QDoubleSpinBox, QSizePolicy
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
    """BLE 패킷 파싱: "SL2c<L>n<R>n" → (L_force, R_force)"""

    def __init__(self):
        self.buffer = ""

    def feed(self, data: bytes):
        """바이트 데이터를 버퍼에 추가하고 파싱된 결과 반환"""
        self.buffer += data.decode("utf-8", errors="ignore")
        results = []

        while True:
            # "SL" 패킷 찾기
            idx = self.buffer.find("SL")
            if idx == -1:
                # 버퍼가 너무 커지지 않도록 정리
                if len(self.buffer) > 256:
                    self.buffer = self.buffer[-64:]
                break

            # "SL" 이전 데이터 버림
            self.buffer = self.buffer[idx:]

            # 패턴 매칭: "SL2c<int>n<int>n"
            match = re.match(r"SL(\d+)c(-?\d+)n(-?\d+)n", self.buffer)
            if match:
                count = int(match.group(1))
                l_raw = int(match.group(2))
                r_raw = int(match.group(3))

                l_force = l_raw / 100.0
                r_force = r_raw / 100.0

                results.append((l_force, r_force))
                self.buffer = self.buffer[match.end():]
            else:
                # 패킷이 아직 불완전하면 대기
                if len(self.buffer) > 64:
                    self.buffer = self.buffer[2:]  # "SL" 건너뛰고 다음 탐색
                else:
                    break

        return results


# ================================================================
# BLE Worker (별도 스레드)
# ================================================================

class BleSignals(QObject):
    data_received = pyqtSignal(float, float)   # (l_force, r_force)
    status_changed = pyqtSignal(str)           # 상태 메시지
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

            # "start" 명령 전송
            await self.client.write_gatt_char(
                NUS_RX_UUID, b"start\n", response=False
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
        results = self.parser.feed(bytes(data))
        for l_force, r_force in results:
            self.signals.data_received.emit(l_force, r_force)


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

    def _on_disconnected(self):
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.tare_btn.setEnabled(False)
        self.log_btn.setEnabled(False)
        self._is_logging = False
        self.log_btn.setText("Log Start")
        self.comparison.set_data(0.0, 0.0)

    # ── 버튼 핸들러 ──

    def on_connect(self):
        self.connect_btn.setEnabled(False)
        self.ble_worker.request_connect()

    def on_disconnect(self):
        self.ble_worker.request_disconnect()

    def on_tare(self):
        self.ble_worker.send_command("tare")

    def on_log_toggle(self):
        if not self._is_logging:
            self.ble_worker.send_command("log")
            self._is_logging = True
            self.log_btn.setText("Log Stop")
            self.log_btn.setStyleSheet(BTN_STYLE.replace("#16213e", "#8b0000"))
        else:
            self.ble_worker.send_command("logstop")
            self._is_logging = False
            self.log_btn.setText("Log Start")
            self.log_btn.setStyleSheet(BTN_STYLE)

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
