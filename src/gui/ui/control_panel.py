"""
H-Walker Control Panel - Glassmorphism Design

모드별 파라미터 입력 및 제어 버튼 패널
QScrollArea 기반 반응형 레이아웃

All signals and public methods preserved from original.
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QComboBox,
    QDoubleSpinBox, QFrame, QTextEdit,
    QStackedWidget, QRadioButton, QLineEdit
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QFont

from ui.styles import C


class PlusMinusSpinBox(QWidget):
    """Custom SpinBox with vertical +/- buttons"""

    valueChanged = pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._init_ui()

    def _init_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.spin = QDoubleSpinBox()
        self.spin.setButtonSymbols(QDoubleSpinBox.NoButtons)
        self.spin.valueChanged.connect(self.valueChanged.emit)
        layout.addWidget(self.spin, 1)

        btn_container = QWidget()
        btn_layout = QVBoxLayout(btn_container)
        btn_layout.setContentsMargins(0, 0, 0, 0)
        btn_layout.setSpacing(0)

        self.up_btn = QPushButton("▲")
        self.up_btn.setFixedSize(24, 15)
        self.up_btn.setObjectName("GreenBtn")
        self.up_btn.setStyleSheet("""
            QPushButton { border-radius:0; border-top-right-radius:4px; padding:0; font-size:8px; font-weight:700; }
        """)
        self.up_btn.clicked.connect(self._increase)
        btn_layout.addWidget(self.up_btn)

        separator = QFrame()
        separator.setFixedSize(24, 1)
        separator.setStyleSheet(f"background-color:{C['border']};")
        btn_layout.addWidget(separator)

        self.down_btn = QPushButton("▼")
        self.down_btn.setFixedSize(24, 15)
        self.down_btn.setObjectName("RedBtn")
        self.down_btn.setStyleSheet("""
            QPushButton { border-radius:0; border-bottom-right-radius:4px; padding:0; font-size:8px; font-weight:700; }
        """)
        self.down_btn.clicked.connect(self._decrease)
        btn_layout.addWidget(self.down_btn)

        layout.addWidget(btn_container)

    def _increase(self):
        self.spin.setValue(self.spin.value() + self.spin.singleStep())

    def _decrease(self):
        self.spin.setValue(self.spin.value() - self.spin.singleStep())

    def setRange(self, min_val, max_val):
        self.spin.setRange(min_val, max_val)

    def setValue(self, val):
        self.spin.setValue(val)

    def value(self):
        return self.spin.value()

    def setSuffix(self, suffix):
        self.spin.setSuffix(suffix)

    def setSingleStep(self, step):
        self.spin.setSingleStep(step)

    def setFixedHeight(self, h):
        super().setFixedHeight(h)
        self.spin.setFixedHeight(h)

    def wheelEvent(self, event):
        """Scroll up = increase, scroll down = decrease"""
        if event.angleDelta().y() > 0:
            self._increase()
        else:
            self._decrease()
        event.accept()


def _section_label(text: str) -> QLabel:
    """Create an UPPERCASE section label (Glassmorphism style)"""
    lbl = QLabel(text.upper())
    lbl.setStyleSheet(
        f"color:{C['muted']}; font-size:9px; font-weight:700; "
        f"letter-spacing:1px; background:transparent; border:none;"
    )
    return lbl


def _glass_card() -> QFrame:
    """Create a GlassCard frame"""
    f = QFrame()
    f.setObjectName("GlassCard")
    return f


class ControlPanel(QWidget):
    """Control Panel - Glassmorphism dark theme

    Signals and public methods preserved exactly from original.
    """

    command_requested = pyqtSignal(str)
    logging_start_requested = pyqtSignal(str)
    logging_stop_requested = pyqtSignal()
    mode_changed = pyqtSignal(int)
    scan_requested = pyqtSignal()
    connect_requested = pyqtSignal(int)
    disconnect_requested = pyqtSignal()

    PANEL_WIDTH = 255

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(self.PANEL_WIDTH)
        self.setObjectName("SidebarInner")
        self._is_logging = False
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        # === BLE Connection ===
        layout.addWidget(self._create_ble_card())

        # === Logging (RealtimeMode 상단 바에 배치됨) ===
        self.log_card = self._create_log_card()

        # === Control ===
        layout.addWidget(self._create_control_card())

        # === Parameters (mode switching) ===
        self.param_stack = QStackedWidget()
        self.param_stack.addWidget(self._create_force_params())
        self.param_stack.addWidget(self._create_position_params())
        layout.addWidget(self.param_stack)

        # === Feedforward ===
        layout.addWidget(self._create_tff_card())

        # === Experiment Presets ===
        layout.addWidget(self._create_preset_card())

        layout.addStretch()

    # ------ BLE Card ------
    def _create_ble_card(self) -> QFrame:
        card = _glass_card()
        cl = QVBoxLayout(card)
        cl.setContentsMargins(10, 10, 10, 10)
        cl.setSpacing(6)

        cl.addWidget(_section_label("Connection"))

        # Status row
        sr = QHBoxLayout()
        self._status_dot = QLabel("\u25cf")
        self._status_dot.setStyleSheet(
            f"color:{C['red']}; font-size:10px; background:transparent; border:none;"
        )
        sr.addWidget(self._status_dot)
        self.connection_label = QLabel("Disconnected")
        self.connection_label.setStyleSheet(
            f"color:{C['red']}; font-size:11px; background:transparent; border:none;"
        )
        sr.addWidget(self.connection_label)
        sr.addStretch()
        cl.addLayout(sr)

        # Device combo
        self.device_combo = QComboBox()
        self.device_combo.setPlaceholderText("Select device...")
        cl.addWidget(self.device_combo)

        # Scan / Connect buttons
        br = QHBoxLayout()
        self.scan_btn = QPushButton("Scan")
        self.scan_btn.setObjectName("SecondaryBtn")
        self.scan_btn.clicked.connect(self.scan_requested.emit)
        br.addWidget(self.scan_btn)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setObjectName("AccentBtn")
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        br.addWidget(self.connect_btn)
        cl.addLayout(br)

        return card

    # ------ Control Card ------
    def _create_control_card(self) -> QFrame:
        card = _glass_card()
        cl = QVBoxLayout(card)
        cl.setContentsMargins(10, 10, 10, 10)
        cl.setSpacing(6)

        cl.addWidget(_section_label("Control"))

        # Mode radio buttons
        mr = QHBoxLayout()
        self._force_radio = QRadioButton("Force")
        self._force_radio.setChecked(True)
        self._pos_radio = QRadioButton("Position")
        mr.addWidget(self._force_radio)
        mr.addWidget(self._pos_radio)
        cl.addLayout(mr)
        self._force_radio.toggled.connect(self._on_mode_toggled)

        # Enable / Disable — Enable은 로깅 중일 때만 활성
        br = QHBoxLayout()
        self.enable_btn = QPushButton("Enable")
        self.enable_btn.setObjectName("GreenBtn")
        self.enable_btn.setEnabled(False)
        self.enable_btn.clicked.connect(lambda: self.command_requested.emit("e"))
        br.addWidget(self.enable_btn)

        self.disable_btn = QPushButton("Disable")
        self.disable_btn.setObjectName("RedBtn")
        self.disable_btn.clicked.connect(lambda: self.command_requested.emit("d"))
        br.addWidget(self.disable_btn)
        cl.addLayout(br)

        # Utility buttons
        ur = QHBoxLayout()
        for name, cmd in [("IMU Cal", "imu"), ("Zero", "motor"), ("Mark", "mark")]:
            b = QPushButton(name)
            b.setObjectName("SmallBtn")
            b.clicked.connect(lambda checked, c=cmd: self.command_requested.emit(c))
            ur.addWidget(b)

        self.clear_btn = QPushButton("Clear")
        self.clear_btn.setObjectName("SmallBtn")
        self.clear_btn.setStyleSheet(
            f"background:rgba(99,102,241,0.3); color:{C['purple']}; "
            f"border:1px solid rgba(99,102,241,0.3); border-radius:4px; "
            f"padding:3px 7px; font-size:11px;"
        )
        ur.addWidget(self.clear_btn)
        cl.addLayout(ur)

        return card

    # ------ Force Parameters ------
    def _create_force_params(self) -> QWidget:
        card = _glass_card()
        cl = QVBoxLayout(card)
        cl.setContentsMargins(10, 10, 10, 10)
        cl.setSpacing(4)

        cl.addWidget(_section_label("Force Parameters"))

        params = [
            ("Onset GCP", "onset_spin", 55, "%", "gs", 100),
            ("Peak GCP", "peak_gcp_spin", 70, "%", "gp", 100),
            ("Release GCP", "release_spin", 85, "%", "ge", 100),
            ("Peak Force", "peak_force_spin", 50, " N", "pf", 1),
        ]

        for label_text, attr_name, default, suffix, cmd_prefix, divisor in params:
            row = QHBoxLayout()
            lbl = QLabel(label_text)
            lbl.setStyleSheet(
                f"color:{C['text2']}; font-size:11px; background:transparent; border:none;"
            )
            row.addWidget(lbl)
            row.addStretch()

            spin = PlusMinusSpinBox()
            spin.setRange(0, 200 if "Force" in label_text else 100)
            spin.setValue(default)
            spin.setSuffix(suffix)
            spin.setSingleStep(1)
            spin.setFixedHeight(28)
            setattr(self, attr_name, spin)
            spin.spin.setFixedWidth(60)
            row.addWidget(spin)

            btn = QPushButton("Set")
            btn.setObjectName("SmallBtn")
            btn.setFixedWidth(30)
            if divisor == 100:
                btn.clicked.connect(
                    lambda checked, s=spin, p=cmd_prefix: self._send_param(p, s.value() / 100)
                )
            else:
                btn.clicked.connect(
                    lambda checked, s=spin, p=cmd_prefix: self._send_param(p, s.value())
                )
            row.addWidget(btn)
            cl.addLayout(row)

        sa = QPushButton("Set All")
        sa.setObjectName("AccentBtn")
        sa.clicked.connect(self._send_all_force_params)
        cl.addWidget(sa)

        return card

    # ------ Position Parameters ------
    def _create_position_params(self) -> QWidget:
        card = _glass_card()
        cl = QVBoxLayout(card)
        cl.setContentsMargins(10, 10, 10, 10)
        cl.setSpacing(4)

        cl.addWidget(_section_label("Position Parameters"))

        params = [
            ("Start GCP", "pos_start_spin", 20, "%", "ps", 100),
            ("End GCP", "pos_end_spin", 70, "%", "pe", 100),
            ("Amplitude", "amplitude_spin", 600, " deg", "pa", 1),
        ]

        for label_text, attr_name, default, suffix, cmd_prefix, divisor in params:
            row = QHBoxLayout()
            lbl = QLabel(label_text)
            lbl.setStyleSheet(
                f"color:{C['text2']}; font-size:11px; background:transparent; border:none;"
            )
            row.addWidget(lbl)
            row.addStretch()

            spin = PlusMinusSpinBox()
            spin.setRange(0, 2000 if "Amplitude" in label_text else 100)
            spin.setValue(default)
            spin.setSuffix(suffix)
            spin.setSingleStep(1)
            spin.setFixedHeight(28)
            setattr(self, attr_name, spin)
            spin.spin.setFixedWidth(60)
            row.addWidget(spin)

            btn = QPushButton("Set")
            btn.setObjectName("SmallBtn")
            btn.setFixedWidth(30)
            if divisor == 100:
                btn.clicked.connect(
                    lambda checked, s=spin, p=cmd_prefix: self._send_param(p, s.value() / 100)
                )
            else:
                btn.clicked.connect(
                    lambda checked, s=spin, p=cmd_prefix: self._send_param(p, s.value())
                )
            row.addWidget(btn)
            cl.addLayout(row)

        sa = QPushButton("Set All")
        sa.setObjectName("AccentBtn")
        sa.clicked.connect(self._send_all_position_params)
        cl.addWidget(sa)

        return card

    # ------ Feedforward Card ------
    def _create_tff_card(self) -> QFrame:
        card = _glass_card()
        cl = QVBoxLayout(card)
        cl.setContentsMargins(10, 10, 10, 10)
        cl.setSpacing(4)

        cl.addWidget(_section_label("Feedforward"))

        ff_params = [
            ("TM Speed", "tff_speed_spin", 100, " cm/s", "tm", 100),
            ("TFF Gain", "tff_gain_spin", 55, " %", "tg", 100),
            ("Motion FF", "motion_ff_spin", 20, " %", "fm", 100),
            ("Force FF", "force_ff_spin", 15, " %", "ff", 100),
        ]

        for label_text, attr_name, default, suffix, cmd_prefix, divisor in ff_params:
            row = QHBoxLayout()
            lbl = QLabel(label_text)
            lbl.setStyleSheet(
                f"color:{C['text2']}; font-size:11px; background:transparent; border:none;"
            )
            row.addWidget(lbl)
            row.addStretch()

            spin = PlusMinusSpinBox()
            max_val = 300 if "Speed" in label_text else (200 if "Motion" in label_text else 100)
            spin.setRange(0, max_val)
            spin.setValue(default)
            spin.setSuffix(suffix)
            spin.setSingleStep(5 if "Speed" in label_text or "TFF" in label_text or "Motion" in label_text else 1)
            spin.setFixedHeight(28)
            setattr(self, attr_name, spin)
            spin.spin.setFixedWidth(60)
            row.addWidget(spin)

            btn = QPushButton("Set")
            btn.setObjectName("SmallBtn")
            btn.setFixedWidth(30)
            btn.clicked.connect(
                lambda checked, s=spin, p=cmd_prefix: self._send_param(p, s.value() / 100)
            )
            row.addWidget(btn)
            cl.addLayout(row)

        apply_all_btn = QPushButton("Apply All FF")
        apply_all_btn.setObjectName("AccentBtn")
        apply_all_btn.clicked.connect(self._send_all_ff_params)
        cl.addWidget(apply_all_btn)

        return card

    # ------ Experiment Preset Card ------
    def _create_preset_card(self) -> QFrame:
        card = _glass_card()
        cl = QVBoxLayout(card)
        cl.setContentsMargins(10, 10, 10, 10)
        cl.setSpacing(4)

        cl.addWidget(_section_label("Experiment Presets"))

        # Preset definitions: (label, preset_number)
        presets = [
            ("High 30°", 1), ("High 0°", 2),
            ("Mid  30°", 3), ("Mid  0°", 4),
            ("Low  30°", 5), ("Low  0°", 6),
            ("Low  45°", 7),
        ]

        grid = QGridLayout()
        grid.setSpacing(4)
        for i, (label, n) in enumerate(presets):
            btn = QPushButton(label)
            btn.setObjectName("SmallBtn")
            btn.setFixedHeight(26)
            btn.clicked.connect(lambda checked, num=n: self._apply_preset(num))
            grid.addWidget(btn, i // 2, i % 2)
        cl.addLayout(grid)

        return card

    # ------ Log Card ------
    def _create_log_card(self) -> QFrame:
        card = _glass_card()
        cl = QVBoxLayout(card)
        cl.setContentsMargins(12, 10, 12, 10)
        cl.setSpacing(6)

        cl.addWidget(_section_label("Logging"))

        # 가로 분할: 왼쪽=log_text, 오른쪽=이름+버튼
        content_row = QHBoxLayout()
        content_row.setSpacing(8)
        content_row.setContentsMargins(0, 0, 0, 0)

        # 왼쪽: log_text
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setObjectName("LogText")
        self.log_text.setFixedWidth(140)
        self.log_text.setFixedHeight(72)
        content_row.addWidget(self.log_text)

        # 오른쪽: 이름 입력 + 버튼+상태
        right_col = QVBoxLayout()
        right_col.setSpacing(5)
        right_col.setContentsMargins(0, 0, 0, 0)

        self._log_name_edit = QLineEdit()
        self._log_name_edit.setPlaceholderText("실험 이름...")
        self._log_name_edit.setFixedHeight(30)
        self._log_name_edit.setStyleSheet(
            f"background:{C.get('card','#1e1e2e')}; color:white; "
            f"border:1px solid {C['border']}; border-radius:4px; padding:0 6px; font-size:11px;"
        )
        right_col.addWidget(self._log_name_edit)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(5)
        btn_row.setContentsMargins(0, 0, 0, 0)

        self._log_start_btn = QPushButton("▶ START")
        self._log_start_btn.setFixedHeight(32)
        self._log_start_btn.setStyleSheet(
            "QPushButton { background:rgba(74,222,128,0.18); color:#4ade80; "
            "border:1px solid rgba(74,222,128,0.5); border-radius:5px; "
            "padding:0 10px; font-size:11px; font-weight:700; }"
            "QPushButton:hover { background:rgba(74,222,128,0.32); }"
            "QPushButton:disabled { color:rgba(74,222,128,0.28); "
            "border-color:rgba(74,222,128,0.15); background:rgba(74,222,128,0.05); }"
        )
        self._log_start_btn.setEnabled(True)
        self._log_start_btn.clicked.connect(self._logging_start)
        btn_row.addWidget(self._log_start_btn, 1)

        self._log_stop_btn = QPushButton("■ STOP")
        self._log_stop_btn.setFixedHeight(32)
        self._log_stop_btn.setStyleSheet(
            "QPushButton { background:rgba(248,113,113,0.18); color:#f87171; "
            "border:1px solid rgba(248,113,113,0.5); border-radius:5px; "
            "padding:0 10px; font-size:11px; font-weight:700; }"
            "QPushButton:hover { background:rgba(248,113,113,0.32); }"
            "QPushButton:disabled { color:rgba(248,113,113,0.28); "
            "border-color:rgba(248,113,113,0.15); background:rgba(248,113,113,0.05); }"
        )
        self._log_stop_btn.setEnabled(False)
        self._log_stop_btn.clicked.connect(self._logging_stop)
        btn_row.addWidget(self._log_stop_btn, 1)

        self._log_dot = QLabel("●")
        self._log_dot.setFixedWidth(14)
        self._log_status_label = QLabel("STOP")
        self._set_logging_indicator(False)
        btn_row.addWidget(self._log_dot)
        btn_row.addWidget(self._log_status_label)

        right_col.addLayout(btn_row)
        content_row.addLayout(right_col)
        cl.addLayout(content_row)

        return card

    def _update_log_btn_states(self):
        """START: 로깅 중 아닐 때만. STOP: 로깅 중일 때만."""
        self._log_start_btn.setEnabled(not self._is_logging)
        self._log_stop_btn.setEnabled(self._is_logging)

    def _set_logging_indicator(self, is_logging: bool):
        was_logging = self._is_logging
        self._is_logging = is_logging
        if is_logging:
            self._log_dot.setStyleSheet(
                "color:#4CAF50; font-size:14px; background:transparent; border:none;"
            )
            self._log_status_label.setText("REC")
            self._log_status_label.setStyleSheet(
                "font-size:10px; font-weight:700; color:#4CAF50; background:transparent; border:none;"
            )
        else:
            self._log_dot.setStyleSheet(
                f"color:{C['muted']}; font-size:14px; background:transparent; border:none;"
            )
            self._log_status_label.setText("STOP")
            self._log_status_label.setStyleSheet(
                f"font-size:10px; font-weight:700; color:{C['muted']}; background:transparent; border:none;"
            )
        # START↔STOP 버튼 상태
        self._log_start_btn.setEnabled(not is_logging)
        self._log_stop_btn.setEnabled(is_logging)
        # Enable 버튼은 로깅 중일 때만 허용 (enable_btn이 아직 생성 안 됐을 수 있음)
        if hasattr(self, 'enable_btn'):
            self.enable_btn.setEnabled(is_logging)

    def _logging_start(self):
        name = self._log_name_edit.text().strip()
        self.logging_start_requested.emit(name)
        self._set_logging_indicator(True)
        self.log_text.append(f"▶ START: {name if name else '(auto)'}")

    def _logging_stop(self):
        self.logging_stop_requested.emit()
        self._set_logging_indicator(False)
        self.command_requested.emit("d")
        self.log_text.append("■ STOP → 모터 비활성화")

    def set_logging_status(self, is_logging: bool):
        """외부에서 로깅 상태 업데이트 (BLE 수신 등)"""
        self._set_logging_indicator(is_logging)

    # === Slots ===

    def _on_connect_clicked(self):
        idx = self.device_combo.currentIndex()
        if idx >= 0:
            self.connect_requested.emit(idx)

    def _on_mode_toggled(self, checked: bool):
        if checked:
            # Force radio is checked
            self.param_stack.setCurrentIndex(0)
            self.mode_changed.emit(0)
        else:
            self.param_stack.setCurrentIndex(1)
            self.mode_changed.emit(1)

    def _send_mode(self):
        mode = "mode0" if self._force_radio.isChecked() else "mode1"
        self.command_requested.emit(mode)

    def _send_param(self, prefix: str, value: float):
        cmd = f"{prefix}{value:.2f}"
        self.command_requested.emit(cmd)

    def _send_all_force_params(self):
        self._send_param("gs", self.onset_spin.value() / 100)
        self._send_param("gp", self.peak_gcp_spin.value() / 100)
        self._send_param("ge", self.release_spin.value() / 100)
        self._send_param("pf", self.peak_force_spin.value())

    def _send_all_position_params(self):
        self._send_param("ps", self.pos_start_spin.value() / 100)
        self._send_param("pe", self.pos_end_spin.value() / 100)
        self._send_param("pa", self.amplitude_spin.value())

    # Preset table: belt(cm/s), TFF(%), MotionFF(%), ActuatorFF(%)
    _PRESET_FF = {
        1: {"name": "High-30°", "belt": 100, "tff": 55, "motion": 35, "actuator": 15},
        2: {"name": "High-0°",  "belt": 100, "tff": 60, "motion": 35, "actuator": 15},
        3: {"name": "Mid-30°",  "belt": 100, "tff": 60, "motion": 35, "actuator": 15},
        4: {"name": "Mid-0°",   "belt": 100, "tff": 60, "motion": 35, "actuator": 15},
        5: {"name": "Low-30°",  "belt": 100, "tff": 50, "motion": 30, "actuator": 15},
        # Low-30° 60cm/s 변형: belt 수동 조절 (preset 5 → belt=60)
        6: {"name": "Low-0°",   "belt": 100, "tff": 50, "motion": 40, "actuator": 15},
        7: {"name": "Low-45°",  "belt": 100, "tff": 45, "motion": 40, "actuator": 15},
    }

    def _apply_preset(self, n: int):
        """Send preset command to Teensy AND update all GUI spinners per condition."""
        p = self._PRESET_FF.get(n)
        if p is None:
            return

        # 1) Update Force param spinners (공통)
        self.onset_spin.setValue(55)
        self.peak_gcp_spin.setValue(70)
        self.release_spin.setValue(85)
        self.peak_force_spin.setValue(50)

        # 2) Update FF spinners (조건별)
        self.tff_speed_spin.setValue(p["belt"])
        self.tff_gain_spin.setValue(p["tff"])
        self.motion_ff_spin.setValue(p["motion"])
        self.force_ff_spin.setValue(p["actuator"])

        # 3) Send preset command to Teensy (force params + FF 모두 설정)
        self.command_requested.emit(f"preset{n}")

        self.log(f"[OK] Preset {n} ({p['name']}) "
                 f"TFF={p['tff']}% MotFF={p['motion']}% ActFF={p['actuator']}%")

    def _send_all_ff_params(self):
        """현재 FF 슬라이더 값 전체를 BLE로 일괄 전송."""
        self._send_param("tm", self.tff_speed_spin.value() / 100)
        self._send_param("tg", self.tff_gain_spin.value() / 100)
        self._send_param("fm", self.motion_ff_spin.value() / 100)
        self._send_param("ff", self.force_ff_spin.value() / 100)
        self.log(f"[OK] Apply All FF: TM={self.tff_speed_spin.value()}cm/s "
                 f"TFF={self.tff_gain_spin.value()}% "
                 f"MotFF={self.motion_ff_spin.value()}% "
                 f"ActFF={self.force_ff_spin.value()}%")

    # === Public Methods (preserved) ===

    def update_devices(self, devices: list):
        self.device_combo.clear()
        for d in devices:
            name = d.name or "Unknown"
            self.device_combo.addItem(f"{name} ({d.address})")

    def set_connected(self, connected: bool):
        if connected:
            self._status_dot.setStyleSheet(
                f"color:{C['green']}; font-size:10px; background:transparent; border:none;"
            )
            self.connection_label.setText("BLE Connected")
            self.connection_label.setStyleSheet(
                f"color:{C['green']}; font-size:11px; background:transparent; border:none;"
            )
            self.connect_btn.setText("Disconnect")
            # Pulse animation
            if not hasattr(self, '_pulse_timer'):
                self._pulse_timer = QTimer()
                self._pulse_state = False
                self._pulse_timer.timeout.connect(self._toggle_pulse)
            self._pulse_timer.start(600)
        else:
            self._status_dot.setStyleSheet(
                f"color:{C['red']}; font-size:10px; background:transparent; border:none;"
            )
            self.connection_label.setText("Disconnected")
            self.connection_label.setStyleSheet(
                f"color:{C['red']}; font-size:11px; background:transparent; border:none;"
            )
            self.connect_btn.setText("Connect")
            if hasattr(self, '_pulse_timer'):
                self._pulse_timer.stop()

    def _toggle_pulse(self):
        """BLE connected pulse animation"""
        self._pulse_state = not self._pulse_state
        sz = "12px" if self._pulse_state else "9px"
        self._status_dot.setStyleSheet(
            f"color:{C['green']}; font-size:{sz}; background:transparent; border:none;"
        )

    def log(self, msg: str):
        """Log message with color support"""
        if "[ERROR]" in msg.upper():
            color = C['red']
        elif "[WARNING]" in msg.upper() or "[WARN]" in msg.upper():
            color = C['amber']
        elif "[OK]" in msg or "[SD]" in msg or "[FW]" in msg:
            color = C['green']
        else:
            color = C['text1']
        html_msg = f'<span style="color:{color};">{msg}</span>'
        self.log_text.append(html_msg)
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def set_scale_factor(self, factor: float):
        pass
