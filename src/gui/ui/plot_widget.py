"""
ARWalker Plot Widget - Optimized Version (No Lock Contention)

★ 최적화 버전:
- deque 기반 (단순, 안정적)
- numpy 변환 최소화
- 현재 탭만 업데이트
"""

from collections import deque
from typing import Optional, Dict
import numpy as np
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTabWidget, QLabel,
    QFrame, QDoubleSpinBox, QPushButton, QLineEdit, QSizePolicy, QComboBox
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont, QPainter, QColor, QPen, QPixmap
import pyqtgraph as pg

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from core.data_parser import WalkerData


# PyQtGraph 설정 (단순화)
pg.setConfigOptions(antialias=False, useOpenGL=False)
pg.setConfigOption('background', '#13131A')
pg.setConfigOption('foreground', '#94A3B8')


# Constants
BASE_WINDOW_WIDTH = 1100
BASE_WINDOW_HEIGHT = 680
MIN_SCALE_FACTOR = 0.7
MAX_SCALE_FACTOR = 1.4


class GCPIndicator(QWidget):
    """GCP 원형 인디케이터 (QPainter circular gauge)"""

    def __init__(self, label: str, color: str = "#4C9EFF", parent=None):
        super().__init__(parent)
        self._label = label
        self._color = QColor(color)
        self._value = 0.0
        self._bg_cache: Optional[QPixmap] = None
        self.setFixedSize(70, 88)

    def set_value(self, value: float):
        if value > 1:
            value = value / 100.0
        new_value = max(0.0, min(1.0, value))
        # ★ 0.5% 미만 변화는 repaint 스킵 (QPainter 오버헤드 감소)
        if abs(new_value - self._value) < 0.005:
            return
        self._value = new_value
        self.update()

    def _ensure_bg_cache(self, size: int):
        if self._bg_cache is not None and self._bg_cache.width() == size:
            return
        self._bg_cache = QPixmap(size, size)
        self._bg_cache.fill(Qt.transparent)
        painter = QPainter(self._bg_cache)
        painter.setRenderHint(QPainter.Antialiasing)
        margin = 4
        rect = (margin, margin, size - margin * 2, size - margin * 2)
        painter.setPen(QPen(QColor(40, 40, 55), 5))
        painter.drawArc(*rect, 0, 360 * 16)
        painter.end()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        cx, cy, r = self.width() // 2, 38, 28

        self._ensure_bg_cache(r * 2 + 8)
        if self._bg_cache:
            p.drawPixmap(cx - r - 4, cy - r - 4, self._bg_cache)

        # Glow effect (3-layer soft glow)
        span = int(-self._value * 360 * 16)
        if self._value > 0.01:
            for glow_expand, glow_alpha in [(6, 15), (4, 25), (2, 45)]:
                gr = r + glow_expand
                glow_color = QColor(self._color.red(), self._color.green(),
                                    self._color.blue(), glow_alpha)
                p.setPen(QPen(glow_color, 5, Qt.SolidLine, Qt.RoundCap))
                p.drawArc(cx - gr, cy - gr, gr * 2, gr * 2, 90 * 16, span)

        # Value arc
        p.setPen(QPen(self._color, 5, Qt.SolidLine, Qt.RoundCap))
        p.drawArc(cx - r, cy - r, r * 2, r * 2, 90 * 16, span)

        # Value text
        p.setPen(self._color)
        p.setFont(QFont("Inter", 14, QFont.Bold))
        p.drawText(0, 20, self.width(), 36, Qt.AlignCenter, f"{int(self._value * 100)}")

        # Label
        p.setPen(QColor("#64748B"))
        p.setFont(QFont("Inter", 9, QFont.Bold))
        p.drawText(0, 70, self.width(), 18, Qt.AlignCenter, self._label)
        p.end()


class TopBarWidget(QWidget):
    """상단 바 - File + Image + GCP"""

    save_requested = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._init_ui()

    def _init_ui(self):
        from ui.styles import C
        layout = QHBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(8)

        # File Logging — Experiment Setup
        file_frame = QFrame()
        file_frame.setObjectName("GlassCard")
        file_outer = QVBoxLayout(file_frame)
        file_outer.setContentsMargins(8, 5, 8, 5)
        file_outer.setSpacing(3)

        # Row 1: FILE label + experiment params
        row1 = QHBoxLayout()
        row1.setSpacing(4)

        file_label = QLabel("FILE")
        file_label.setStyleSheet(
            f"color:{C['muted']}; font-size:9px; font-weight:700; "
            f"letter-spacing:1px; background:transparent; border:none;"
        )
        row1.addWidget(file_label)

        combo_style = (
            f"QComboBox {{ background:{C['card']}; color:{C['text1']}; "
            f"border:1px solid {C['border']}; border-radius:3px; "
            f"font-size:10px; padding:1px 4px; }}"
            f"QComboBox::drop-down {{ border:none; width:14px; }}"
        )

        # Subject ID (text — 사람마다 다름)
        self.subject_input = QLineEdit()
        self.subject_input.setPlaceholderText("ID")
        self.subject_input.setMaxLength(10)
        self.subject_input.setFixedSize(46, 22)
        self.subject_input.setStyleSheet(
            f"background:{C['card']}; color:{C['text1']}; "
            f"border:1px solid {C['border']}; border-radius:3px; "
            f"font-size:10px; padding:1px 4px;"
        )
        row1.addWidget(self.subject_input)

        # Modality
        self.modality_combo = QComboBox()
        self.modality_combo.addItems(['TD', 'OG'])
        self.modality_combo.setFixedSize(46, 22)
        self.modality_combo.setStyleSheet(combo_style)
        row1.addWidget(self.modality_combo)

        # Incline (TD only)
        self.incline_combo = QComboBox()
        self.incline_combo.addItems(['level', 'incline'])
        self.incline_combo.setFixedSize(58, 22)
        self.incline_combo.setStyleSheet(combo_style)
        row1.addWidget(self.incline_combo)

        # Speed (TD only) — free text input, e.g. 0.5, 0.8, 1.0
        self.speed_input = QLineEdit()
        self.speed_input.setPlaceholderText("m/s")
        self.speed_input.setText("0.5")
        self.speed_input.setFixedSize(48, 22)
        self.speed_input.setStyleSheet(
            f"background:{C['card']}; color:{C['text1']}; "
            f"border:1px solid {C['border']}; border-radius:3px; "
            f"font-size:10px; padding:1px 4px;"
        )
        row1.addWidget(self.speed_input)

        # Device
        self.device_combo = QComboBox()
        self.device_combo.addItems(['walker', 'noassist'])
        self.device_combo.setFixedSize(70, 22)
        self.device_combo.setStyleSheet(combo_style)
        row1.addWidget(self.device_combo)

        # Attachment: high / middle / low (walker only)
        self.attachment_combo = QComboBox()
        self.attachment_combo.addItems(['high', 'middle', 'low'])
        self.attachment_combo.setFixedSize(62, 22)
        self.attachment_combo.setStyleSheet(combo_style)
        row1.addWidget(self.attachment_combo)

        # Angle: 0 / 30 (walker only)
        self.angle_combo = QComboBox()
        self.angle_combo.addItems(['0', '30'])
        self.angle_combo.setFixedSize(40, 22)
        self.angle_combo.setStyleSheet(combo_style)
        row1.addWidget(self.angle_combo)

        # Weight bearing: wb / nwb (noassist only, hidden by default)
        self.wb_combo = QComboBox()
        self.wb_combo.addItems(['wb', 'nwb'])
        self.wb_combo.setFixedSize(48, 22)
        self.wb_combo.setStyleSheet(combo_style)
        self.wb_combo.setVisible(False)
        row1.addWidget(self.wb_combo)

        row1.addStretch()
        file_outer.addLayout(row1)

        # Row 2: generated filename (read-only) + Save
        row2 = QHBoxLayout()
        row2.setSpacing(6)

        self._generated_filename = ''
        self.filename_display = QLabel()
        self.filename_display.setStyleSheet(
            f"color:{C['text2']}; font-size:9px; font-family:monospace; "
            f"background:transparent; border:none;"
        )
        row2.addWidget(self.filename_display, 1)

        self.save_btn = QPushButton("Save")
        self.save_btn.setObjectName("AccentBtn")
        self.save_btn.setFixedSize(50, 22)
        self.save_btn.clicked.connect(self._on_save_clicked)
        row2.addWidget(self.save_btn)

        file_outer.addLayout(row2)

        # Connect signals
        self.subject_input.textChanged.connect(self._update_filename)
        self.modality_combo.currentTextChanged.connect(self._on_modality_changed)
        self.incline_combo.currentTextChanged.connect(self._update_filename)
        self.speed_input.textChanged.connect(self._update_filename)
        self.device_combo.currentTextChanged.connect(self._on_device_changed)
        self.attachment_combo.currentTextChanged.connect(self._update_filename)
        self.angle_combo.currentTextChanged.connect(self._update_filename)
        self.wb_combo.currentTextChanged.connect(self._update_filename)

        self._update_filename()  # initial render

        file_frame.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        layout.addWidget(file_frame)

        # Image
        self.image_frame = QFrame()
        self.image_frame.setObjectName("GlassCard")
        image_layout = QVBoxLayout(self.image_frame)
        image_layout.setContentsMargins(6, 4, 6, 4)

        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet(
            f"color:{C['muted']}; font-size:11px; background:transparent; border:none;"
        )
        self.image_label.setMinimumSize(100, 50)
        image_layout.addWidget(self.image_label)

        self.image_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self.image_frame, 1)

        # GCP Indicators
        gcp_frame = QFrame()
        gcp_frame.setObjectName("GlassCard")
        gcp_layout = QHBoxLayout(gcp_frame)
        gcp_layout.setContentsMargins(8, 4, 8, 4)
        gcp_layout.setSpacing(8)

        gcp_label = QLabel("GCP")
        gcp_label.setStyleSheet(
            f"color:{C['muted']}; font-size:9px; font-weight:700; "
            f"letter-spacing:1px; background:transparent; border:none;"
        )
        gcp_layout.addWidget(gcp_label)

        self.gcp_left = GCPIndicator("Left", C['teal'])
        self.gcp_right = GCPIndicator("Right", C['pink'])
        gcp_layout.addWidget(self.gcp_left)
        gcp_layout.addWidget(self.gcp_right)

        gcp_frame.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        layout.addWidget(gcp_frame)

        self.setFixedHeight(140)
        self._original_pixmap = None

    def _on_modality_changed(self, modality: str):
        is_td = modality == 'TD'
        self.incline_combo.setVisible(is_td)
        self.speed_input.setVisible(is_td)
        self._update_filename()

    def _on_device_changed(self, device: str):
        is_walker = device == 'walker'
        self.attachment_combo.setVisible(is_walker)
        self.angle_combo.setVisible(is_walker)
        self.wb_combo.setVisible(not is_walker)
        self._update_filename()

    def _update_filename(self):
        from datetime import date
        today = date.today().strftime('%Y%m%d')
        subject = self.subject_input.text().strip() or 'ID'
        modality = self.modality_combo.currentText()
        device = self.device_combo.currentText()

        parts = [today, 'Robot', subject, modality]
        if modality == 'TD':
            parts.append(self.incline_combo.currentText())
            speed_raw = self.speed_input.text().strip() or '1_0'
            parts.append(speed_raw.replace('.', '_'))
        parts.append(device)
        if device == 'walker':
            parts.append(self.attachment_combo.currentText())
            parts.append(self.angle_combo.currentText())
        else:
            parts.append(self.wb_combo.currentText())

        self._generated_filename = '_'.join(parts)
        self.filename_display.setText(self._generated_filename)

    def _on_save_clicked(self):
        self.save_requested.emit(self._generated_filename)

    def set_left_gcp(self, value: float):
        self.gcp_left.set_value(value)

    def set_right_gcp(self, value: float):
        self.gcp_right.set_value(value)

    def set_image(self, image_path: str):
        from PyQt5.QtGui import QPixmap
        pixmap = QPixmap(image_path)
        if not pixmap.isNull():
            self._original_pixmap = pixmap
            QTimer.singleShot(100, self._update_image)

    def _update_image(self):
        if self._original_pixmap:
            w = self.image_frame.width() - 20
            h = self.image_frame.height() - 20
            if w > 0 and h > 0:
                scaled = self._original_pixmap.scaled(w, h, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                self.image_label.setPixmap(scaled)
                self.image_label.setText("")

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_image()


class SinglePlot(QWidget):
    """단일 플롯"""

    COLOR_LEFT = '#22d3ee'
    COLOR_RIGHT = '#f472b6'

    def __init__(self, title: str, y_range: tuple = None, parent=None):
        super().__init__(parent)
        self._title = title
        self._curves: Dict[str, pg.PlotDataItem] = {}
        self._y_range = y_range
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(3)

        # Y축 범위 컨트롤
        range_frame = QFrame()
        range_frame.setObjectName("GlassCard")
        range_bar = QHBoxLayout(range_frame)
        range_bar.setContentsMargins(8, 4, 8, 4)
        range_bar.setSpacing(6)

        range_bar.addWidget(QLabel("Y:"))

        self.y_min_spin = QDoubleSpinBox()
        self.y_min_spin.setRange(-10000, 10000)
        self.y_min_spin.setValue(self._y_range[0] if self._y_range else -100)
        self.y_min_spin.setFixedWidth(85)
        range_bar.addWidget(self.y_min_spin)

        range_bar.addWidget(QLabel("~"))

        self.y_max_spin = QDoubleSpinBox()
        self.y_max_spin.setRange(-10000, 10000)
        self.y_max_spin.setValue(self._y_range[1] if self._y_range else 100)
        self.y_max_spin.setFixedWidth(85)
        range_bar.addWidget(self.y_max_spin)

        self.apply_btn = QPushButton("Apply")
        self.apply_btn.setObjectName("AccentBtn")
        self.apply_btn.setFixedWidth(60)
        self.apply_btn.clicked.connect(self._apply_y_range)
        range_bar.addWidget(self.apply_btn)

        self.auto_btn = QPushButton("Auto")
        self.auto_btn.setObjectName("SecondaryBtn")
        self.auto_btn.setFixedWidth(55)
        self.auto_btn.clicked.connect(self._auto_y_range)
        range_bar.addWidget(self.auto_btn)

        range_bar.addStretch()
        layout.addWidget(range_frame)

        # Plot
        self.plot = pg.PlotWidget(title=self._title)
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self._legend = self.plot.addLegend(offset=(70, 10))
        self.plot.setMouseEnabled(x=False, y=False)
        self.plot.getPlotItem().setMenuEnabled(False)
        self.plot.setClipToView(True)
        self.plot.setDownsampling(auto=True, mode='peak')

        # X 범위는 batch_update에서 setXRange로 직접 제어 (autoRange 재계산 비용 제거)
        self.plot.enableAutoRange(axis='x', enable=False)

        if self._y_range:
            self.plot.setYRange(*self._y_range)

        layout.addWidget(self.plot)

    def _apply_y_range(self):
        y_min, y_max = self.y_min_spin.value(), self.y_max_spin.value()
        if y_min < y_max:
            self.plot.setYRange(y_min, y_max)
            self.plot.enableAutoRange(axis='y', enable=False)

    def _auto_y_range(self):
        self.plot.enableAutoRange(axis='y', enable=True)

    def add_curve(self, name: str, color: str, style=Qt.SolidLine):
        pen = pg.mkPen(color, width=2, style=style)
        curve = pg.PlotDataItem(pen=pen, name=name)
        self.plot.addItem(curve)
        self._curves[name] = curve
        return curve

    def update_curve(self, name: str, x_data, y_data):
        if name in self._curves:
            self._curves[name].setData(x_data, y_data)

    def batch_update(self, updates: list, x_end: int = None):
        """배치 업데이트 — autoRange 토글 없이 고정 X range 직접 설정.

        enableAutoRange(True) 호출 시 ViewBox.updateAutoRange()가 즉시 발동되어
        모든 커브 dataBounds()를 순회하는 비용(30Hz × 4커브 = 매 33ms 스파이크)을 제거.
        대신 x_end를 받아 setXRange로 O(1) 범위 설정.
        """
        for name, x_data, y_data in updates:
            if name in self._curves:
                self._curves[name].setData(x_data, y_data)
        if x_end is not None:
            x_start = max(0, x_end - PlotTabWidget.BUFFER_SIZE)
            self.plot.setXRange(x_start, x_end, padding=0.02)


class PlotTabWidget(QWidget):
    """탭 기반 플롯 위젯 - 단순화된 버전"""

    BUFFER_SIZE = 150  # ★ 50Hz × 3초 = 150샘플 (실시간 모니터링에 최적)

    gcp_updated = pyqtSignal(float, float)
    save_requested = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._mode = 0
        self._sample_count = 0
        self._gcp_callback = None  # RealtimeMode에서 설정

        # 단순한 deque 버퍼 (lock 없음)
        self._buffers = self._init_buffers()

        self._init_ui()

    def _init_buffers(self) -> dict:
        keys = [
            'time', 'l_gcp', 'r_gcp', 'l_pitch', 'r_pitch',
            'l_gyro', 'r_gyro',
            'l_des_force', 'r_des_force', 'l_act_force', 'r_act_force'
        ]
        return {k: deque(maxlen=self.BUFFER_SIZE) for k in keys}

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)

        self.tab_widget = QTabWidget()
        self._update_tab_style()
        layout.addWidget(self.tab_widget, 1)

        self._create_tabs()

    def _update_tab_style(self):
        # 전역 QSS의 QTabWidget/QTabBar 규칙 활용 — 추가 오버라이드 불필요
        pass

    def _create_tabs(self):
        self.tab_widget.clear()

        if self._mode == 0:
            self.force_plot = SinglePlot("Force (N)", (-10, 120))
            self.force_plot.add_curve("L Desired", "#34d399", Qt.DashLine)
            self.force_plot.add_curve("L Actual", "#22d3ee")
            self.force_plot.add_curve("R Desired", "#a78bfa", Qt.DashLine)
            self.force_plot.add_curve("R Actual", "#f472b6")
        else:
            self.force_plot = SinglePlot("Force (N)", (-10, 120))
            self.force_plot.add_curve("L Force", SinglePlot.COLOR_LEFT)
            self.force_plot.add_curve("R Force", SinglePlot.COLOR_RIGHT)
        self.tab_widget.addTab(self.force_plot, "Force")

        self.imu_plot = SinglePlot("IMU Pitch (deg)", (-45, 45))
        self.imu_plot.add_curve("L Pitch", SinglePlot.COLOR_LEFT)
        self.imu_plot.add_curve("R Pitch", SinglePlot.COLOR_RIGHT)
        self.tab_widget.addTab(self.imu_plot, "IMU Pitch")

        self.gyro_plot = SinglePlot("IMU Gyro Y (deg/s)", (-200, 200))
        self.gyro_plot.add_curve("L Gyro", SinglePlot.COLOR_LEFT)
        self.gyro_plot.add_curve("R Gyro", SinglePlot.COLOR_RIGHT)
        self.tab_widget.addTab(self.gyro_plot, "Gyro")


    def set_mode(self, mode: int):
        if mode != self._mode:
            self._mode = mode
            current = self.tab_widget.currentIndex()
            self._create_tabs()
            if current < self.tab_widget.count():
                self.tab_widget.setCurrentIndex(current)

    def add_data(self, data: WalkerData):
        """데이터 추가 - 단순 deque append"""
        self._sample_count += 1
        b = self._buffers

        b['time'].append(self._sample_count)
        b['l_gcp'].append(data.l_gcp * 100)
        b['r_gcp'].append(data.r_gcp * 100)
        b['l_pitch'].append(data.l_pitch)
        b['r_pitch'].append(data.r_pitch)
        b['l_gyro'].append(data.l_gyro_y)
        b['r_gyro'].append(data.r_gyro_y)
        b['l_des_force'].append(data.l_des_force)
        b['r_des_force'].append(data.r_des_force)
        b['l_act_force'].append(data.l_act_force)
        b['r_act_force'].append(data.r_act_force)

    def _to_array(self, d: deque) -> np.ndarray:
        """deque → numpy (캐시 재사용)"""
        n = len(d)
        if n == 0:
            return np.array([], dtype=np.float32)
        # 직접 복사 (list() 중간 단계 없음)
        arr = np.fromiter(d, dtype=np.float32, count=n)
        return arr

    def update_plots(self):
        """현재 탭만 업데이트 — 탭 인덱스 기반, 고정 X range로 autoRange 재계산 제거."""
        b = self._buffers
        if len(b['time']) < 2:
            return

        # GCP 콜백 (RealtimeMode 원형 게이지)
        if b['l_gcp'] and b['r_gcp'] and self._gcp_callback:
            self._gcp_callback(b['l_gcp'][-1], b['r_gcp'][-1])

        current = self.tab_widget.currentIndex()
        time_arr = self._to_array(b['time'])
        x_end = int(time_arr[-1]) if len(time_arr) > 0 else None

        if current == 0:  # Force
            if self._mode == 0:
                self.force_plot.batch_update([
                    ("L Desired", time_arr, self._to_array(b['l_des_force'])),
                    ("L Actual",  time_arr, self._to_array(b['l_act_force'])),
                    ("R Desired", time_arr, self._to_array(b['r_des_force'])),
                    ("R Actual",  time_arr, self._to_array(b['r_act_force'])),
                ], x_end=x_end)
            else:
                self.force_plot.batch_update([
                    ("L Force", time_arr, self._to_array(b['l_act_force'])),
                    ("R Force", time_arr, self._to_array(b['r_act_force'])),
                ], x_end=x_end)

        elif current == 1:  # IMU Pitch
            self.imu_plot.batch_update([
                ("L Pitch", time_arr, self._to_array(b['l_pitch'])),
                ("R Pitch", time_arr, self._to_array(b['r_pitch'])),
            ], x_end=x_end)

        elif current == 2:  # Gyro
            self.gyro_plot.batch_update([
                ("L Gyro", time_arr, self._to_array(b['l_gyro'])),
                ("R Gyro", time_arr, self._to_array(b['r_gyro'])),
            ], x_end=x_end)

    def set_gcp_callback(self, callback):
        """Set callback for GCP updates: callback(l_gcp, r_gcp)"""
        self._gcp_callback = callback

    def clear_data(self):
        self._buffers = self._init_buffers()
        self._sample_count = 0

        empty = np.array([], dtype=np.float32)
        for plot in [self.force_plot, self.imu_plot, self.gyro_plot]:
            if hasattr(plot, '_curves'):
                for curve in plot._curves.values():
                    curve.setData(empty, empty)

        if self._gcp_callback:
            self._gcp_callback(0.0, 0.0)

    def get_latest_values(self) -> dict:
        b = self._buffers
        return {
            'l_gcp': b['l_gcp'][-1] if b['l_gcp'] else 0,
            'r_gcp': b['r_gcp'][-1] if b['r_gcp'] else 0,
            'l_force': b['l_act_force'][-1] if b['l_act_force'] else 0,
            'r_force': b['r_act_force'][-1] if b['r_act_force'] else 0,
            'samples': self._sample_count
        }

    def set_scale_factor(self, factor: float):
        pass
