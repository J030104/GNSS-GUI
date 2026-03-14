import random
from typing import Optional

from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QGroupBox,
    QPushButton,
    QGridLayout,
    QFrame,
    QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer

from ...components.log_viewer import LogViewer
from ...components.video_viewer import VideoViewer
from ...utilities.video_streamer import StaticImageCamera, VideoFileCamera, ActiveCameraManager

import os
from pathlib import Path


class DashboardTab(QWidget):
    # Camera indices read from environment variables (fallback to defaults)
    AUX_CAMERA_INDEX = int(os.getenv("AUX_CAMERA_INDEX", "1"))
    GROUND_CAMERA_INDEX = int(os.getenv("GROUND_CAMERA_INDEX", "2"))

    # Feed modes
    FEED_AUX = "aux"
    FEED_GROUND = "ground"

    # Temporary feed sources
    GROUND_FEED_VIDEO = str(Path(__file__).resolve().parents[3] / "assets" / "videos" / "ground_feed.mp4")
    AUX_FEED_IMAGE = str(Path(__file__).resolve().parents[3] / "assets" / "images" / "soil_aux.png")

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._webcam_source = None
        self._active_feed = self.FEED_AUX  # which feed is currently selected
        self._drill_depth = 11.8  # cm, range 0.0–13.0
        self._la_direction = 0   # -1 rev, 0 stopped, 1 fwd
        self._init_ui()
        self._start_data_simulation()
        self._depth_timer = QTimer(self)
        self._depth_timer.setInterval(500)  # 0.1 cm per tick → 0.2 cm/s
        self._depth_timer.timeout.connect(self._update_drill_depth)

    def _start_data_simulation(self):
        """Start a 1-second timer to simulate live sensor data."""
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._update_sensor_data)
        self._timer.start(1000)

    def _update_drill_depth(self):
        """Increment or decrement drill depth while LA is moving."""
        self._drill_depth += self._la_direction * 0.1
        self._drill_depth = round(min(13.0, max(0.0, self._drill_depth)), 1)
        self.drill_depth_label.setText(f"Drill Depth: {self._drill_depth:.1f} cm")

    def _update_sensor_data(self):
        """Update atmospheric and soil probe labels with small random fluctuations."""
        def jitter(base, delta):
            return base + random.uniform(-delta, delta)

        # Atmospheric (Singapore, tropical, near sea level)
        self._atmos_temp_label.setText(f"Temp: {jitter(31.8, 0.15):.1f} °C")
        self._atmos_pressure_label.setText(f"Pressure: {jitter(1009.5, 0.1):.1f} hPa")
        self._atmos_humidity_label.setText(f"Humidity: {jitter(82.4, 0.3):.1f}%")

        # Soil Probe (tropical laterite soil)
        self._soil_moisture_label.setText(f"Moisture: {jitter(38.5, 0.2):.1f}%")
        self._soil_ph_label.setText(f"pH: {jitter(4.9, 0.02):.2f}")
        self._soil_temp_label.setText(f"Temp: {jitter(28.6, 0.1):.1f} °C")
        self._soil_conductivity_label.setText(f"Conductivity: {jitter(246, 3):.0f} µS/cm")
        self._soil_nitrogen_label.setText(f"Nitrogen: {jitter(143, 2):.0f} mg/kg")
        self._soil_phosphorus_label.setText(f"Phosphorus: {jitter(31, 1):.0f} mg/kg")
        self._soil_potassium_label.setText(f"Potassium: {jitter(118, 2):.0f} mg/kg")

    def _init_ui(self):
        """Initialize the dashboard layout."""
        # Main Layout: Vertical (Top Row, Bottom Row)
        main_layout = QVBoxLayout()
        
        # --- Top Row: Video Feed + Data ---
        top_row = QHBoxLayout()
        
        # 1. Single Video Feed (Left) with switch button
        video_section = QVBoxLayout()
        
        self.video_viewer = VideoViewer(camera_name="Aux Probe Feed")
        self.video_viewer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        video_section.addWidget(self.video_viewer, stretch=1)
        
        # Button row: Connect + Switch Feed
        btn_row = QHBoxLayout()
        btn_row.setSpacing(5)
        
        self._webcam_btn = QPushButton("🎥 Connect Webcam")
        self._webcam_btn.setFixedHeight(28)
        self._webcam_btn.clicked.connect(self._toggle_webcam)
        btn_row.addWidget(self._webcam_btn)
        
        self._switch_feed_btn = QPushButton("🔄 Switch to Ground Feed")
        self._switch_feed_btn.setFixedHeight(28)
        self._switch_feed_btn.clicked.connect(self._switch_feed)
        btn_row.addWidget(self._switch_feed_btn)
        
        video_section.addLayout(btn_row)
        
        top_row.addLayout(video_section, stretch=7)
        
        # 2. Data Displays (Right) - Single column: Atmospheric stacked above Soil
        data_column_layout = QVBoxLayout()
        data_column_layout.setContentsMargins(0, 0, 0, 0)
        
        # Atmospheric Data
        atmos_group = QGroupBox("Atmospheric Data")
        atmos_inner = QVBoxLayout()
        self._atmos_temp_label = QLabel("Temp: 31.8 °C")
        self._atmos_pressure_label = QLabel("Pressure: 1009.5 hPa")
        self._atmos_humidity_label = QLabel("Humidity: 82.4%")
        atmos_inner.addWidget(self._atmos_temp_label)
        atmos_inner.addWidget(self._atmos_pressure_label)
        atmos_inner.addWidget(self._atmos_humidity_label)
        atmos_group.setLayout(atmos_inner)
        data_column_layout.addWidget(atmos_group)
        
        # Soil Probe Data
        soil_group = QGroupBox("Soil Probe Data")
        soil_inner = QVBoxLayout()
        self._soil_moisture_label = QLabel("Moisture: 38.5%")
        self._soil_ph_label = QLabel("pH: 4.90")
        self._soil_temp_label = QLabel("Temp: 28.6 °C")
        self._soil_conductivity_label = QLabel("Conductivity: 246 µS/cm")
        self._soil_nitrogen_label = QLabel("Nitrogen: 143 mg/kg")
        self._soil_phosphorus_label = QLabel("Phosphorus: 31 mg/kg")
        self._soil_potassium_label = QLabel("Potassium: 118 mg/kg")
        soil_inner.addWidget(self._soil_moisture_label)
        soil_inner.addWidget(self._soil_ph_label)
        soil_inner.addWidget(self._soil_temp_label)
        soil_inner.addWidget(self._soil_conductivity_label)
        soil_inner.addWidget(self._soil_nitrogen_label)
        soil_inner.addWidget(self._soil_phosphorus_label)
        soil_inner.addWidget(self._soil_potassium_label)
        soil_group.setLayout(soil_inner)
        data_column_layout.addWidget(soil_group)
        
        data_column_layout.addStretch()
        
        # Container for data column
        data_container = QWidget()
        data_container.setLayout(data_column_layout)
        top_row.addWidget(data_container, stretch=2)
        
        main_layout.addLayout(top_row, stretch=2) # Top row gets more vertical space
        
        # --- Bottom Row: Controls + Log ---
        bottom_row = QHBoxLayout()
        
        # 1. Controls (Left) - Vertical Stack of (Cache/Drill) and (Manual)
        controls_layout = QVBoxLayout()
        controls_layout.setSpacing(5)
        controls_layout.setContentsMargins(0, 0, 0, 0)
        
        # Row 1 of Controls: Cache & Drill Depth
        ctrl_row1 = QHBoxLayout()
        ctrl_row1.setContentsMargins(0, 0, 0, 0)
        
        # Cache Control
        cache_group = QGroupBox("Cache")
        cache_inner = QHBoxLayout()
        cache_inner.setContentsMargins(5, 5, 5, 5) # Compact margins
        self.cache_open_btn = QPushButton("Open")
        self.cache_open_btn.clicked.connect(self._on_cache_open)
        self.cache_close_btn = QPushButton("Close")
        self.cache_close_btn.clicked.connect(self._on_cache_close)
        self.cache_indicator = QLabel()
        self.cache_indicator.setFixedSize(20, 20)
        self.cache_indicator.setStyleSheet("background-color: #222222; border-radius: 10px; border: 1px solid black;")

        cache_inner.addWidget(self.cache_open_btn)
        cache_inner.addWidget(self.cache_close_btn)
        cache_inner.addWidget(self.cache_indicator)
        cache_group.setLayout(cache_inner)
        ctrl_row1.addWidget(cache_group)
        
        # Drill Depth
        drill_depth_frame = QFrame()
        drill_depth_frame.setFrameShape(QFrame.StyledPanel)
        drill_depth_frame.setFrameShadow(QFrame.Raised)
        drill_depth_frame.setStyleSheet("QFrame { border: 1px solid #C0C0C0; border-radius: 3px; }")

        drill_depth_layout = QVBoxLayout()
        drill_depth_layout.setContentsMargins(2, 2, 2, 2) # Very compact margins
        self.drill_depth_label = QLabel("Drill Depth: 11.8 cm")
        self.drill_depth_label.setStyleSheet("border: none; font-weight: bold;")
        self.drill_depth_label.setAlignment(Qt.AlignCenter)
        drill_depth_layout.addWidget(self.drill_depth_label)
        drill_depth_frame.setLayout(drill_depth_layout)
        ctrl_row1.addWidget(drill_depth_frame)
        
        controls_layout.addLayout(ctrl_row1)
        
        # Row 2 of Controls: Manual Control Group
        manual_group = QGroupBox("Manual Control")
        manual_layout = QGridLayout()
        manual_layout.setContentsMargins(5, 5, 5, 5) # Compact margins
        manual_layout.setVerticalSpacing(5)
        
        self._la_rev_btn = QPushButton("REV")
        self._la_stop_btn = QPushButton("STOP")
        self._la_fwd_btn = QPushButton("FWD")
        self._la_active_btn: Optional[QPushButton] = None
        self._la_rev_btn.clicked.connect(self._on_la_rev)
        self._la_stop_btn.clicked.connect(self._on_la_stop)
        self._la_fwd_btn.clicked.connect(self._on_la_fwd)

        manual_layout.addWidget(QLabel("Linear Actuator"), 0, 0)
        manual_layout.addWidget(self._la_rev_btn, 0, 1)
        manual_layout.addWidget(self._la_stop_btn, 0, 2)
        manual_layout.addWidget(self._la_fwd_btn, 0, 3)
        
        self._drill_rev_btn = QPushButton("REV")
        self._drill_stop_btn = QPushButton("STOP")
        self._drill_fwd_btn = QPushButton("FWD")
        self._drill_active_btn: Optional[QPushButton] = None
        self._drill_rev_btn.clicked.connect(self._on_drill_rev)
        self._drill_stop_btn.clicked.connect(self._on_drill_stop)
        self._drill_fwd_btn.clicked.connect(self._on_drill_fwd)

        manual_layout.addWidget(QLabel("Drill"), 1, 0)
        manual_layout.addWidget(self._drill_rev_btn, 1, 1)
        manual_layout.addWidget(self._drill_stop_btn, 1, 2)
        manual_layout.addWidget(self._drill_fwd_btn, 1, 3)
        
        start_drill_btn = QPushButton("Start Drilling")
        start_drill_btn.setStyleSheet("background-color: #90EE90; font-weight: bold;")
        start_drill_btn.clicked.connect(self._on_start_drilling)
        retract_app_btn = QPushButton("Retract Apparatus")
        retract_app_btn.setStyleSheet("background-color: #F08080; font-weight: bold;")
        retract_app_btn.clicked.connect(self._on_retract_apparatus)

        manual_layout.addWidget(start_drill_btn, 2, 0, 1, 2)
        manual_layout.addWidget(retract_app_btn, 2, 2, 1, 2)
        
        manual_group.setLayout(manual_layout)
        controls_layout.addWidget(manual_group)
        
        # Container for controls to manage layout
        controls_container = QWidget()
        controls_container.setLayout(controls_layout)
        bottom_row.addWidget(controls_container, stretch=2)
        
        # 2. Log Viewer (Right)
        log_group = QGroupBox("Log")
        log_layout = QVBoxLayout()
        self.log_viewer = LogViewer()
        log_layout.addWidget(self.log_viewer)
        log_group.setLayout(log_layout)
        bottom_row.addWidget(log_group, stretch=3) # Log is wider than controls
        
        main_layout.addLayout(bottom_row, stretch=1)
        
        self.setLayout(main_layout)

    @property
    def _active_camera_index(self) -> int:
        """Return the camera index for the currently selected feed."""
        if self._active_feed == self.FEED_GROUND:
            return self.GROUND_CAMERA_INDEX
        return self.AUX_CAMERA_INDEX

    @property
    def _active_feed_name(self) -> str:
        if self._active_feed == self.FEED_GROUND:
            return "Ground Feed"
        return "Aux Probe Feed"

    def _do_connect(self) -> None:
        """Actually instantiate and start the camera for the active feed."""
        try:
            if self._active_feed == self.FEED_GROUND:
                cam = VideoFileCamera(path=self.GROUND_FEED_VIDEO)
            else:
                import cv2 as _cv2
                cam = StaticImageCamera(path=self.AUX_FEED_IMAGE, rotate=_cv2.ROTATE_90_CLOCKWISE)
            ActiveCameraManager.start_camera(cam, self.video_viewer)
            self.video_viewer.attach_camera(cam)
            self._webcam_source = cam
            self._webcam_btn.setText("⏹ Disconnect Webcam")
            self.log_viewer.append(f"Connected to {self._active_feed_name}")
        except Exception as e:
            self._webcam_btn.setText("🎥 Connect Webcam")
            self.log_viewer.append(f"Failed to connect: {e}")

    def _toggle_webcam(self) -> None:
        """Connect or disconnect the webcam for the currently active feed."""
        if self._webcam_source is not None:
            ActiveCameraManager.stop_camera(self._webcam_source)
            self.video_viewer.attach_camera(None)
            self._webcam_source = None
            self._webcam_btn.setText("🎥 Connect Webcam")
            self.log_viewer.append(f"Disconnected from {self._active_feed_name}")
        else:
            self._webcam_btn.setEnabled(False)
            self.log_viewer.append(f"Connecting to {self._active_feed_name}...")
            self.video_viewer._show_text_placeholder_message(f"{self._active_feed_name}\nConnecting...")
            QTimer.singleShot(2000, self._on_connect_delay)

    def _on_connect_delay(self) -> None:
        """Called after the 2-second connection delay."""
        self._webcam_btn.setEnabled(True)
        self._do_connect()

    def _switch_feed(self) -> None:
        """Switch the viewer between Aux Probe Feed and Ground Feed."""
        was_streaming = self._webcam_source is not None

        # Stop current camera if running
        if was_streaming:
            ActiveCameraManager.stop_camera(self._webcam_source)
            self.video_viewer.attach_camera(None)
            self._webcam_source = None

        # Toggle the active feed
        if self._active_feed == self.FEED_AUX:
            self._active_feed = self.FEED_GROUND
            self._switch_feed_btn.setText("🔄 Switch to Aux Probe Feed")
        else:
            self._active_feed = self.FEED_AUX
            self._switch_feed_btn.setText("🔄 Switch to Ground Feed")

        self.video_viewer.set_camera_name(self._active_feed_name)
        self.log_viewer.append(f"Switched to {self._active_feed_name}")

        # Reconnect on the new camera if we were streaming
        if was_streaming:
            self._webcam_btn.setEnabled(False)
            self.log_viewer.append(f"Connecting to {self._active_feed_name}...")
            self.video_viewer._show_text_placeholder_message(f"{self._active_feed_name}\nConnecting...")
            QTimer.singleShot(2000, self._on_connect_delay)
        else:
            self._webcam_btn.setText("🎥 Connect Webcam")

    _BTN_PRESSED_STYLE = (
        "border-radius: 4px;"
        "border-top: 2px solid #777;"
        "border-left: 2px solid #777;"
        "border-bottom: 2px solid #d0d0d0;"
        "border-right: 2px solid #d0d0d0;"
        "background-color: #a8a8a8;"
        "color: #222;"
        "padding-top: 3px;"
        "padding-left: 3px;"
    )

    def _press_button_briefly(self, btn: QPushButton, on_release=None) -> None:
        """Indent a button for 1 second, then restore and fire on_release."""
        btn.setStyleSheet(self._BTN_PRESSED_STYLE)
        btn.setEnabled(False)

        def _restore():
            btn.setStyleSheet("")
            btn.setEnabled(True)
            if on_release:
                on_release()

        QTimer.singleShot(1000, _restore)

    def _on_cache_open(self) -> None:
        self.log_viewer.append("Cache open command sent")

        def _on_release():
            self.cache_indicator.setStyleSheet(
                "background-color: #222222; border-radius: 10px; border: 1px solid black;"
            )
            self.log_viewer.append("Cache door opened")

        self._press_button_briefly(self.cache_open_btn, on_release=_on_release)

    def _on_cache_close(self) -> None:
        self.log_viewer.append("Cache close command sent")

        def _on_release():
            self.cache_indicator.setStyleSheet(
                "background-color: #00FF00; border-radius: 10px; border: 1px solid black;"
            )
            self.log_viewer.append("Cache door closed")

        self._press_button_briefly(self.cache_close_btn, on_release=_on_release)

    _LA_ACTIVE_STYLE = (
        "background-color: #3a8ee6;"
        "color: white;"
        "font-weight: bold;"
        "border-radius: 4px;"
        "border: 1px solid #1a6ec4;"
    )

    def _la_set_active(self, btn: Optional[QPushButton]) -> None:
        """Highlight btn as the active direction; clear the previous one."""
        if self._la_active_btn is not None:
            self._la_active_btn.setStyleSheet("")
        self._la_active_btn = btn
        if btn is not None:
            btn.setStyleSheet(self._LA_ACTIVE_STYLE)

    def _on_la_rev(self) -> None:
        self.log_viewer.append("Linear actuator reverse command sent")

        def _on_release():
            self._la_set_active(self._la_rev_btn)
            self._la_direction = -1
            self._depth_timer.start()
            self.log_viewer.append("Linear actuator reversing")

        self._press_button_briefly(self._la_rev_btn, on_release=_on_release)

    def _on_la_stop(self) -> None:
        self.log_viewer.append("Linear actuator stop command sent")

        def _on_release():
            self._la_set_active(None)
            self._la_direction = 0
            self._depth_timer.stop()
            self.log_viewer.append("Linear actuator stopped")

        self._press_button_briefly(self._la_stop_btn, on_release=_on_release)

    def _on_la_fwd(self) -> None:
        self.log_viewer.append("Linear actuator forward command sent")

        def _on_release():
            self._la_set_active(self._la_fwd_btn)
            self._la_direction = 1
            self._depth_timer.start()
            self.log_viewer.append("Linear actuator moving forward")

        self._press_button_briefly(self._la_fwd_btn, on_release=_on_release)

    def _drill_set_active(self, btn: Optional[QPushButton]) -> None:
        if self._drill_active_btn is not None:
            self._drill_active_btn.setStyleSheet("")
        self._drill_active_btn = btn
        if btn is not None:
            btn.setStyleSheet(self._LA_ACTIVE_STYLE)

    def _on_drill_rev(self) -> None:
        self.log_viewer.append("Drill reverse command sent")

        def _on_release():
            self._drill_set_active(self._drill_rev_btn)
            self.log_viewer.append("Drill reversing")

        self._press_button_briefly(self._drill_rev_btn, on_release=_on_release)

    def _on_drill_stop(self) -> None:
        self.log_viewer.append("Drill stop command sent")

        def _on_release():
            self._drill_set_active(None)
            self.log_viewer.append("Drill stopped")

        self._press_button_briefly(self._drill_stop_btn, on_release=_on_release)

    def _on_drill_fwd(self) -> None:
        self.log_viewer.append("Drill forward command sent")

        def _on_release():
            self._drill_set_active(self._drill_fwd_btn)
            self.log_viewer.append("Drill moving forward")

        self._press_button_briefly(self._drill_fwd_btn, on_release=_on_release)

    def _on_start_drilling(self) -> None:
        self._on_la_fwd()
        self._on_drill_fwd()

    def _on_retract_apparatus(self) -> None:
        self._on_la_rev()
        self._on_drill_rev()
