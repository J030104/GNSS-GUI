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

from ...components.video_viewer import VideoViewer
from ...components.log_viewer import LogViewer
from ...utilities.video_streamer import LocalCamera

import os


class DashboardTab(QWidget):
    # Camera indices read from environment variables (fallback to defaults)
    AUX_CAMERA_INDEX = int(os.getenv("AUX_CAMERA_INDEX", "1"))
    GROUND_CAMERA_INDEX = int(os.getenv("GROUND_CAMERA_INDEX", "2"))

    # Feed modes
    FEED_AUX = "aux"
    FEED_GROUND = "ground"

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._webcam_source: Optional[LocalCamera] = None
        self._active_feed = self.FEED_AUX  # which feed is currently selected
        self._init_ui()
        self._start_data_simulation()

    def _start_data_simulation(self):
        """Start a 1-second timer to simulate live sensor data."""
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._update_sensor_data)
        self._timer.start(1000)

    def _update_sensor_data(self):
        """Update atmospheric and soil probe labels with small random fluctuations."""
        def jitter(base, delta):
            return base + random.uniform(-delta, delta)

        # Atmospheric
        self._atmos_temp_label.setText(f"Temp: {jitter(24.5, 0.3):.1f} °C")
        self._atmos_pressure_label.setText(f"Pressure: {jitter(1013, 0.5):.1f} hPa")
        self._atmos_humidity_label.setText(f"Humidity: {jitter(45, 0.5):.1f}%")

        # Soil Probe
        self._soil_moisture_label.setText(f"Moisture: {jitter(12, 0.2):.1f}%")
        self._soil_ph_label.setText(f"pH: {jitter(7.2, 0.05):.2f}")
        self._soil_temp_label.setText(f"Temp: {jitter(18.0, 0.2):.1f} °C")
        self._soil_conductivity_label.setText(f"Conductivity: {jitter(5790, 10):.0f} µS/cm")
        self._soil_nitrogen_label.setText(f"Nitrogen: {jitter(2000, 20):.0f} mg/kg")
        self._soil_phosphorus_label.setText(f"Phosphorus: {jitter(1500, 15):.0f} mg/kg")
        self._soil_potassium_label.setText(f"Potassium: {jitter(1800, 18):.0f} mg/kg")

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
        self._atmos_temp_label = QLabel("Temp: 24.5 °C")
        self._atmos_pressure_label = QLabel("Pressure: 1013.0 hPa")
        self._atmos_humidity_label = QLabel("Humidity: 45.0%")
        atmos_inner.addWidget(self._atmos_temp_label)
        atmos_inner.addWidget(self._atmos_pressure_label)
        atmos_inner.addWidget(self._atmos_humidity_label)
        atmos_group.setLayout(atmos_inner)
        data_column_layout.addWidget(atmos_group)
        
        # Soil Probe Data
        soil_group = QGroupBox("Soil Probe Data")
        soil_inner = QVBoxLayout()
        self._soil_moisture_label = QLabel("Moisture: 12.0%")
        self._soil_ph_label = QLabel("pH: 7.20")
        self._soil_temp_label = QLabel("Temp: 18.0 °C")
        self._soil_conductivity_label = QLabel("Conductivity: 5790 µS/cm")
        self._soil_nitrogen_label = QLabel("Nitrogen: 2000 mg/kg")
        self._soil_phosphorus_label = QLabel("Phosphorus: 1500 mg/kg")
        self._soil_potassium_label = QLabel("Potassium: 1800 mg/kg")
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
        self.cache_close_btn = QPushButton("Close")
        self.cache_indicator = QLabel()
        self.cache_indicator.setFixedSize(20, 20)
        self.cache_indicator.setStyleSheet("background-color: #00FF00; border-radius: 10px; border: 1px solid black;")
        
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
        self.drill_depth_label = QLabel("Drill Depth: 5.7 cm")
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
        
        manual_layout.addWidget(QLabel("Linear Actuator"), 0, 0)
        manual_layout.addWidget(QPushButton("REV"), 0, 1)
        manual_layout.addWidget(QPushButton("STOP"), 0, 2)
        manual_layout.addWidget(QPushButton("FWD"), 0, 3)
        
        manual_layout.addWidget(QLabel("Drill"), 1, 0)
        manual_layout.addWidget(QPushButton("REV"), 1, 1)
        manual_layout.addWidget(QPushButton("STOP"), 1, 2)
        manual_layout.addWidget(QPushButton("FWD"), 1, 3)
        
        start_drill_btn = QPushButton("Start Drilling")
        start_drill_btn.setStyleSheet("background-color: #90EE90; font-weight: bold;")
        retract_app_btn = QPushButton("Retract Apparatus")
        retract_app_btn.setStyleSheet("background-color: #F08080; font-weight: bold;")
        
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

    def _toggle_webcam(self) -> None:
        """Connect or disconnect the webcam for the currently active feed."""
        if self._webcam_source is not None:
            self._webcam_source.stop()
            self.video_viewer.attach_camera(None)
            self._webcam_source = None
            self._webcam_btn.setText("🎥 Connect Webcam")
            self.log_viewer.append(f"Webcam disconnected from {self._active_feed_name}")
        else:
            # Show "Connecting..." on the video viewer area
            self.video_viewer.label.setText(f"{self._active_feed_name}\nConnecting...")
            self.video_viewer._show_text_placeholder = True
            self.video_viewer.label.repaint()
            from PyQt5.QtWidgets import QApplication
            QApplication.processEvents()
            try:
                cam = LocalCamera(index=self._active_camera_index)
                cam.start()
                self.video_viewer.attach_camera(cam)
                self._webcam_source = cam
                self._webcam_btn.setText("⏹ Disconnect Webcam")
                self.log_viewer.append(
                    f"Camera {self._active_camera_index} connected to {self._active_feed_name}"
                )
            except Exception as e:
                self._webcam_btn.setText("🎥 Connect Webcam")
                self.video_viewer.set_camera_name(self._active_feed_name)
                self.log_viewer.append(f"Failed to connect webcam: {e}")

    def _switch_feed(self) -> None:
        """Switch the viewer between Aux Probe Feed and Ground Feed."""
        was_streaming = self._webcam_source is not None

        # Stop current camera if running
        if was_streaming:
            self._webcam_source.stop()
            self.video_viewer.attach_camera(None)
            self._webcam_source = None

        # Toggle the active feed
        if self._active_feed == self.FEED_AUX:
            self._active_feed = self.FEED_GROUND
            self._switch_feed_btn.setText("🔄 Switch to Aux Probe Feed")
        else:
            self._active_feed = self.FEED_AUX
            self._switch_feed_btn.setText("🔄 Switch to Ground Feed")

        # Update the viewer title
        self.video_viewer.set_camera_name(self._active_feed_name)
        self.log_viewer.append(f"Switched to {self._active_feed_name}")

        # Reconnect on the new camera if we were streaming
        if was_streaming:
            self.video_viewer.label.setText(f"{self._active_feed_name}\nConnecting...")
            self.video_viewer._show_text_placeholder = True
            self.video_viewer.label.repaint()
            from PyQt5.QtWidgets import QApplication
            QApplication.processEvents()
            try:
                cam = LocalCamera(index=self._active_camera_index)
                cam.start()
                self.video_viewer.attach_camera(cam)
                self._webcam_source = cam
                self._webcam_btn.setText("⏹ Disconnect Webcam")
                self.log_viewer.append(
                    f"Camera {self._active_camera_index} connected to {self._active_feed_name}"
                )
            except Exception as e:
                self._webcam_btn.setText("🎥 Connect Webcam")
                self.log_viewer.append(f"Failed to connect webcam: {e}")
        else:
            self._webcam_btn.setText("🎥 Connect Webcam")

