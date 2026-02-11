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
from PyQt5.QtCore import Qt

from ...components.video_viewer import VideoViewer
from ...components.log_viewer import LogViewer


class DashboardTab(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._init_ui()

    def _init_ui(self):
        """Initialize the dashboard layout."""
        # Main Layout: Vertical (Top Row, Bottom Row)
        main_layout = QVBoxLayout()
        
        # --- Top Row: Video + Data ---
        top_row = QHBoxLayout()
        
        # 1. Video Viewer (Left)
        # Note: Added simple file logger for now or use main log
        self.video_viewer = VideoViewer(camera_name="Aux Probe Feed")
        self.video_viewer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        top_row.addWidget(self.video_viewer, stretch=3)
        
        # 2. Data Displays (Right) - Adjacent Columns
        data_row_layout = QHBoxLayout()
        data_row_layout.setContentsMargins(0, 0, 0, 0)
        
        # Atmospheric Data
        atmos_group = QGroupBox("Atmospheric Data")
        atmos_inner = QVBoxLayout()
        # Add placeholders for data
        atmos_inner.addWidget(QLabel("Temp: 24.5 °C"))
        atmos_inner.addWidget(QLabel("Pressure: 1013 hPa"))
        atmos_inner.addWidget(QLabel("Humidity: 45%"))
        atmos_inner.addStretch()
        atmos_group.setLayout(atmos_inner)
        data_row_layout.addWidget(atmos_group)
        
        # Soil Probe Data
        soil_group = QGroupBox("Soil Probe Data")
        soil_inner = QVBoxLayout()
        # Add placeholders for data
        soil_inner.addWidget(QLabel("Moisture: 12%"))
        soil_inner.addWidget(QLabel("pH: 7.2"))
        soil_inner.addWidget(QLabel("Temp: 18.0 °C"))
        soil_inner.addStretch()
        soil_group.setLayout(soil_inner)
        data_row_layout.addWidget(soil_group)
        
        # Container for data to manage width
        data_container = QWidget()
        data_container.setLayout(data_row_layout)
        top_row.addWidget(data_container, stretch=1)
        
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
