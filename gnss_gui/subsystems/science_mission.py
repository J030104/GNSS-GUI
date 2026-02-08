"""Placeholder widget for the science mission subsystem.

Shows dummy sensor readings and a log viewer.  Teams can integrate
real instruments (e.g. spectrometers, microscopes) and control
interfaces in this tab.
"""

from typing import Optional

from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QTabWidget,
    QGroupBox,
    QPushButton,
    QGridLayout,
    QFrame,
    QSizePolicy,
    QSlider,
    QRadioButton
)
from PyQt5.QtCore import Qt

from ..components.video_viewer import VideoViewer
from ..components.log_viewer import LogViewer


class ScienceMissionWidget(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.sub_tabs = QTabWidget()
        
        # specific style for the main tabs
        main_tab_style = """
            QTabBar::tab {
                height: 40px;
                width: 150px;
                font-size: 14px;
                font-weight: bold;
            }
            QTabWidget::pane {
                border: 1px solid #C4C4C3;
                top: -1px;
            }
        """
        self.sub_tabs.setStyleSheet(main_tab_style)
        
        # specific style for the nested tabs (wider)
        nested_tab_style = """
            QTabBar::tab {
                height: 30px;
                width: 200px; /* Wider for full text visibility */
                font-size: 12px;
                font-weight: bold;
            }
            QTabWidget::pane {
                border: 1px solid #C4C4C3;
                top: -1px;
            }
        """
        
        # --- Dashboard Tab ---
        self.dashboard_tab = QWidget()
        self._init_dashboard_ui()
        
        # --- Test Site 1 ---
        self.test_site_1_tab = QWidget()
        ts1_layout = QVBoxLayout()
        ts1_layout.setContentsMargins(0, 0, 0, 0)
        
        self.ts1_tabs = QTabWidget()
        self.ts1_tabs.setStyleSheet(nested_tab_style)
        
        # TS1: UV Fluorescence
        self.ts1_uv_tab = QWidget()
        self._setup_uv_tab(self.ts1_uv_tab, "Site 1")
        
        # TS1: Raman Spectroscopy
        self.ts1_raman_tab = QWidget()
        raman1_layout = QVBoxLayout()
        raman1_layout.addWidget(QLabel("Raman Spectroscopy (Site 1)"))
        self.ts1_raman_tab.setLayout(raman1_layout)
        
        self.ts1_tabs.addTab(self.ts1_uv_tab, "UV Fluorescence")
        self.ts1_tabs.addTab(self.ts1_raman_tab, "Raman Spectroscopy")
        
        ts1_layout.addWidget(self.ts1_tabs)
        self.test_site_1_tab.setLayout(ts1_layout)
        
        # --- Test Site 2 ---
        self.test_site_2_tab = QWidget()
        ts2_layout = QVBoxLayout()
        ts2_layout.setContentsMargins(0, 0, 0, 0)
        
        self.ts2_tabs = QTabWidget()
        self.ts2_tabs.setStyleSheet(nested_tab_style)

        # TS2: UV Fluorescence
        self.ts2_uv_tab = QWidget()
        self._setup_uv_tab(self.ts2_uv_tab, "Site 2")
        
        # TS2: Raman Spectroscopy
        self.ts2_raman_tab = QWidget()
        raman2_layout = QVBoxLayout()
        raman2_layout.addWidget(QLabel("Raman Spectroscopy (Site 2)"))
        self.ts2_raman_tab.setLayout(raman2_layout)
        
        self.ts2_tabs.addTab(self.ts2_uv_tab, "UV Fluorescence")
        self.ts2_tabs.addTab(self.ts2_raman_tab, "Raman Spectroscopy")
        
        ts2_layout.addWidget(self.ts2_tabs)
        self.test_site_2_tab.setLayout(ts2_layout)
        
        self.sub_tabs.addTab(self.dashboard_tab, "Dashboard")
        self.sub_tabs.addTab(self.test_site_1_tab, "Test Site 1")
        self.sub_tabs.addTab(self.test_site_2_tab, "Test Site 2")
        
        layout.addWidget(self.sub_tabs)
        self.setLayout(layout)

    def _setup_uv_tab(self, tab_widget: QWidget, site_name: str):
        """Setup the layout for a UV Fluorescence tab."""
        main_layout = QVBoxLayout()
        
        # Top: Video Feed
        video_viewer = VideoViewer(camera_name=f"UV Fluorescence Feed ({site_name})")
        video_viewer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        main_layout.addWidget(video_viewer, stretch=3)
        
        # Bottom: Controls + Log
        bottom_row = QHBoxLayout()
        
        # Left: Controls
        controls_col = QVBoxLayout()
        controls_col.setSpacing(5)
        controls_col.setContentsMargins(0, 0, 0, 0)
        
        # Row 1: Sheaths
        sheaths_row = QHBoxLayout()
        
        # Sheath 1
        sheath1_group = QGroupBox("Cache Docking Sheath 1")
        s1_layout = QHBoxLayout()
        s1_layout.setContentsMargins(5, 5, 5, 5)
        btn_on1 = QPushButton("On")
        btn_on1.setFixedWidth(40)
        btn_off1 = QPushButton("Off")
        btn_off1.setFixedWidth(40)
        s1_layout.addWidget(btn_on1)
        s1_layout.addWidget(btn_off1)
        ind1 = QLabel()
        ind1.setFixedSize(25, 25)
        ind1.setStyleSheet("background-color: #00FF00; border-radius: 12px; border: 1px solid black;")
        s1_layout.addWidget(ind1)
        sheath1_group.setLayout(s1_layout)
        sheaths_row.addWidget(sheath1_group)
        
        # Sheath 2
        sheath2_group = QGroupBox("Cache Docking Sheath 2")
        s2_layout = QHBoxLayout()
        s2_layout.setContentsMargins(5, 5, 5, 5)
        btn_on2 = QPushButton("On")
        btn_on2.setFixedWidth(40)
        btn_off2 = QPushButton("Off")
        btn_off2.setFixedWidth(40)
        s2_layout.addWidget(btn_on2)
        s2_layout.addWidget(btn_off2)
        ind2 = QLabel()
        ind2.setFixedSize(25, 25)
        ind2.setStyleSheet("background-color: #00FF00; border-radius: 12px; border: 1px solid black;")
        s2_layout.addWidget(ind2)
        sheath2_group.setLayout(s2_layout)
        sheaths_row.addWidget(sheath2_group)
        
        controls_col.addLayout(sheaths_row)
        
        # Row 2: LED & Switch Button
        led_row = QHBoxLayout()
        
        # LED Switch
        led_group = QGroupBox("LED switch")
        led_layout = QGridLayout()
        led_layout.setContentsMargins(5, 5, 5, 5)
        
        # LED Switch
        led_group = QGroupBox("LED switch")
        led_layout = QGridLayout()
        led_layout.setContentsMargins(5, 5, 5, 5)
        
        # Filter Switcher Status (Row 0)
        # Layout: [Radio 340nm] [Radio 450nm]
        filter_layout = QHBoxLayout()
        filter_layout.setContentsMargins(0, 0, 0, 0)
        
        rb_340 = QRadioButton("340nm")
        rb_450 = QRadioButton("450nm")
        rb_340.setChecked(True)
        # Assuming user doesn't want them clickable, but for UI we just show them.
        
        filter_layout.addWidget(rb_340)
        filter_layout.addWidget(rb_450)
        filter_layout.addStretch()
        
        led_layout.addLayout(filter_layout, 0, 0, 1, 3)
        
        # LED On/Off + Switch (Row 1)
        power_layout = QHBoxLayout()
        power_layout.setContentsMargins(0, 0, 0, 0)
        
        btn_on_led = QPushButton("On")
        btn_on_led.setFixedWidth(40)
        btn_off_led = QPushButton("Off")
        btn_off_led.setFixedWidth(40)
        
        power_layout.addWidget(btn_on_led)
        power_layout.addWidget(btn_off_led)
        
        ind_led = QLabel()
        ind_led.setFixedSize(25, 25)
        ind_led.setStyleSheet("background-color: #00FF00; border-radius: 12px; border: 1px solid black;")
        power_layout.addWidget(ind_led)
        
        # Switch Button (moved here)
        btn_switch_filter = QPushButton("Switch")
        btn_switch_filter.setFixedWidth(90)
        btn_switch_filter.setStyleSheet("background-color: #4287f5; color: white; font-weight: bold;")
        power_layout.addWidget(btn_switch_filter)
        
        power_layout.addStretch()
        
        led_layout.addLayout(power_layout, 1, 0, 1, 3)
        
        led_group.setLayout(led_layout)
        led_row.addWidget(led_group)
        
        # Right Side Buttons (Begin Test + Switch to Raman)
        right_btns_layout = QVBoxLayout()
        right_btns_layout.setContentsMargins(0, 0, 0, 0)
        
        # Begin Test Button
        btn_begin_test = QPushButton("Begin Test")
        btn_begin_test.setStyleSheet("background-color: #90EE90; font-weight: bold; padding: 10px;")
        right_btns_layout.addWidget(btn_begin_test)
        
        # Switch Button
        switch_btn = QPushButton("Switch to Raman\nSpectroscopy ->")
        switch_btn.setStyleSheet("background-color: #F08080; font-weight: bold; padding: 10px;")
        right_btns_layout.addWidget(switch_btn)
        
        led_row.addLayout(right_btns_layout)
        
        controls_col.addLayout(led_row)
        
        # Container for controls
        controls_container = QWidget()
        controls_container.setLayout(controls_col)
        bottom_row.addWidget(controls_container, stretch=0) # Controls take minimum width
        
        # Right: Log
        log_group = QGroupBox("Log")
        log_layout = QVBoxLayout()
        log_viewer = LogViewer()
        log_layout.addWidget(log_viewer)
        log_group.setLayout(log_layout)
        bottom_row.addWidget(log_group, stretch=1) # Log takes remaining space (effectively wider)
        
        main_layout.addLayout(bottom_row, stretch=1)
        tab_widget.setLayout(main_layout)

    def _init_dashboard_ui(self):
        """Initialize the dashboard layout."""
        # Main Layout: Vertical (Top Row, Bottom Row)
        main_layout = QVBoxLayout()
        
        # --- Top Row: Video + Data ---
        top_row = QHBoxLayout()
        
        # 1. Video Viewer (Left)
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
        
        self.dashboard_tab.setLayout(main_layout)
