from typing import Optional

from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QGroupBox,
    QPushButton,
    QGridLayout,
    QSizePolicy,
    QRadioButton
)

from ...components.video_viewer import VideoViewer
from ...components.log_viewer import LogViewer


class UVFluorescenceTab(QWidget):
    def __init__(self, site_name: str, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.site_name = site_name
        self._setup_ui()

    def _setup_ui(self):
        """Setup the layout for a UV Fluorescence tab."""
        main_layout = QVBoxLayout()
        
        # Top: Video Feed
        video_viewer = VideoViewer(camera_name=f"UV Fluorescence Feed ({self.site_name})")
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
        self.setLayout(main_layout)
