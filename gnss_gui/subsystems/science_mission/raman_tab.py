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
    QFrame,
)
from PyQt5.QtCore import Qt

from ...components.video_viewer import VideoViewer
from ...components.log_viewer import LogViewer
from ...components.spectral_plot import SpectralPlotWidget


class RamanSpectroscopyTab(QWidget):
    def __init__(self, site_name: str, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.site_name = site_name
        self._init_ui()

    def _init_ui(self):
        """Initialize the Raman Spectroscopy layout."""
        main_layout = QVBoxLayout()
        
        # --- Top Row: Video + Graph ---
        top_row = QHBoxLayout()
        
        # 1. Video Viewer (Left)
        # Placeholder text: "Raman Feed Unavailable"
        self.video_viewer = VideoViewer(camera_name=f"Raman Feed ({self.site_name})")
        # For prototype, force the "Unavailable" message by not attaching a source
        # But we can set the name to appear in the box
        self.video_viewer.set_camera_name("Raman Feed\nUnavailable")
        self.video_viewer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        top_row.addWidget(self.video_viewer, stretch=2)
        
        # 2. Spectral Plot (Right)
        # Using our custom widget
        self.plot_widget = SpectralPlotWidget() # No title, dynamic data
        self.plot_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        # Increase stretch to make graph wider relative to video
        top_row.addWidget(self.plot_widget, stretch=3)
        
        main_layout.addLayout(top_row, stretch=2)
        
        # --- Bottom Row: Controls + Log ---
        bottom_row = QHBoxLayout()
        
        # 1. Controls (Left)
        controls_col = QVBoxLayout()
        controls_col.setSpacing(10)
        controls_col.setContentsMargins(0, 0, 0, 0)
        
        # Laser & Power Row
        laser_power_row = QHBoxLayout()
        
        # Laser Control Group
        laser_group = QGroupBox("532nm Laser")
        laser_layout = QHBoxLayout()
        laser_layout.setContentsMargins(5, 5, 5, 5)
        
        self.btn_laser_on = QPushButton("On")
        self.btn_laser_on.setFixedWidth(40)
        self.btn_laser_off = QPushButton("Off")
        self.btn_laser_off.setFixedWidth(40)
        
        self.laser_indicator = QLabel()
        self.laser_indicator.setFixedSize(25, 25)
        # Default off (grey or black border? Screenshot has green circle which implies ON or Ready? 
        # Making it green for visual match, or grey if off. Let's make it Green to match screenshot style)
        self.laser_indicator.setStyleSheet("background-color: #00FF00; border-radius: 12px; border: 1px solid black;")

        laser_layout.addWidget(self.btn_laser_on)
        laser_layout.addWidget(self.btn_laser_off)
        laser_layout.addWidget(self.laser_indicator)
        laser_group.setLayout(laser_layout)
        laser_power_row.addWidget(laser_group)
        
        # Power Meter Group
        power_group = QFrame() # Using frame to look like a box
        power_group.setFrameShape(QFrame.StyledPanel)
        power_group.setStyleSheet("QFrame { border: 1px solid #C0C0C0; background-color: white; }")
        power_layout = QVBoxLayout()
        power_layout.setAlignment(Qt.AlignCenter)
        # Smaller font for Power Meter to be more compact
        power_lbl = QLabel("Power Meter:   67%")
        power_lbl.setStyleSheet("border: none; font-weight: bold; font-size: 10pt;")
        power_layout.addWidget(power_lbl)
        power_group.setLayout(power_layout)
        # Fix height to prevent it from squashing buttons
        power_group.setFixedHeight(60) 
        laser_power_row.addWidget(power_group)
        
        controls_col.addLayout(laser_power_row)
        
        # Buttons Row (Begin Test / Return)
        btns_row = QHBoxLayout()
        
        self.btn_begin_test = QPushButton("Begin Test")
        # Match UV Tab color: #90EE90
        self.btn_begin_test.setStyleSheet("background-color: #90EE90; font-weight: bold; font-size: 9pt; padding: 10px;") 
        # Increase height slightly or let it expand? Fixed 60 should be enough for 2 lines if needed, but "Begin Test" is short.
        self.btn_begin_test.setFixedHeight(60)
        btns_row.addWidget(self.btn_begin_test)
        
        self.btn_return = QPushButton("Return to\nDashboard ->")
        # Match UV Tab color: #F08080
        self.btn_return.setStyleSheet("background-color: #F08080; font-weight: bold; font-size: 8pt; padding: 5px;") 
        # Increase height to accommodate 2 lines of text comfortably
        self.btn_return.setFixedHeight(60)
        btns_row.addWidget(self.btn_return)
        
        controls_col.addLayout(btns_row)
        
        # Container for controls
        controls_container = QWidget()
        controls_container.setLayout(controls_col)
        # Use stretch 0 (minimum size) to let buttons take space they need but no more, giving room for others if needed
        # But here we want to ensure it has enough width for text.
        bottom_row.addWidget(controls_container, stretch=0)
        
        # 2. Log Viewer (Right)
        log_group = QGroupBox("Log")
        log_layout = QVBoxLayout()
        self.log_viewer = LogViewer()
        log_layout.addWidget(self.log_viewer)
        log_group.setLayout(log_layout)
        bottom_row.addWidget(log_group, stretch=2) 
        
        main_layout.addLayout(bottom_row, stretch=1)
        
        self.setLayout(main_layout)
