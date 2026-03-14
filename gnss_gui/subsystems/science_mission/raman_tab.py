from typing import Optional
from pathlib import Path

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
from PyQt5.QtCore import Qt, QTimer

from ...components.video_viewer import VideoViewer
from ...components.log_viewer import LogViewer
from ...components.spectral_plot import SpectralPlotWidget
from ...utilities.video_streamer import VideoFileCamera, ActiveCameraManager


class RamanSpectroscopyTab(QWidget):
    # Temporary Raman spectroscopy video file
    RAMAN_FEED_VIDEO = str(Path(__file__).resolve().parents[3] / "assets" / "videos" / "raman_spec.mp4")

    def __init__(self, site_name: str, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.site_name = site_name
        self._camera_source: Optional[VideoFileCamera] = None
        self._init_ui()

    def _init_ui(self):
        """Initialize the Raman Spectroscopy layout."""
        main_layout = QVBoxLayout()
        
        # --- Top Row: Video + Graph ---
        top_row = QHBoxLayout()
        
        # 1. Video Viewer (Left)
        video_section = QVBoxLayout()
        self.video_viewer = VideoViewer(camera_name=f"Raman Feed ({self.site_name})")
        self.video_viewer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        video_section.addWidget(self.video_viewer, stretch=1)

        # Camera toggle button
        self._camera_btn = QPushButton("🎥 Connect Raman Camera")
        self._camera_btn.setFixedHeight(28)
        self._camera_btn.clicked.connect(self._toggle_camera)
        video_section.addWidget(self._camera_btn)

        top_row.addLayout(video_section, stretch=2)
        
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
        self.btn_laser_on.clicked.connect(self._on_laser_on)
        self.btn_laser_off = QPushButton("Off")
        self.btn_laser_off.setFixedWidth(40)
        self.btn_laser_off.clicked.connect(self._on_laser_off)

        self.laser_indicator = QLabel()
        self.laser_indicator.setFixedSize(25, 25)
        self.laser_indicator.setStyleSheet("background-color: #222222; border-radius: 12px; border: 1px solid black;")

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
        self._power_lbl = QLabel("Power Meter:   0%")
        self._power_lbl.setStyleSheet("border: none; font-weight: bold; font-size: 10pt;")
        power_layout.addWidget(self._power_lbl)
        power_group.setLayout(power_layout)

        self._power_value = 0
        self._power_direction = 0  # 1 = going up, -1 = going down
        self._power_timer = QTimer(self)
        self._power_timer.setInterval(8)  # 100 steps × 8 ms = 0.8 s
        self._power_timer.timeout.connect(self._update_power_meter)
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

    def _toggle_camera(self) -> None:
        """Connect or disconnect the Raman camera."""
        if self._camera_source is not None:
            ActiveCameraManager.stop_camera(self._camera_source)
            self.video_viewer.attach_camera(None)
            self._camera_source = None
            self._camera_btn.setText("🎥 Connect Raman Camera")
            self.log_viewer.append("Raman camera disconnected")
        else:
            self._camera_btn.setEnabled(False)
            self.log_viewer.append("Connecting to Raman camera...")
            self.video_viewer._show_text_placeholder_message(f"Raman Feed ({self.site_name})\nConnecting...")
            QTimer.singleShot(2000, self._on_connect_delay)

    def _on_connect_delay(self) -> None:
        """Called after the 2-second connection delay."""
        self._camera_btn.setEnabled(True)
        try:
            cam = VideoFileCamera(path=self.RAMAN_FEED_VIDEO)
            ActiveCameraManager.start_camera(cam, self.video_viewer)
            self.video_viewer.attach_camera(cam)
            self._camera_source = cam
            self._camera_btn.setText("⏹ Disconnect Raman Camera")
            self.log_viewer.append("Connected to Raman camera")
        except Exception as e:
            self.video_viewer.set_camera_name(f"Raman Feed ({self.site_name})")
            self.log_viewer.append(f"Failed to connect Raman camera: {e}")

    _IND_ON  = "background-color: #00FF00; border-radius: 12px; border: 1px solid black;"
    _IND_OFF = "background-color: #222222; border-radius: 12px; border: 1px solid black;"
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

    def _press_button_briefly(self, btn, on_release=None) -> None:
        btn.setStyleSheet(self._BTN_PRESSED_STYLE)
        btn.setEnabled(False)

        def _restore():
            btn.setStyleSheet("")
            btn.setEnabled(True)
            if on_release:
                on_release()

        QTimer.singleShot(1000, _restore)

    def _on_laser_on(self) -> None:
        self.log_viewer.append("532nm laser on command sent")

        def _on_release():
            self.laser_indicator.setStyleSheet(self._IND_ON)
            self.log_viewer.append("532nm laser on")
            self._power_value = 0
            self._power_direction = 1
            self._power_timer.start()

        self._press_button_briefly(self.btn_laser_on, on_release=_on_release)

    def _on_laser_off(self) -> None:
        self.log_viewer.append("532nm laser off command sent")

        def _on_release():
            self.laser_indicator.setStyleSheet(self._IND_OFF)
            self.log_viewer.append("532nm laser off")
            self._power_direction = -1
            self._power_timer.start()

        self._press_button_briefly(self.btn_laser_off, on_release=_on_release)

    def _update_power_meter(self) -> None:
        self._power_value += self._power_direction
        self._power_value = max(0, min(100, self._power_value))
        self._power_lbl.setText(f"Power Meter:   {self._power_value}%")
        if self._power_value >= 100 or self._power_value <= 0:
            self._power_timer.stop()
