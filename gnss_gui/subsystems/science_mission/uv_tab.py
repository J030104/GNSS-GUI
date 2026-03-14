from typing import Optional
import os
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
    QRadioButton
)
from PyQt5.QtCore import QTimer

from ...components.video_viewer import VideoViewer
from ...components.log_viewer import LogViewer
from ...utilities.video_streamer import VideoFileCamera, ActiveCameraManager


class UVFluorescenceTab(QWidget):
    # Temporary UV microscopy video file
    UV_FEED_VIDEO = str(Path(__file__).resolve().parents[3] / "assets" / "videos" / "uv_microscopy.MOV")

    def __init__(self, site_name: str, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.site_name = site_name
        self._camera_source: Optional[VideoFileCamera] = None
        self._setup_ui()

    def _setup_ui(self):
        """Setup the layout for a UV Fluorescence tab."""
        main_layout = QVBoxLayout()
        
        # Top: Video Feed
        video_section = QVBoxLayout()
        self.video_viewer = VideoViewer(camera_name=f"UV Fluorescence Feed ({self.site_name})")
        self.video_viewer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        video_section.addWidget(self.video_viewer, stretch=1)

        # Camera toggle button
        self._camera_btn = QPushButton("🎥 Connect UV Microscope")
        self._camera_btn.setFixedHeight(28)
        self._camera_btn.clicked.connect(self._toggle_camera)
        video_section.addWidget(self._camera_btn)

        main_layout.addLayout(video_section, stretch=3)
        
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
        self._s1_on_btn = QPushButton("On")
        self._s1_on_btn.setFixedWidth(40)
        self._s1_on_btn.clicked.connect(self._on_sheath1_on)
        self._s1_off_btn = QPushButton("Off")
        self._s1_off_btn.setFixedWidth(40)
        self._s1_off_btn.clicked.connect(self._on_sheath1_off)
        s1_layout.addWidget(self._s1_on_btn)
        s1_layout.addWidget(self._s1_off_btn)
        self._s1_indicator = QLabel()
        self._s1_indicator.setFixedSize(25, 25)
        self._s1_indicator.setStyleSheet("background-color: #222222; border-radius: 12px; border: 1px solid black;")
        s1_layout.addWidget(self._s1_indicator)
        sheath1_group.setLayout(s1_layout)
        sheaths_row.addWidget(sheath1_group)

        # Sheath 2
        sheath2_group = QGroupBox("Cache Docking Sheath 2")
        s2_layout = QHBoxLayout()
        s2_layout.setContentsMargins(5, 5, 5, 5)
        self._s2_on_btn = QPushButton("On")
        self._s2_on_btn.setFixedWidth(40)
        self._s2_on_btn.clicked.connect(self._on_sheath2_on)
        self._s2_off_btn = QPushButton("Off")
        self._s2_off_btn.setFixedWidth(40)
        self._s2_off_btn.clicked.connect(self._on_sheath2_off)
        s2_layout.addWidget(self._s2_on_btn)
        s2_layout.addWidget(self._s2_off_btn)
        self._s2_indicator = QLabel()
        self._s2_indicator.setFixedSize(25, 25)
        self._s2_indicator.setStyleSheet("background-color: #222222; border-radius: 12px; border: 1px solid black;")
        s2_layout.addWidget(self._s2_indicator)
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
        
        self._rb_340 = QRadioButton("340nm")
        self._rb_450 = QRadioButton("450nm")
        self._rb_340.setChecked(True)

        filter_layout.addWidget(self._rb_340)
        filter_layout.addWidget(self._rb_450)
        filter_layout.addStretch()
        
        led_layout.addLayout(filter_layout, 0, 0, 1, 3)
        
        # LED On/Off + Switch (Row 1)
        power_layout = QHBoxLayout()
        power_layout.setContentsMargins(0, 0, 0, 0)
        
        self._led_on_btn = QPushButton("On")
        self._led_on_btn.setFixedWidth(40)
        self._led_on_btn.clicked.connect(self._on_led_on)
        self._led_off_btn = QPushButton("Off")
        self._led_off_btn.setFixedWidth(40)
        self._led_off_btn.clicked.connect(self._on_led_off)

        power_layout.addWidget(self._led_on_btn)
        power_layout.addWidget(self._led_off_btn)

        self._led_indicator = QLabel()
        self._led_indicator.setFixedSize(25, 25)
        self._led_indicator.setStyleSheet("background-color: #222222; border-radius: 12px; border: 1px solid black;")
        power_layout.addWidget(self._led_indicator)
        
        # Switch Button (moved here)
        self._btn_switch_filter = QPushButton("Switch")
        self._btn_switch_filter.setFixedWidth(90)
        self._btn_switch_filter.setStyleSheet("background-color: #4287f5; color: white; font-weight: bold;")
        self._btn_switch_filter.clicked.connect(self._on_switch_filter)
        power_layout.addWidget(self._btn_switch_filter)
        
        power_layout.addStretch()
        
        led_layout.addLayout(power_layout, 1, 0, 1, 3)
        
        led_group.setLayout(led_layout)
        led_row.addWidget(led_group)
        
        # Right Side Buttons (Begin Test + Switch to Raman)
        right_btns_layout = QVBoxLayout()
        right_btns_layout.setContentsMargins(0, 0, 0, 0)
        
        # Begin Docker Button
        btn_begin_test = QPushButton("Begin Docker")
        btn_begin_test.setStyleSheet("background-color: #90EE90; font-weight: bold; padding: 10px;")
        btn_begin_test.clicked.connect(self._on_begin_docker)
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
        self.log_viewer = LogViewer()
        log_layout.addWidget(self.log_viewer)
        log_group.setLayout(log_layout)
        bottom_row.addWidget(log_group, stretch=1) # Log takes remaining space (effectively wider)

        main_layout.addLayout(bottom_row, stretch=1)
        self.setLayout(main_layout)

    def _toggle_camera(self) -> None:
        """Connect or disconnect the UV microscope stream."""
        if self._camera_source is not None:
            ActiveCameraManager.stop_camera(self._camera_source)
            self.video_viewer.attach_camera(None)
            self._camera_source = None
            self._camera_btn.setText("🎥 Connect UV Microscope")
            self.log_viewer.append("UV microscope disconnected")
        else:
            self._camera_btn.setEnabled(False)
            self.log_viewer.append("Connecting to UV microscope...")
            self.video_viewer._show_text_placeholder_message(f"UV Fluorescence Feed ({self.site_name})\nConnecting...")
            QTimer.singleShot(2000, self._on_connect_delay)

    def _on_connect_delay(self) -> None:
        """Called after the 2-second connection delay."""
        self._camera_btn.setEnabled(True)
        try:
            cam = VideoFileCamera(path=self.UV_FEED_VIDEO)
            ActiveCameraManager.start_camera(cam, self.video_viewer)
            self.video_viewer.attach_camera(cam)
            self._camera_source = cam
            self._camera_btn.setText("⏹ Disconnect UV Microscope")
            self.log_viewer.append("Connected to UV microscope")
        except Exception as e:
            self.video_viewer.set_camera_name(f"UV Fluorescence Feed ({self.site_name})")
            self.log_viewer.append(f"Failed to connect UV microscope: {e}")

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

    def _on_sheath1_on(self) -> None:
        self.log_viewer.append("Sheath 1 on command sent")

        def _on_release():
            self._s1_indicator.setStyleSheet(self._IND_ON)
            self.log_viewer.append("Cache Docking Sheath 1 on")

        self._press_button_briefly(self._s1_on_btn, on_release=_on_release)

    def _on_sheath1_off(self) -> None:
        self.log_viewer.append("Sheath 1 off command sent")

        def _on_release():
            self._s1_indicator.setStyleSheet(self._IND_OFF)
            self.log_viewer.append("Cache Docking Sheath 1 off")

        self._press_button_briefly(self._s1_off_btn, on_release=_on_release)

    def _on_sheath2_on(self) -> None:
        self.log_viewer.append("Sheath 2 on command sent")

        def _on_release():
            self._s2_indicator.setStyleSheet(self._IND_ON)
            self.log_viewer.append("Cache Docking Sheath 2 on")

        self._press_button_briefly(self._s2_on_btn, on_release=_on_release)

    def _on_sheath2_off(self) -> None:
        self.log_viewer.append("Sheath 2 off command sent")

        def _on_release():
            self._s2_indicator.setStyleSheet(self._IND_OFF)
            self.log_viewer.append("Cache Docking Sheath 2 off")

        self._press_button_briefly(self._s2_off_btn, on_release=_on_release)

    def _on_led_on(self) -> None:
        self.log_viewer.append("LED on command sent")

        def _on_release():
            self._led_indicator.setStyleSheet(self._IND_ON)
            self.log_viewer.append("LED on")

        self._press_button_briefly(self._led_on_btn, on_release=_on_release)

    def _on_led_off(self) -> None:
        self.log_viewer.append("LED off command sent")

        def _on_release():
            self._led_indicator.setStyleSheet(self._IND_OFF)
            self.log_viewer.append("LED off")

        self._press_button_briefly(self._led_off_btn, on_release=_on_release)

    def _on_begin_docker(self) -> None:
        self._on_sheath1_on()
        self._on_sheath2_on()

    def _on_switch_filter(self) -> None:
        currently_340 = self._rb_340.isChecked()
        target = "450nm" if currently_340 else "340nm"
        self.log_viewer.append(f"Filter switch command sent → {target}")
        self._btn_switch_filter.setEnabled(False)

        def _do_switch():
            if currently_340:
                self._rb_450.setChecked(True)
            else:
                self._rb_340.setChecked(True)
            self._btn_switch_filter.setEnabled(True)
            self.log_viewer.append(f"Filter switched to {target}")

        QTimer.singleShot(1000, _do_switch)
