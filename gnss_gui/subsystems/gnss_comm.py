"""GNSS & Communication subsystem tab.

This module implements the core of the GNSS & Communication GUI.  It
combines several components: a video viewer, a map viewer, a control
panel with sliders and buttons, and a log viewer.  It also manages
connection status and simulated bandwidth updates.
"""

from __future__ import annotations

from typing import Optional

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QMessageBox, QListWidget, QPushButton, QLabel, QSplitter

from ..components import VideoViewer, MapViewer, ControlPanel, LogViewer, ShellTabs
from ..components.video_layout_tabs import VideoLayoutTabWidget
from ..utilities.connection_manager import ConnectionManager, SSHConfig
from ..utilities.video_streamer import VideoStreamer, CameraManager


class GNSSCommWidget(QWidget):
    """Widget containing all GUI elements for the GNSS & Communication subsystem."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)

        # Camera layout tab definitions (customize as needed)
        self.video_layout_tabs = VideoLayoutTabWidget([
            {
                'name': 'Mode 1',
                'boxes': [
                    {'row': 0, 'col': 0, 'camera_name': 'Dual Camera'},
                    {'row': 0, 'col': 2, 'camera_name': 'Dual Camera'},
                    {'row': 0, 'col': 1, 'rowspan': 3, 'camera_name': 'Insta360'},
                    {'row': 2, 'col': 0, 'camera_name': 'Left USB Camera'},
                    {'row': 2, 'col': 2, 'camera_name': 'Right USB Camera'},
                ],
            },
            {
                'name': 'Mode 2',
                'boxes': [
                    {'row': 0, 'col': 0, 'camera_name': 'Dual Camera'},
                    {'row': 0, 'col': 1, 'camera_name': 'Dual Camera'},
                    {'row': 1, 'col': 0, 'camera_name': 'Left USB Camera'},
                    {'row': 1, 'col': 1, 'camera_name': 'Right USB Camera'},
                ],
            },
            {
                'name': 'Mode 3',
                'boxes': [
                    {'row': 0, 'col': 0, 'camera_name': 'Insta360'},
                ],
            },
        ])
        self.map_placeholder = QWidget()
        self.map_viewer = MapViewer(parent=self)
        self.map_viewer.hide()  # Initially hidden
        self.control_panel = ControlPanel()
        self.log_viewer = LogViewer()
        self.shell_tabs = ShellTabs(log_viewer=self.log_viewer)
        self.connection = ConnectionManager(SSHConfig(host="jetson.local"))
        self.video_streamer = None

        # Create map toggle button
        self.map_toggle_button = QPushButton("📍")
        self.map_toggle_button.setFixedSize(30, 30)
        self.map_toggle_button.setToolTip("Toggle Map")
        self.map_toggle_button.clicked.connect(self._toggle_map)

        # Layout configuration
        # Create top widget with map button
        top_widget = QWidget()
        top_widget_layout = QVBoxLayout()
        top_widget_layout.setContentsMargins(0, 0, 0, 0)
        
        # Button container at the top
        button_container = QWidget()
        button_layout = QHBoxLayout()
        button_layout.setContentsMargins(0, 0, 0, 0)
        button_layout.addWidget(self.map_toggle_button)
        button_layout.addStretch()
        button_container.setLayout(button_layout)
        
        top_widget_layout.addWidget(button_container)
        top_widget_layout.addWidget(self.video_layout_tabs)
        top_widget.setLayout(top_widget_layout)

        # Create bottom widget for control panel and shell tabs
        bottom_widget = QWidget()
        bottom_layout = QHBoxLayout()
        bottom_layout.setContentsMargins(1, 1, 1, 1)  # Reduce margins for tighter fit
        bottom_layout.addWidget(self.control_panel, stretch=1)
        bottom_layout.addWidget(self.shell_tabs, stretch=2)
        bottom_widget.setLayout(bottom_layout)
        
        # Set minimum sizes for individual components
        self.control_panel.setMinimumHeight(60)
        self.shell_tabs.setMinimumHeight(60)

        # Create vertical splitter for resizable sections
        splitter = QSplitter(Qt.Vertical)
        splitter.addWidget(top_widget)
        splitter.addWidget(bottom_widget)
        splitter.setStretchFactor(0, 3)  # Top section gets more space
        splitter.setStretchFactor(1, 1)  # Bottom section gets less space
        
        # Set minimum sizes to control collapse behavior
        top_widget.setMinimumHeight(300)  # Video section minimum
        bottom_widget.setMinimumHeight(180)  # Lower minimum for control panel section
        
        # Configure splitter behavior
        splitter.setCollapsible(0, False)  # Prevent video section from collapsing completely
        splitter.setCollapsible(1, True)   # Allow control panel section to collapse

        main_layout = QVBoxLayout()
        main_layout.addWidget(splitter)
        self.setLayout(main_layout)

        QTimer.singleShot(0, self._place_map_over_placeholder)

        # Connect control panel signals to actions
        self.control_panel.cameraChanged.connect(self.on_camera_changed)
        self.control_panel.bitrateChanged.connect(self.on_bitrate_changed)
        self.control_panel.brightnessChanged.connect(self.on_brightness_changed)
        self.control_panel.zoomChanged.connect(self.on_zoom_changed)
        self.control_panel.startStreamRequested.connect(self.on_start_stream)
        self.control_panel.stopStreamRequested.connect(self.on_stop_stream)

        # For backward compatibility, keep a reference to the first video viewer for parameter adjustment
        self.video_viewer = self.video_layout_tabs.get_video_viewers(0)[0]

        # Start initial connection
        if self.connection.connect():
            self.control_panel.set_connection_status(True)
            self.log_viewer.append("Connected to rover")
        else:
            self.control_panel.set_connection_status(False)
            self.log_viewer.append("Failed to connect to rover")

        # Timer to periodically update bandwidth
        self.bandwidth_timer = QTimer(self)
        self.bandwidth_timer.timeout.connect(self.update_bandwidth)
        self.bandwidth_timer.start(1000)

    def _toggle_map(self) -> None:
        """Toggle the map visibility."""
        if self.map_viewer.isVisible():
            self.map_viewer.hide()
            self.map_toggle_button.setText("📍")
            self.map_toggle_button.setToolTip("Show Map")
        else:
            self.map_viewer.show()
            self.map_viewer.raise_()
            self._place_map_over_placeholder()
            self.map_toggle_button.setText("✖")
            self.map_toggle_button.setToolTip("Hide Map")

    def _on_view_item_clicked(self, item) -> None:
        """Handle direct clicks on a view list item (Shell or Log)."""
        try:
            if item is None:
                return
            text = item.text()
            if text == "Shell":
                try:
                    self.shell_tabs.add_shell()
                except Exception:
                    pass
            elif text == "Log":
                try:
                    # If log is in tabs, bring it forward; otherwise focus it
                    # shell_tabs.log_viewer() is expected to return the log widget
                    log_widget = self.shell_tabs.log_viewer()
                    if getattr(self.shell_tabs, '_log_in_tab', True):
                        idx = self.shell_tabs._tabs.indexOf(log_widget)
                        if idx != -1:
                            self.shell_tabs._tabs.setCurrentIndex(idx)
                    else:
                        try:
                            log_widget.setFocus()
                        except Exception:
                            pass
                except Exception:
                    pass
        except Exception:
            pass

    def _place_map_over_placeholder(self) -> None:
        """Place the floating MapViewer over the video area."""
        try:
            # Position the map in the top-right corner of the video layout tabs
            video_geom = self.video_layout_tabs.geometry()
            map_width = 300
            map_height = 200
            x = video_geom.right() - map_width - 10
            y = video_geom.top() + 40  # Account for the button area
            self.map_viewer.setGeometry(x, y, map_width, map_height)
        except Exception:
            # Be defensive in case widget hasn't been laid out for some reason
            pass

    # Slot implementations
    def on_camera_changed(self, camera_name: str) -> None:
        # Only update the first video viewer for parameter adjustment
        try:
            viewer = self.video_layout_tabs.get_video_viewers(0)[0]
        except Exception:
            viewer = self.video_viewer
        try:
            if camera_name.lower().startswith("local"):
                cam = CameraManager.get_camera("local")
                if cam is not None:
                    viewer.attach_camera(cam)
                    self._attached_camera = cam
                    self.video_streamer = None
                    return
        except Exception:
            pass
        device_map = {
            "Camera 1": "/dev/video0",
            "Camera 2": "/dev/video1",
            "Camera 3": "/dev/video2",
            "Insta 360": "/dev/video3",
        }
        device = device_map.get(camera_name, "/dev/video0")
        self.video_streamer = VideoStreamer(camera_device=device)
        try:
            viewer.attach_camera(None)
        except Exception:
            pass

    def on_bitrate_changed(self, value: int) -> None:
        self.log_viewer.append(f"Bitrate set to {value} kbps")
        # If a stream is active, adjust bitrate (not implemented)
        # In a real implementation you might send a command to ffmpeg
        if self.video_streamer is not None:
            pass

    def on_brightness_changed(self, value: int) -> None:
        # Adjust brightness in the video stream (stub)
        pass

    def on_zoom_changed(self, value: int) -> None:
        # Adjust zoom in the video stream (stub)
        pass

    def on_start_stream(self) -> None:
        cam_name = getattr(self.control_panel, '_current_camera', None)
        try:
            viewer = self.video_layout_tabs.get_video_viewers(0)[0]
        except Exception:
            viewer = self.video_viewer
        try:
            if isinstance(cam_name, str) and cam_name.lower().startswith('local'):
                cam = CameraManager.get_camera('local')
                if cam is not None:
                    try:
                        viewer.attach_camera(cam)
                        self._attached_camera = cam
                        cam.start()
                        return
                    except Exception:
                        pass
        except Exception:
            pass
        if self.video_streamer is None:
            self.video_streamer = VideoStreamer(camera_device="/dev/video0")
        self.video_streamer.start_stream(self.control_panel.bitrate_slider.value())

    def on_stop_stream(self) -> None:
        try:
            viewer = self.video_layout_tabs.get_video_viewers(0)[0]
        except Exception:
            viewer = self.video_viewer
        try:
            if getattr(self, '_attached_camera', None) is not None:
                try:
                    self._attached_camera.stop()
                except Exception:
                    pass
                try:
                    viewer.attach_camera(None)
                except Exception:
                    pass
                self._attached_camera = None
                return
        except Exception:
            pass
        if self.video_streamer is not None:
            self.video_streamer.stop_stream()

    def update_bandwidth(self) -> None:
        """Update bandwidth label and maybe rover position."""
        if self.connection.is_connected():
            bw = self.connection.get_bandwidth()
            self.control_panel.set_bandwidth(bw)
        else:
            self.control_panel.set_bandwidth(0.0)
        # Simulate rover motion by updating map viewer position/heading
        import random

        lat = random.random()
        lon = random.random()
        heading = random.uniform(0, 360)
        self.map_viewer.set_position(lat, lon)
        self.map_viewer.set_heading(heading)