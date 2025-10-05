"""GNSS & Communication subsystem tab.

This module implements the core of the GNSS & Communication GUI.  It
combines several components: a video viewer, a map viewer, a control
panel with sliders and buttons, and a log viewer.  It also manages
connection status and simulated bandwidth updates.
"""

from __future__ import annotations

from typing import Optional

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QMessageBox

from ..components import VideoViewer, MapViewer, ControlPanel, LogViewer
from ..utilities.connection_manager import ConnectionManager, SSHConfig
from ..utilities.video_streamer import VideoStreamer


class GNSSCommWidget(QWidget):
    """Widget containing all GUI elements for the GNSS & Communication subsystem."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)

        # Components
        self.video_viewer = VideoViewer(fps=10)
        self.map_viewer = MapViewer()
        self.control_panel = ControlPanel()
        self.log_viewer = LogViewer()

        # Connection and streaming utilities
        self.connection = ConnectionManager(SSHConfig(host="jetson.local"))
        self.video_streamer: Optional[VideoStreamer] = None

        # Layout configuration
        top_layout = QHBoxLayout()
        top_layout.addWidget(self.video_viewer, stretch=3)
        top_layout.addWidget(self.map_viewer, stretch=2)

        bottom_layout = QHBoxLayout()
        bottom_layout.addWidget(self.control_panel, stretch=1)
        bottom_layout.addWidget(self.log_viewer, stretch=2)

        main_layout = QVBoxLayout()
        main_layout.addLayout(top_layout, stretch=3)
        main_layout.addLayout(bottom_layout, stretch=2)
        self.setLayout(main_layout)

        # Connect control panel signals to actions
        self.control_panel.cameraChanged.connect(self.on_camera_changed)
        self.control_panel.bitrateChanged.connect(self.on_bitrate_changed)
        self.control_panel.brightnessChanged.connect(self.on_brightness_changed)
        self.control_panel.zoomChanged.connect(self.on_zoom_changed)
        self.control_panel.startStreamRequested.connect(self.on_start_stream)
        self.control_panel.stopStreamRequested.connect(self.on_stop_stream)

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

    # Slot implementations
    def on_camera_changed(self, camera_name: str) -> None:
        self.log_viewer.append(f"Camera changed to {camera_name}")
        # Choose camera device path based on selection
        device_map = {
            "Camera 1": "/dev/video0",
            "Camera 2": "/dev/video1",
            "Camera 3": "/dev/video2",
            "Insta 360": "/dev/video3",
        }
        device = device_map.get(camera_name, "/dev/video0")
        # Recreate video streamer with new device
        self.video_streamer = VideoStreamer(camera_device=device)

    def on_bitrate_changed(self, value: int) -> None:
        self.log_viewer.append(f"Bitrate set to {value} kbps")
        # If a stream is active, adjust bitrate (not implemented)
        # In a real implementation you might send a command to ffmpeg
        if self.video_streamer is not None:
            pass

    def on_brightness_changed(self, value: int) -> None:
        self.log_viewer.append(f"Brightness offset set to {value}")
        # Adjust brightness in the video stream (stub)

    def on_zoom_changed(self, value: int) -> None:
        self.log_viewer.append(f"Zoom level set to {value}")
        # Adjust zoom in the video stream (stub)

    def on_start_stream(self) -> None:
        self.log_viewer.append("Starting video stream")
        if self.video_streamer is None:
            # Default to first camera
            self.video_streamer = VideoStreamer(camera_device="/dev/video0")
        self.video_streamer.start_stream(self.control_panel.bitrate_slider.value())

    def on_stop_stream(self) -> None:
        self.log_viewer.append("Stopping video stream")
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