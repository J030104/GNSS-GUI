"""GNSS & Communication subsystem tab.

This module implements the core of the GNSS & Communication GUI.  It
combines several components: a video viewer, a map viewer, a control
panel with sliders and buttons, and a log viewer.  It also manages
connection status and simulated bandwidth updates.
"""

from __future__ import annotations

from typing import Optional

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QMessageBox, QListWidget, QPushButton, QLabel

from ..components import VideoViewer, MapViewer, ControlPanel, LogViewer, ShellTabs
from ..utilities.connection_manager import ConnectionManager, SSHConfig
from ..utilities.video_streamer import VideoStreamer, CameraManager


class GNSSCommWidget(QWidget):
    """Widget containing all GUI elements for the GNSS & Communication subsystem."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)

        # Components
        self.video_viewer = VideoViewer(fps=10)
        # Create a placeholder in the layout; the real MapViewer will be
        # a floating child so the user can move/resize it without the
        # layout snapping it back.
        self.map_placeholder = QWidget()
        self.map_viewer = MapViewer(parent=self)
        self.control_panel = ControlPanel()
        self.log_viewer = LogViewer()
        # ShellTabs contains the log as the first non-closable tab
        self.shell_tabs = ShellTabs(log_viewer=self.log_viewer)

        # Connection and streaming utilities
        self.connection = ConnectionManager(SSHConfig(host="jetson.local"))
        self.video_streamer = None
        # Do not attach or start the local camera until the user requests
        # streaming. The CameraManager has the camera instance registered
        # but it will remain idle until started.

        # Layout configuration
        top_layout = QHBoxLayout()
        # Collapsible view widget (top-left, collapsed by default)
        class CollapsibleView(QWidget):
            def __init__(self, parent: Optional[QWidget] = None) -> None:
                super().__init__(parent)
                self._collapsed_w = 28
                self._expanded_w = 220
                # Limit vertical size so the view list doesn't expand the whole window
                self._collapsed_h = 28
                self._expanded_h = 160
                # Start collapsed with a small button-like appearance
                self.setFixedWidth(self._collapsed_w)
                self.setMaximumHeight(self._collapsed_h)
                self.setAutoFillBackground(True)
                self._layout = QVBoxLayout()
                self._layout.setContentsMargins(2, 2, 2, 2)
                self._layout.setAlignment(Qt.AlignTop)
                # Toggle button (visible when collapsed) — behaves like a small header
                self._toggle = QPushButton("☰")
                self._toggle.setFixedSize(24, 24)
                self._toggle.setToolTip("Views")
                self._toggle.setFocusPolicy(Qt.NoFocus)
                # List of views shown when expanded
                self._list = QListWidget()
                # Prevent the list from growing too tall
                self._list.setMaximumHeight(self._expanded_h - 32)
                self._layout.addWidget(self._toggle, alignment=Qt.AlignCenter)
                self._layout.addWidget(self._list)
                self.setLayout(self._layout)
                # Start collapsed: hide list
                self._list.hide()

            def enterEvent(self, event):  # type: ignore[override]
                try:
                    self.setFixedWidth(self._expanded_w)
                    self.setMaximumHeight(self._expanded_h)
                    self._list.show()
                except Exception:
                    pass

            def leaveEvent(self, event):  # type: ignore[override]
                try:
                    self._list.hide()
                    self.setFixedWidth(self._collapsed_w)
                    self.setMaximumHeight(self._collapsed_h)
                except Exception:
                    pass

        # self.view_widget = CollapsibleView(parent=self)
        # Add default items
        # self.view_widget._list.addItem("Shell")
        # self.view_widget._list.addItem("Log")

        # top_layout.addWidget(self.view_widget, stretch=0)
        top_layout.addWidget(self.video_viewer, stretch=3)
        top_layout.addWidget(self.map_placeholder, stretch=2)

        # Connect list clicks directly to the handler (no separate View button)
        # self.view_widget._list.itemClicked.connect(self._on_view_item_clicked)

        bottom_layout = QHBoxLayout()
        bottom_layout.addWidget(self.control_panel, stretch=1)
        bottom_layout.addWidget(self.shell_tabs, stretch=2)

        main_layout = QVBoxLayout()
        main_layout.addLayout(top_layout, stretch=3)
        main_layout.addLayout(bottom_layout, stretch=2)
        self.setLayout(main_layout)

        # Position the floating map over the placeholder area so the initial
        # layout looks unchanged. Do this after the event loop runs so the
        # layout has finalised widget geometries.
        QTimer.singleShot(0, self._place_map_over_placeholder)

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
        """Place the floating MapViewer over the placeholder widget."""
        try:
            geom = self.map_placeholder.geometry()
            self.map_viewer.setGeometry(geom)
            self.map_viewer.show()
            self.map_viewer.raise_()
        except Exception:
            # Be defensive in case widget hasn't been laid out for some reason
            pass

    # Slot implementations
    def on_camera_changed(self, camera_name: str) -> None:
        self.log_viewer.append(f"Camera changed to {camera_name}")
        # If the user chose the Local camera use the CameraManager to
        # attach the platform's default camera directly to the viewer.
        try:
            if camera_name.lower().startswith("local"):
                cam = CameraManager.get_camera("local")
                if cam is not None:
                    # Attach camera to the viewer; VideoViewer will start it
                    self.video_viewer.attach_camera(cam)
                    # Keep a reference to allow start/stop via the UI
                    self._attached_camera = cam
                    # Clear any VideoStreamer (we're showing local feed)
                    self.video_streamer = None
                    return
        except Exception:
            # fall back to previous behaviour if camera manager fails
            pass

        # Choose camera device path based on selection for remote/ffmpeg use
        device_map = {
            "Camera 1": "/dev/video0",
            "Camera 2": "/dev/video1",
            "Camera 3": "/dev/video2",
            "Insta 360": "/dev/video3",
        }
        device = device_map.get(camera_name, "/dev/video0")
        # Recreate video streamer with new device
        self.video_streamer = VideoStreamer(camera_device=device)
        # Detach any attached camera view
        try:
            self.video_viewer.attach_camera(None)
        except Exception:
            pass

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
        # If the selected camera is Local, attach and start it now.
        cam_name = getattr(self.control_panel, '_current_camera', None)
        try:
            if isinstance(cam_name, str) and cam_name.lower().startswith('local'):
                cam = CameraManager.get_camera('local')
                if cam is not None:
                    # Attach the camera to the viewer (this does not stop
                    # any existing camera; we will replace the renderer).
                    try:
                        self.video_viewer.attach_camera(cam)
                        self._attached_camera = cam
                        cam.start()
                        return
                    except Exception:
                        # fall back to streamer below
                        pass
        except Exception:
            pass

        # Non-local path: start the VideoStreamer stub (ffmpeg path)
        if self.video_streamer is None:
            self.video_streamer = VideoStreamer(camera_device="/dev/video0")
        self.video_streamer.start_stream(self.control_panel.bitrate_slider.value())

    def on_stop_stream(self) -> None:
        self.log_viewer.append("Stopping video stream")
        # If a local camera is attached, stop it and detach the viewer
        try:
            if getattr(self, '_attached_camera', None) is not None:
                try:
                    self._attached_camera.stop()
                except Exception:
                    pass
                # Detach from the viewer so placeholder returns
                try:
                    self.video_viewer.attach_camera(None)
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