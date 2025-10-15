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
from ..utilities.video_streamer import VideoStreamer, CameraManager, CameraSource


class GNSSCommWidget(QWidget):
    """Widget containing all GUI elements for the GNSS & Communication subsystem."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)

        # Camera layout tab definitions (customize as needed)
        self.video_layout_tabs = VideoLayoutTabWidget([
            {
                'name': 'Mode 1',
                'boxes': [
                    {'row': 0, 'col': 0, 'camera_name': 'Dual Camera 1'},
                    {'row': 0, 'col': 2, 'camera_name': 'Dual Camera 2'},
                    {'row': 0, 'col': 1, 'rowspan': 3, 'camera_name': 'Insta360'},
                    {'row': 2, 'col': 0, 'camera_name': 'Left USB Camera'},
                    {'row': 2, 'col': 2, 'camera_name': 'Right USB Camera'},
                ],
            },
            {
                'name': 'Mode 2',
                'boxes': [
                    {'row': 0, 'col': 0, 'camera_name': 'Dual Camera 1'},
                    {'row': 0, 'col': 1, 'camera_name': 'Dual Camera 2'},
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
        # Track which viewers we've attached a local CameraSource to
        self._attached_viewers = []  # type: list
        # Track which camera the control panel is currently adjusting
        self._parameter_camera = self.control_panel._current_camera
        # Per-camera runtime state: key -> {streaming:bool, source:object}
        # 'key' corresponds to control panel camera names (e.g. 'Local', 'Camera 1')
        self._camera_states = {}

        # Map a testing-local camera to a viewer name for now
        # (per your request, the local camera should play in 'Dual Camera 1')
        self._testing_local_target = 'Dual Camera 1'

        # Keep layouts in sync when the tab changes
        try:
            self.video_layout_tabs.tab_widget.currentChanged.connect(self._on_layout_tab_changed)
        except Exception:
            pass

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
        # Camera selection in the control panel determines which camera's
        # parameters (brightness/zoom) are being adjusted. Do NOT change
        # which viewers have active streams here — Start/Stop controls that.
        # The control panel already stores per-camera settings; just record
        # which camera is being adjusted so subsequent brightness/zoom
        # updates are targeted at that camera (if it is streaming).
        self._parameter_camera = camera_name

    def _ensure_camera_state(self, key: str) -> dict:
        if key not in self._camera_states:
            self._camera_states[key] = {
                'streaming': False,
                'source': None,
            }
        return self._camera_states[key]

    def _on_layout_tab_changed(self, index: int) -> None:
        """Called when the layout tab changes; attach/detach viewers so
        ongoing streams appear in the new layout where applicable."""
        try:
            viewers = self.video_layout_tabs.get_video_viewers(index)
        except Exception:
            viewers = self.video_layout_tabs.get_video_viewers(0)

        for v in viewers:
            vname = getattr(v, 'camera_name', '')
            # find a streaming camera whose key maps to this viewer
            matched_key = None
            for key, state in self._camera_states.items():
                if not state.get('streaming'):
                    continue
                # local testing target: if key is local, map to testing target
                if key.lower().startswith('local') and vname == self._testing_local_target:
                    matched_key = key
                    break
                # fuzzy match between viewer name and camera key
                kn = key.lower().replace(' ', '')
                vn = vname.lower().replace(' ', '')
                if kn in vn or vn in kn:
                    matched_key = key
                    break

            if matched_key is not None:
                state = self._camera_states[matched_key]
                source = state.get('source')
                try:
                    if isinstance(source, CameraSource):
                        v.attach_camera(source)
                    else:
                        # no CameraSource available (e.g. VideoStreamer stub)
                        v.attach_camera(None)
                except Exception:
                    pass
                # Apply stored settings for that camera key
                try:
                    s = self.control_panel.get_camera_settings(matched_key)
                    v.set_brightness(int(s.get('Image', {}).get('brightness', 0)))
                    v.set_zoom(int(s.get('Image', {}).get('zoom', 1)))
                except Exception:
                    pass
            else:
                # detach if nothing streaming for this viewer
                try:
                    v.attach_camera(None)
                except Exception:
                    pass

        # Update camera state attached_viewers lists so subsequent
        # adjustments affect all viewers that are currently displaying
        # a given stream. We ensure any viewers we attached above are
        # recorded, and remove viewers that are no longer attached.
        try:
            all_viewers = self.video_layout_tabs.get_all_video_viewers()
            # build reverse map viewer -> matched_key
            viewer_map = {}
            for key, state in self._camera_states.items():
                if not state.get('streaming'):
                    continue
                attached = state.get('attached_viewers') or []
                # refresh attached list by checking which of the viewers
                # currently reference this state's source
                new_attached = []
                for v in all_viewers:
                    try:
                        # if the viewer currently has a camera source that
                        # equals this state's source, consider it attached
                        if getattr(v, '_camera_source', None) is state.get('source'):
                            new_attached.append(v)
                    except Exception:
                        pass
                state['attached_viewers'] = new_attached
                for v in new_attached:
                    viewer_map[v] = key

            # For each camera state, copy pan/zoom/brightness from the
            # first attached viewer to any newly attached viewers so
            # they match the live appearance.
            for key, state in self._camera_states.items():
                attached = state.get('attached_viewers') or []
                if not attached:
                    continue
                # use first attached viewer as source of truth
                src = attached[0]
                try:
                    pan_x, pan_y = src.get_pan()
                except Exception:
                    pan_x, pan_y = (0.5, 0.5)
                try:
                    # copy settings to all attached viewers
                    for v in attached[1:]:
                        try:
                            v.set_zoom(int(src._zoom))
                            v.set_brightness(int(src._brightness))
                            v.set_pan(pan_x, pan_y)
                        except Exception:
                            pass
                except Exception:
                    pass
        except Exception:
            pass

    def on_bitrate_changed(self, value: int) -> None:
        self.log_viewer.append(f"Bitrate set to {value} kbps")
        # If a stream is active, adjust bitrate (not implemented)
        # In a real implementation you might send a command to ffmpeg
        if self.video_streamer is not None:
            pass

    def on_brightness_changed(self, value: int) -> None:
        # Update per-camera stored settings (ControlPanel already does this)
        sel = getattr(self, '_parameter_camera', None) or self.control_panel._current_camera
        state = self._ensure_camera_state(sel)
        # Only apply to viewers if this camera is currently streaming
        if state.get('streaming'):
            attached = state.get('attached_viewers') or []
            for v in attached:
                try:
                    v.set_brightness(int(value))
                except Exception:
                    pass

    def on_zoom_changed(self, value: int) -> None:
        sel = getattr(self, '_parameter_camera', None) or self.control_panel._current_camera
        state = self._ensure_camera_state(sel)
        if state.get('streaming'):
            attached = state.get('attached_viewers') or []
            for v in attached:
                try:
                    v.set_zoom(int(value))
                except Exception:
                    pass

    def on_start_stream(self) -> None:
        cam_name = getattr(self.control_panel, '_current_camera', None)
        viewers = self.video_layout_tabs.get_all_video_viewers()
        try:
            if isinstance(cam_name, str) and cam_name.lower().startswith('local'):
                cam = CameraManager.get_camera('local')
                if cam is not None:
                    try:
                        # Attach local camera only to the first 'dual' viewer
                        # (testing-only behavior requested). If no 'dual'
                        # label is found attach to the primary viewer.
                        target = None
                        for v in viewers:
                            try:
                                if 'dual' in getattr(v, 'camera_name', '').lower():
                                    target = v
                                    break
                            except Exception:
                                pass
                        if target is None and viewers:
                            target = viewers[0]
                        if target is not None:
                            try:
                                target.attach_camera(cam)
                                attached = [target]
                                # record runtime state for this control-panel camera key
                                state = self._ensure_camera_state(cam_name)
                                state['streaming'] = True
                                state['source'] = cam
                                state['attached_viewers'] = attached
                                self._attached_viewers = attached
                            except Exception:
                                self._attached_viewers = []
                        else:
                            self._attached_viewers = []
                        self._attached_camera = cam
                        cam.start()
                        return
                    except Exception:
                        pass
        except Exception:
            pass
        if self.video_streamer is None:
            self.video_streamer = VideoStreamer(camera_device="/dev/video0")
        # For remote streams we mark the camera as streaming but we don't
        # currently have a CameraSource to attach to viewers.
        state = self._ensure_camera_state(cam_name)
        state['streaming'] = True
        state['source'] = self.video_streamer
        state['attached_viewers'] = []
        self.video_streamer.start_stream(self.control_panel.bitrate_slider.value())

    def on_stop_stream(self) -> None:
        cam_name = getattr(self.control_panel, '_current_camera', None)
        state = self._ensure_camera_state(cam_name)
        try:
            if state.get('streaming'):
                source = state.get('source')
                attached = state.get('attached_viewers') or []
                # Stop camera source if possible
                try:
                    if isinstance(source, CameraSource):
                        source.stop()
                except Exception:
                    pass
                # Detach viewers that were attached for this camera
                for v in attached:
                    try:
                        v.attach_camera(None)
                    except Exception:
                        pass
                state['streaming'] = False
                state['attached_viewers'] = []
                # Clear top-level attached pointers
                if getattr(self, '_attached_camera', None) is source:
                    self._attached_camera = None
                    self._attached_viewers = []
                # If this was a VideoStreamer remote stream, stop it
                try:
                    if isinstance(source, VideoStreamer):
                        source.stop_stream()
                except Exception:
                    pass
                return
        except Exception:
            pass

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