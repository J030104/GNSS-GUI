"""Control panel for adjusting video and communication settings.

This widget groups together various input controls that can be used to
adjust properties of the rover's video streams and monitor
communication parameters. It emits signals when values change so
other parts of the application can react accordingly.
"""

from typing import Optional

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QTabWidget,
    QSlider,
    QComboBox,
    QPushButton,
    QGroupBox,
    QFormLayout,
    QSpinBox,
)


class ControlPanel(QWidget):
    """A panel of sliders and controls for camera and communication settings."""

    # Define custom signals for parameter updates
    cameraChanged = pyqtSignal(str)
    bitrateChanged = pyqtSignal(int)
    brightnessChanged = pyqtSignal(int)
    zoomChanged = pyqtSignal(int)
    startStreamRequested = pyqtSignal()
    stopStreamRequested = pyqtSignal()

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)

        layout = QVBoxLayout()

        # Camera selection
        camera_layout = QHBoxLayout()
        camera_label = QLabel("Camera:")
        self.camera_combo = QComboBox()
        # Include a Local camera option for the device's default camera.
        # The string value here is presented to users; consumers map it to
        # underlying device names (e.g. 'local' -> local camera).
        self.camera_combo.addItems(["Local", "Camera 1", "Camera 2", "Camera 3", "Insta 360"])
        # handle saving/loading when selection changes
        self.camera_combo.currentTextChanged.connect(self._on_camera_changed)
        camera_layout.addWidget(camera_label)
        camera_layout.addWidget(self.camera_combo)
        layout.addLayout(camera_layout)

        # Tab widget groups controls for easier navigation
        self.tabs = QTabWidget()

        # Video tab
        video_tab = QWidget()
        video_layout = QVBoxLayout()
        bitrate_group = QGroupBox("Video bitrate (kbps)")
        bitrate_form = QFormLayout()
        self.bitrate_slider = QSlider(Qt.Horizontal)
        self.bitrate_slider.setRange(500, 5000)
        self.bitrate_slider.setValue(2000)
        bitrate_form.addRow(self.bitrate_slider)
        bitrate_group.setLayout(bitrate_form)
        self.bitrate_slider.valueChanged.connect(self._on_bitrate_changed)
        video_layout.addWidget(bitrate_group)
        video_layout.addStretch(1)
        video_tab.setLayout(video_layout)

        # Image tab
        image_tab = QWidget()
        image_layout = QVBoxLayout()
        brightness_group = QGroupBox("Brightness")
        brightness_form = QFormLayout()
        self.brightness_slider = QSlider(Qt.Horizontal)
        self.brightness_slider.setRange(-50, 50)
        self.brightness_slider.setValue(0)
        brightness_form.addRow(self.brightness_slider)
        brightness_group.setLayout(brightness_form)
        self.brightness_slider.valueChanged.connect(self._on_brightness_changed)
        image_layout.addWidget(brightness_group)

        zoom_group = QGroupBox("Zoom")
        zoom_form = QFormLayout()
        self.zoom_slider = QSlider(Qt.Horizontal)
        self.zoom_slider.setRange(1, 10)
        self.zoom_slider.setValue(1)
        zoom_form.addRow(self.zoom_slider)
        zoom_group.setLayout(zoom_form)
        self.zoom_slider.valueChanged.connect(self._on_zoom_changed)
        image_layout.addWidget(zoom_group)

        image_layout.addStretch(1)
        image_tab.setLayout(image_layout)

        # Stream tab
        stream_tab = QWidget()
        stream_layout = QVBoxLayout()
        stream_btns = QHBoxLayout()
        self.start_button = QPushButton("Start Stream")
        self.stop_button = QPushButton("Stop Stream")
        # use local handlers so we can track per-camera state
        self.start_button.clicked.connect(self._on_start_stream)
        self.stop_button.clicked.connect(self._on_stop_stream)
        stream_btns.addWidget(self.start_button)
        stream_btns.addWidget(self.stop_button)
        stream_layout.addLayout(stream_btns)

        status_layout = QHBoxLayout()
        self.bandwidth_label = QLabel("Bandwidth: -- kbps")
        self.conn_status_label = QLabel("Connection: Disconnected")
        status_layout.addWidget(self.bandwidth_label)
        status_layout.addWidget(self.conn_status_label)
        stream_layout.addLayout(status_layout)

        stream_layout.addStretch(1)
        stream_tab.setLayout(stream_layout)

        self.tabs.addTab(video_tab, "Video")
        self.tabs.addTab(image_tab, "Image")
        self.tabs.addTab(stream_tab, "Stream")

        layout.addWidget(self.tabs)

        layout.addStretch(1)
        self.setLayout(layout)

        # Per-camera settings storage: camera_name -> {tab: {key: value}}
        self.camera_settings = {}
        for i in range(self.camera_combo.count()):
            cam = self.camera_combo.itemText(i)
            self.camera_settings[cam] = self._default_settings()

        # Track current camera and load its settings
        self._current_camera = self.camera_combo.currentText()
        self._load_camera_settings(self._current_camera)

    def _on_bitrate_changed(self, value: int) -> None:
        """Internal slot to normalise bitrate value, store it and emit signal."""
        cam = getattr(self, "_current_camera", None)
        if cam and cam in self.camera_settings:
            self.camera_settings[cam]["Video"]["bitrate"] = int(value)
        self.bitrateChanged.emit(value)

    def _on_brightness_changed(self, value: int) -> None:
        cam = getattr(self, "_current_camera", None)
        if cam and cam in self.camera_settings:
            self.camera_settings[cam]["Image"]["brightness"] = int(value)
        self.brightnessChanged.emit(value)

    def _on_zoom_changed(self, value: int) -> None:
        cam = getattr(self, "_current_camera", None)
        if cam and cam in self.camera_settings:
            self.camera_settings[cam]["Image"]["zoom"] = int(value)
        self.zoomChanged.emit(value)

    def _on_start_stream(self) -> None:
        cam = getattr(self, "_current_camera", None)
        if cam and cam in self.camera_settings:
            self.camera_settings[cam]["Stream"]["streaming"] = True
        self.startStreamRequested.emit()

    def _on_stop_stream(self) -> None:
        cam = getattr(self, "_current_camera", None)
        if cam and cam in self.camera_settings:
            self.camera_settings[cam]["Stream"]["streaming"] = False
        self.stopStreamRequested.emit()

    # Public methods to update status labels
    def set_bandwidth(self, kbps: float) -> None:
        self.bandwidth_label.setText(f"Bandwidth: {kbps:.1f} kbps")
        cam = getattr(self, "_current_camera", None)
        if cam and cam in self.camera_settings:
            self.camera_settings[cam]["Stream"]["bandwidth"] = float(kbps)

    def set_connection_status(self, connected: bool) -> None:
        self.conn_status_label.setText(
            "Connection: Connected" if connected else "Connection: Disconnected"
        )
        cam = getattr(self, "_current_camera", None)
        if cam and cam in self.camera_settings:
            self.camera_settings[cam]["Stream"]["connected"] = bool(connected)

    # --- Helper methods for per-camera settings ---
    def _default_settings(self) -> dict:
        return {
            "Video": {"bitrate": 2000},
            "Image": {"brightness": 0, "zoom": 1},
            "Stream": {"bandwidth": None, "connected": False, "streaming": False},
        }

    def _save_current_camera_settings(self) -> None:
        """Save transient UI values into the settings dict for the current camera."""
        cam = getattr(self, "_current_camera", None)
        if not cam or cam not in self.camera_settings:
            return
        s = self.camera_settings[cam]
        s["Video"]["bitrate"] = int(self.bitrate_slider.value())
        s["Image"]["brightness"] = int(self.brightness_slider.value())
        s["Image"]["zoom"] = int(self.zoom_slider.value())

    def _load_camera_settings(self, cam: str) -> None:
        """Load settings for given camera into the UI, blocking signals while doing so."""
        if cam not in self.camera_settings:
            return
        s = self.camera_settings[cam]
        # block signals while setting values to avoid emitting events
        self.bitrate_slider.blockSignals(True)
        self.brightness_slider.blockSignals(True)
        self.zoom_slider.blockSignals(True)
        try:
            self.bitrate_slider.setValue(int(s["Video"].get("bitrate", 2000)))
            self.brightness_slider.setValue(int(s["Image"].get("brightness", 0)))
            self.zoom_slider.setValue(int(s["Image"].get("zoom", 1)))
        finally:
            self.bitrate_slider.blockSignals(False)
            self.brightness_slider.blockSignals(False)
            self.zoom_slider.blockSignals(False)

    def _on_camera_changed(self, new_camera: str) -> None:
        """Slot called when the camera combo selection changes.
        Saves the previous camera's UI values, switches internal pointer and loads the
        new camera's settings, then emits the public cameraChanged signal.
        """
        # save previous
        self._save_current_camera_settings()
        # update current
        self._current_camera = new_camera
        # load new settings
        self._load_camera_settings(new_camera)
        # emit external signal for consumers
        self.cameraChanged.emit(new_camera)

    def get_camera_settings(self, cam: Optional[str] = None) -> dict:
        """Return stored settings for a camera (default: current camera)."""
        if cam is None:
            cam = getattr(self, "_current_camera", None)
        return self.camera_settings.get(cam, {})