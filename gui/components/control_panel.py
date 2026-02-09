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
    QScrollArea,
    QLineEdit,
)

from ..config import CAMERA_NAMES

class ControlPanel(QWidget):
    """A panel of sliders and controls for camera and communication settings."""

    # Define custom signals for parameter updates
    cameraChanged = pyqtSignal(str)
    framerateChanged = pyqtSignal(int)
    resolutionChanged = pyqtSignal(str)
    brightnessChanged = pyqtSignal(int)
    zoomChanged = pyqtSignal(int)
    startStreamRequested = pyqtSignal()
    stopStreamRequested = pyqtSignal()

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)

        layout = QVBoxLayout()

        # Camera selection
        self._camera_layout = QHBoxLayout()
        self._camera_label = QLabel("Camera:")
        self.camera_combo = QComboBox()
        # Include a Local camera option for the device's default camera.
        # The string value here is presented to users; consumers map it to
        # underlying device names (e.g. 'local' -> local camera).
        self.camera_combo.addItems(list(CAMERA_NAMES.values()))
        # handle saving/loading when selection changes
        self.camera_combo.currentTextChanged.connect(self._on_camera_changed)
        self._camera_layout.addWidget(self._camera_label)
        self._camera_layout.addWidget(self.camera_combo)
        layout.addLayout(self._camera_layout)

        # Tab widget groups controls for easier navigation
        self.tabs = QTabWidget()

        # Basic tab - combines video, image, and stream controls
        basic_tab = QWidget()
        basic_layout = QVBoxLayout()
        
        # Create a scroll area for the content
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        
        # Create a widget to hold all the scrollable content
        scroll_content = QWidget()
        scroll_content_layout = QVBoxLayout()
        
        # Stream controls at the top
        stream_btns = QHBoxLayout()
        self.start_button = QPushButton("Start Stream")
        self.stop_button = QPushButton("Stop Stream")
        # use local handlers so we can track per-camera state
        self.start_button.clicked.connect(self._on_start_stream)
        self.stop_button.clicked.connect(self._on_stop_stream)
        stream_btns.addWidget(self.start_button)
        stream_btns.addWidget(self.stop_button)
        scroll_content_layout.addLayout(stream_btns)

        status_layout = QHBoxLayout()
        self.bandwidth_label = QLabel("Bandwidth: -- kbps")
        status_layout.addWidget(self.bandwidth_label)
        scroll_content_layout.addLayout(status_layout)

        # Video controls
        video_settings_layout = QHBoxLayout()

        framerate_group = QGroupBox("Framerate")
        framerate_form = QFormLayout()
        self.framerate_input = QSpinBox()
        self.framerate_input.setRange(1, 120)
        self.framerate_input.setValue(30)
        framerate_form.addRow(self.framerate_input)
        framerate_group.setLayout(framerate_form)
        self.framerate_input.editingFinished.connect(self._on_framerate_editing_finished)
        self.framerate_input.setKeyboardTracking(False)
        video_settings_layout.addWidget(framerate_group)

        resolution_group = QGroupBox("Resolution")
        resolution_form = QFormLayout()
        self.resolution_input = QLineEdit()
        self.resolution_input.setText("640x480")
        resolution_form.addRow(self.resolution_input)
        resolution_group.setLayout(resolution_form)
        self.resolution_input.editingFinished.connect(self._on_resolution_editing_finished)
        video_settings_layout.addWidget(resolution_group)

        scroll_content_layout.addLayout(video_settings_layout)

        # Image controls
        image_controls_layout = QHBoxLayout()

        brightness_group = QGroupBox("Brightness")
        brightness_form = QFormLayout()
        self.brightness_slider = QSlider(Qt.Horizontal)
        self.brightness_slider.setRange(-50, 50)
        self.brightness_slider.setValue(0)
        brightness_form.addRow(self.brightness_slider)
        brightness_group.setLayout(brightness_form)
        self.brightness_slider.valueChanged.connect(self._on_brightness_changed)
        image_controls_layout.addWidget(brightness_group)

        zoom_group = QGroupBox("Zoom")
        zoom_form = QFormLayout()
        self.zoom_slider = QSlider(Qt.Horizontal)
        self.zoom_slider.setRange(1, 10)
        self.zoom_slider.setValue(1)
        zoom_form.addRow(self.zoom_slider)
        zoom_group.setLayout(zoom_form)
        self.zoom_slider.valueChanged.connect(self._on_zoom_changed)
        image_controls_layout.addWidget(zoom_group)

        scroll_content_layout.addLayout(image_controls_layout)

        scroll_content_layout.addStretch(1)
        scroll_content.setLayout(scroll_content_layout)
        
        # Set the scroll content widget to the scroll area
        scroll_area.setWidget(scroll_content)
        
        # Add the scroll area to the basic tab layout
        basic_layout.addWidget(scroll_area)
        basic_tab.setLayout(basic_layout)

        # Advanced tab - empty for now
        advanced_tab = QWidget()
        advanced_layout = QVBoxLayout()
        
        # Create a scroll area for the advanced tab content
        advanced_scroll_area = QScrollArea()
        advanced_scroll_area.setWidgetResizable(True)
        advanced_scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        advanced_scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        
        # Create a widget to hold the advanced tab content
        advanced_scroll_content = QWidget()
        advanced_scroll_content_layout = QVBoxLayout()
        advanced_scroll_content_layout.addWidget(QLabel("Advanced controls coming soon..."))
        advanced_scroll_content_layout.addStretch(1)
        advanced_scroll_content.setLayout(advanced_scroll_content_layout)
        
        # Set the scroll content widget to the scroll area
        advanced_scroll_area.setWidget(advanced_scroll_content)
        
        # Add the scroll area to the advanced tab layout
        advanced_layout.addWidget(advanced_scroll_area)
        advanced_tab.setLayout(advanced_layout)

        self.tabs.addTab(basic_tab, "Basic")
        self.tabs.addTab(advanced_tab, "Advanced")

        layout.addWidget(self.tabs)

        layout.addStretch(1)
        self.setLayout(layout)

        # Per-camera settings storage: camera_name -> {tab: {key: value}}
        self.camera_settings = {}
        for i in range(self.camera_combo.count()):
            cam = self.camera_combo.itemText(i)
            self.camera_settings[cam] = self._default_settings()

        # Track current camera and load its settings
        # This design has made future changes harder
        # (Can only adjust one camera at a time)
        self._current_camera = self.camera_combo.currentText()
        self._load_camera_settings(self._current_camera)

    def _on_framerate_editing_finished(self) -> None:
        """Handle framerate input finish (e.g. Enter pressed)."""
        value = self.framerate_input.value()
        cam = getattr(self, "_current_camera", None)

        # Check if value has changed
        if cam and cam in self.camera_settings:
            current_val = self.camera_settings[cam]["Video"].get("framerate")
            if current_val == value:
                return

        print(f"Camera '{cam}' switching framerate to {value}")
        self._on_framerate_changed(value)

    def _on_framerate_changed(self, value: int) -> None:
        cam = getattr(self, "_current_camera", None)
        if cam and cam in self.camera_settings:
            self.camera_settings[cam]["Video"]["framerate"] = int(value)
        self.framerateChanged.emit(value)

    def _on_resolution_editing_finished(self) -> None:
        """Handle resolution input finish (e.g. Enter pressed)."""
        value = self.resolution_input.text()
        cam = getattr(self, "_current_camera", None)

        # Check if value has changed
        if cam and cam in self.camera_settings:
            current_val = self.camera_settings[cam]["Video"].get("resolution")
            if current_val == value:
                return

        print(f"Camera '{cam}' switching resolution to {value}")
        self._on_resolution_changed(value)

    def _on_resolution_changed(self, value: str) -> None:
        cam = getattr(self, "_current_camera", None)
        if cam and cam in self.camera_settings:
            self.camera_settings[cam]["Video"]["resolution"] = str(value)
        self.resolutionChanged.emit(value)

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

    def set_camera_prefix_widget(self, widget: QWidget) -> None:
        """Insert a widget before the Camera label (e.g., map button)."""
        if widget is None:
            return
        self._camera_layout.insertWidget(0, widget)

    # --- Helper methods for per-camera settings ---
    def _default_settings(self) -> dict:
        return {
            "Video": {"framerate": 30, "resolution": "640x480"},
            "Image": {"brightness": 0, "zoom": 1},
            "Stream": {"bandwidth": None, "connected": False, "streaming": False},
        }

    def _save_current_camera_settings(self) -> None:
        """Save transient UI values into the settings dict for the current camera."""
        cam = getattr(self, "_current_camera", None)
        if not cam or cam not in self.camera_settings:
            return
        s = self.camera_settings[cam]
        s["Video"]["framerate"] = int(self.framerate_input.value())
        s["Video"]["resolution"] = self.resolution_input.text()
        s["Image"]["brightness"] = int(self.brightness_slider.value())
        s["Image"]["zoom"] = int(self.zoom_slider.value())

    def _load_camera_settings(self, cam: str) -> None:
        """Load settings for given camera into the UI, blocking signals while doing so."""
        if cam not in self.camera_settings:
            return
        s = self.camera_settings[cam]
        # block signals while setting values to avoid emitting events
        self.framerate_input.blockSignals(True)
        self.resolution_input.blockSignals(True)
        self.brightness_slider.blockSignals(True)
        self.zoom_slider.blockSignals(True)
        try:
            self.framerate_input.setValue(int(s["Video"].get("framerate", 30)))
            self.resolution_input.setText(s["Video"].get("resolution", "640x480"))
            self.brightness_slider.setValue(int(s["Image"].get("brightness", 0)))
            self.zoom_slider.setValue(int(s["Image"].get("zoom", 1)))
        finally:
            self.framerate_input.blockSignals(False)
            self.resolution_input.blockSignals(False)
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
        print(f"[DEBUG] Control Panel Camera Changed to: {new_camera}")
        self.cameraChanged.emit(new_camera)

    def get_camera_settings(self, cam: Optional[str] = None) -> dict:
        """Return stored settings for a camera (default: current camera)."""
        if cam is None:
            cam = getattr(self, "_current_camera", None)
        return self.camera_settings.get(cam, {})