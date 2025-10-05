"""Control panel for adjusting video and communication settings.

This widget groups together various input controls that can be used to
adjust properties of the rover’s video streams and monitor
communication parameters.  It emits signals when values change so
other parts of the application can react accordingly.
"""

from typing import Optional

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
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
        self.camera_combo.addItems(["Camera 1", "Camera 2", "Camera 3", "Insta 360"])
        self.camera_combo.currentTextChanged.connect(self.cameraChanged.emit)
        camera_layout.addWidget(camera_label)
        camera_layout.addWidget(self.camera_combo)
        layout.addLayout(camera_layout)

        # Bitrate control
        bitrate_group = QGroupBox("Video bitrate (kbps)")
        bitrate_form = QFormLayout()
        self.bitrate_slider = QSlider(Qt.Horizontal)
        self.bitrate_slider.setRange(500, 5000)
        self.bitrate_slider.setValue(2000)
        bitrate_form.addRow(self.bitrate_slider)
        bitrate_group.setLayout(bitrate_form)
        self.bitrate_slider.valueChanged.connect(self._on_bitrate_changed)
        layout.addWidget(bitrate_group)

        # Brightness control
        brightness_group = QGroupBox("Brightness")
        brightness_form = QFormLayout()
        self.brightness_slider = QSlider(Qt.Horizontal)
        self.brightness_slider.setRange(-50, 50)
        self.brightness_slider.setValue(0)
        brightness_form.addRow(self.brightness_slider)
        brightness_group.setLayout(brightness_form)
        self.brightness_slider.valueChanged.connect(self.brightnessChanged.emit)
        layout.addWidget(brightness_group)

        # Zoom control
        zoom_group = QGroupBox("Zoom")
        zoom_form = QFormLayout()
        self.zoom_slider = QSlider(Qt.Horizontal)
        self.zoom_slider.setRange(1, 10)
        self.zoom_slider.setValue(1)
        zoom_form.addRow(self.zoom_slider)
        zoom_group.setLayout(zoom_form)
        self.zoom_slider.valueChanged.connect(self.zoomChanged.emit)
        layout.addWidget(zoom_group)

        # Stream controls
        stream_layout = QHBoxLayout()
        self.start_button = QPushButton("Start Stream")
        self.stop_button = QPushButton("Stop Stream")
        self.start_button.clicked.connect(self.startStreamRequested.emit)
        self.stop_button.clicked.connect(self.stopStreamRequested.emit)
        stream_layout.addWidget(self.start_button)
        stream_layout.addWidget(self.stop_button)
        layout.addLayout(stream_layout)

        # Bandwidth and connection status display
        status_layout = QHBoxLayout()
        self.bandwidth_label = QLabel("Bandwidth: -- kbps")
        self.conn_status_label = QLabel("Connection: Disconnected")
        status_layout.addWidget(self.bandwidth_label)
        status_layout.addWidget(self.conn_status_label)
        layout.addLayout(status_layout)

        layout.addStretch(1)
        self.setLayout(layout)

    def _on_bitrate_changed(self, value: int) -> None:
        """Internal slot to normalise bitrate value and emit signal."""
        self.bitrateChanged.emit(value)

    # Public methods to update status labels
    def set_bandwidth(self, kbps: float) -> None:
        self.bandwidth_label.setText(f"Bandwidth: {kbps:.1f} kbps")

    def set_connection_status(self, connected: bool) -> None:
        self.conn_status_label.setText(
            "Connection: Connected" if connected else "Connection: Disconnected"
        )