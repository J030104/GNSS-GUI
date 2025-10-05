"""Video viewing widget for the URC control GUI.

This component encapsulates a simple video display area.  For the
prototype a static placeholder image is shown and updated every second
to demonstrate that the widget can refresh its content.  In a future
implementation the ``update_frame`` method can be connected to a
real video stream (e.g. via OpenCV or ffmpeg) to display live
imagery from the rover's cameras.
"""

import os
import sys
import time
from typing import Optional

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtWidgets import (
    QWidget,
    QLabel,
    QSizePolicy,
    QVBoxLayout,
)

import numpy as np


class VideoViewer(QWidget):
    """A widget that displays video frames.

    Attributes
    ----------
    fps : int
        How many times per second the widget should refresh.  For the
        prototype this simply updates the placeholder image to a new
        random noise pattern.  When connected to a real stream this
        should match the incoming frame rate.
    """

    def __init__(self, fps: int = 10, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.fps = fps
        self.label = QLabel(self)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

        # placeholder frame data
        self._placeholder_size = (320, 240)

        # Start a timer to periodically update the frame
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._on_timer)
        self._timer.start(int(1000 / self.fps))

        # Frame generator function; can be overridden later
        self.frame_generator = self._generate_placeholder_frame

    def _on_timer(self) -> None:
        """Refresh the displayed frame.

        Called periodically by the internal timer.  It retrieves the
        next frame from the configured frame generator and updates the
        QLabel.
        """
        frame = self.frame_generator()
        if frame is not None:
            self.update_frame(frame)

    def _generate_placeholder_frame(self) -> np.ndarray:
        """Generate a pseudo‑random image for demonstration purposes.

        Returns
        -------
        np.ndarray
            A ``(H, W, 3)`` array of uint8 representing an RGB image.
        """
        h, w = self._placeholder_size
        # Create a gradient pattern that changes over time
        t = int(time.time() * 10) % 255
        gradient = np.linspace(0, 255, w, dtype=np.uint8)
        frame = np.tile(gradient, (h, 1))
        frame = np.stack([frame, np.roll(frame, t, axis=0), np.roll(frame, t // 2, axis=0)], axis=2)
        return frame

    def update_frame(self, frame: np.ndarray) -> None:
        """Update the label with a new image.

        Parameters
        ----------
        frame : np.ndarray
            A ``(H, W, 3)`` array of uint8 representing an RGB image.
        """
        # Convert NumPy array to QImage
        h, w, c = frame.shape
        bytes_per_line = c * w
        image = QImage(frame.data.tobytes(), w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(image)
        self.label.setPixmap(pixmap.scaled(self.label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def resizeEvent(self, event):  # type: ignore[override]
        """Ensure the current pixmap scales when the widget is resized."""
        if (pixmap := self.label.pixmap()) is not None:
            self.label.setPixmap(pixmap.scaled(self.label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        super().resizeEvent(event)