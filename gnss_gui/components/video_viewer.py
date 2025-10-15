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

from ..utilities.video_streamer import CameraSource


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

    def __init__(self, fps: int = 10, parent: Optional[QWidget] = None, camera_name: str = "Camera") -> None:
        super().__init__(parent)
        self.fps = fps
        self.camera_name = camera_name
        self.label = QLabel(self)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

        # Allow video boxes to stretch to fill available space
        self.setMinimumSize(160, 120)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # placeholder frame data
        self._placeholder_size = (160, 120)

        # Start a timer to periodically update the frame
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._on_timer)
        self._timer.start(int(1000 / self.fps))

        # Frame generator function; can be overridden later
        # By default use placeholder generator; callers may set
        # ``video_viewer.attach_camera(camera_source)`` to attach a
        # CameraSource instance which will be polled for real frames.
        self.frame_generator = self._generate_placeholder_frame
        self._camera_source: Optional[CameraSource] = None
        self._show_text_placeholder = True
        
        # Set initial text placeholder
        self._show_text_placeholder_message()

    def attach_camera(self, source: Optional[CameraSource]) -> None:
        """Attach a CameraSource to the viewer.

        If ``source`` is ``None`` the viewer will fall back to the
        placeholder frame generator.  When a source is attached the
        viewer calls ``start()`` on it and polls ``read_frame`` for
        frames.
        """
        # Do not stop the previously attached camera; keep it running so
        # switching back is instant. Just replace the pointer we use for
        # rendering. This trades a small amount of resource usage for
        # responsiveness during development/testing.
        prev = self._camera_source
        self._camera_source = source
        if source is None:
            self.frame_generator = self._generate_placeholder_frame
            self._show_text_placeholder = True
            self._show_text_placeholder_message()
        else:
            self._show_text_placeholder = False
            try:
                # Start the source if it defines a start method and appears
                # not to be already running. Implementations should make
                # start() idempotent; we call it defensively here.
                try:
                    source.start()
                except Exception:
                    pass
                self.frame_generator = source.read_frame
            except Exception:
                # If attaching the source fails, restore previous source
                # pointer and fall back to placeholder
                self._camera_source = prev
                self.frame_generator = self._generate_placeholder_frame
                self._show_text_placeholder = True
                self._show_text_placeholder_message()

    def _on_timer(self) -> None:
        """Refresh the displayed frame.

        Called periodically by the internal timer.  It retrieves the
        next frame from the configured frame generator and updates the
        QLabel.
        """
        if self._show_text_placeholder:
            # Don't update when showing text placeholder
            return
            
        frame = self.frame_generator()
        if frame is not None:
            self.update_frame(frame)

    def _show_text_placeholder_message(self) -> None:
        """Show a text message when no video stream is active."""
        self.label.clear()
        self.label.setText(f"{self.camera_name}\nUnavailable")
        self.label.setStyleSheet("""
            QLabel {
                background-color: #2b2b2b;
                color: #ffffff;
                border: 1px solid #555555;
                font-size: 12px;
                font-weight: bold;
            }
        """)

    def set_camera_name(self, name: str) -> None:
        """Set the camera name for display in placeholder."""
        self.camera_name = name
        if self._show_text_placeholder:
            self._show_text_placeholder_message()

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
        # Clear any text styling when showing video
        self.label.setStyleSheet("")
        
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