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
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
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

        # Image adjustment state
        # brightness: integer offset added to each channel (-50..50)
        self._brightness = 0
        # zoom: 1..n; when >1 we crop then scale. Stored as float for math.
        self._zoom = 1.0
        # pan: normalized top-left fraction (0..1) of the crop window
        self._pan_x = 0.5
        self._pan_y = 0.5
        # dragging state for mouse panning
        self._dragging = False
        self._drag_start_pos = None

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

        # Apply brightness and zoom/pan to a working copy of the frame
        try:
            working = frame.astype(np.int16, copy=True)
        except Exception:
            # Fallback if dtype conversions fail
            working = frame.copy()

        # Brightness: add offset then clip
        if getattr(self, '_brightness', 0) != 0:
            working = working + int(self._brightness)
            np.clip(working, 0, 255, out=working)

        # Convert back to uint8 for later operations
        working = working.astype(np.uint8)

        h, w, c = working.shape

        # Zoom / crop
        z = max(1.0, float(self._zoom))
        if z > 1.0:
            cw = max(1, int(round(w / z)))
            ch = max(1, int(round(h / z)))
            # Compute top-left based on normalized pan (centered by default)
            max_x = max(0, w - cw)
            max_y = max(0, h - ch)
            # pan_x, pan_y are normalized center positions in [0,1]
            # Convert to top-left
            tx = int(round((self._pan_x) * max_x))
            ty = int(round((self._pan_y) * max_y))
            # Ensure within bounds
            tx = min(max(0, tx), max_x)
            ty = min(max(0, ty), max_y)
            working = working[ty:ty + ch, tx:tx + cw]

        # Convert NumPy array to QImage
        bytes_per_line = c * working.shape[1]
        image = QImage(working.data.tobytes(), working.shape[1], working.shape[0], bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(image)
        self.label.setPixmap(pixmap.scaled(self.label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def resizeEvent(self, event):  # type: ignore[override]
        """Ensure the current pixmap scales when the widget is resized."""
        if (pixmap := self.label.pixmap()) is not None:
            self.label.setPixmap(pixmap.scaled(self.label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        super().resizeEvent(event)

    # --- Public APIs for adjustments ---
    def set_brightness(self, value: int) -> None:
        """Set brightness offset (-50..50)."""
        try:
            self._brightness = int(value)
        except Exception:
            self._brightness = 0

    def set_zoom(self, value: int) -> None:
        """Set zoom factor (1..n). Resets pan to center when zoom changes."""
        try:
            z = float(value)
            if z < 1.0:
                z = 1.0
            self._zoom = z
        except Exception:
            self._zoom = 1.0
        # center pan when zoom changes
        self._pan_x = 0.5
        self._pan_y = 0.5

    def get_pan(self) -> tuple:
        """Return the current normalized pan (pan_x, pan_y) in [0,1]."""
        return (float(self._pan_x), float(self._pan_y))

    def set_pan(self, x: float, y: float) -> None:
        """Set the current normalized pan (clamped to [0,1])."""
        try:
            self._pan_x = min(max(0.0, float(x)), 1.0)
            self._pan_y = min(max(0.0, float(y)), 1.0)
        except Exception:
            self._pan_x = 0.5
            self._pan_y = 0.5

    # --- Mouse handling for panning when zoomed ---
    def mousePressEvent(self, event):  # type: ignore[override]
        if event.button() == Qt.LeftButton and getattr(self, '_zoom', 1.0) > 1.0:
            self._dragging = True
            self._drag_start_pos = event.pos()
            event.accept()
        else:
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):  # type: ignore[override]
        if self._dragging and self._drag_start_pos is not None:
            # Compute delta in widget coordinates
            dx = event.pos().x() - self._drag_start_pos.x()
            dy = event.pos().y() - self._drag_start_pos.y()
            lw = max(1, self.label.width())
            lh = max(1, self.label.height())
            # Translate widget pixel delta to pan fraction
            # moving right should shift crop left (show area to the right)
            delta_pan_x = -dx / lw
            delta_pan_y = -dy / lh
            # Scale delta by inverse zoom so dragging speed feels natural
            inv_zoom = 1.0 / max(1.0, float(self._zoom))
            self._pan_x = min(max(0.0, self._pan_x + delta_pan_x * inv_zoom), 1.0)
            self._pan_y = min(max(0.0, self._pan_y + delta_pan_y * inv_zoom), 1.0)
            self._drag_start_pos = event.pos()
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):  # type: ignore[override]
        if event.button() == Qt.LeftButton and self._dragging:
            self._dragging = False
            self._drag_start_pos = None
            event.accept()
            return
        super().mouseReleaseEvent(event)