"""Simple GNSS map display widget.

This widget provides a rudimentary map for showing the rover's current
position and heading.  In the prototype it draws a white background
with a red triangle indicating the rover’s location and orientation.
Future implementations could replace this with a more sophisticated
map (e.g. overlaying terrain images, waypoints or planned routes).
"""

from typing import Optional, Tuple

import numpy as np

from PyQt5.QtCore import Qt, QRectF, QPointF, QPoint, QRect
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QCursor
from PyQt5.QtWidgets import QWidget


class MapViewer(QWidget):
    """Widget for displaying rover position on a simple map."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setMinimumSize(200, 200)
        # Normalised coordinates of rover within the map (0..1)
        self.rover_pos: Tuple[float, float] = (0.5, 0.5)
        # Orientation in degrees (0 = pointing up)
        self.rover_heading: float = 0.0
        # Drag state
        self._drag_active = False
        self._drag_offset = QPoint(0, 0)
        # Resize state
        self._resizing = False
        self._resize_start_global = QPoint(0, 0)
        self._initial_size = self.size()
        # Size (in pixels) of the draggable grip area in the bottom-right corner
        self._resize_grip_size = 12
        # Track the mouse even when no button is pressed so we can change cursor
        self.setMouseTracking(True)

    def set_position(self, latitude: float, longitude: float) -> None:
        """Update rover position on the map.

        For the prototype the coordinates are normalised into the range
        [0,1] by discarding the absolute lat/lon; in a real
        implementation you would project the GNSS coordinates onto
        your map view.
        """
        # For demonstration just oscillate within bounds based on lat/lon
        self.rover_pos = (
            (latitude % 1.0),
            (longitude % 1.0),
        )
        self.update()

    def set_heading(self, heading_degrees: float) -> None:
        """Set the rover's orientation (0 degrees = up)."""
        self.rover_heading = heading_degrees
        self.update()

    def paintEvent(self, event):  # type: ignore[override]
        """Custom paint routine to draw map background and rover marker."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Draw background
        painter.fillRect(self.rect(), QBrush(QColor(240, 240, 240)))

        # Draw a border
        pen = QPen(QColor(200, 200, 200))
        pen.setWidth(1)
        painter.setPen(pen)
        painter.drawRect(self.rect().adjusted(0, 0, -1, -1))

        # Compute rover coordinates in widget space
        w, h = self.width(), self.height()
        x = self.rover_pos[0] * w
        y = self.rover_pos[1] * h

        # Draw rover orientation as a triangle
        size = min(w, h) * 0.05
        angle_rad = np.deg2rad(self.rover_heading)
        # Base triangle pointing up at origin; three vertices
        pts = np.array([
            [0, -size],
            [-size * 0.5, size * 0.5],
            [size * 0.5, size * 0.5],
        ])
        # Rotate and translate
        rot = np.array([
            [np.cos(angle_rad), -np.sin(angle_rad)],
            [np.sin(angle_rad), np.cos(angle_rad)],
        ])
        transformed = (pts @ rot) + np.array([x, y])

        painter.setBrush(QBrush(QColor(200, 0, 0)))
        painter.setPen(QPen(QColor(150, 0, 0)))
        painter.drawPolygon(*[QPointF(px, py) for px, py in transformed])

        # Draw a small resize grip in the bottom-right corner
        grip_size = self._resize_grip_size
        grip_rect = QRect(self.width() - grip_size - 2, self.height() - grip_size - 2, grip_size, grip_size)
        painter.setBrush(QBrush(QColor(220, 220, 220)))
        painter.setPen(QPen(QColor(160, 160, 160)))
        painter.drawRect(grip_rect)

    # --- Drag handlers --------------------------------------------------------
    def mousePressEvent(self, event) -> None:  # type: ignore[override]
        if event.button() == Qt.LeftButton:
            # If click is in the resize grip, start resizing instead of dragging
            if self._in_resize_area(event.pos()):
                self._resizing = True
                self._resize_start_global = event.globalPos()
                self._initial_size = self.size()
                self.setCursor(QCursor(Qt.SizeFDiagCursor))
                return

            # Otherwise start dragging the widget
            self._drag_active = True
            # store local offset so movement feels natural
            self._drag_offset = event.pos()
            self.setCursor(QCursor(Qt.ClosedHandCursor))
        return super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:  # type: ignore[override]
        # If resizing, adjust the widget size based on mouse movement
        if self._resizing:
            parent = self.parentWidget()
            delta = event.globalPos() - self._resize_start_global
            new_w = max(self.minimumWidth(), self._initial_size.width() + delta.x())
            new_h = max(self.minimumHeight(), self._initial_size.height() + delta.y())
            # If we have a parent, clamp so the widget doesn't escape the parent
            if parent is not None:
                max_w = parent.width() - self.x()
                max_h = parent.height() - self.y()
                new_w = min(new_w, max_w)
                new_h = min(new_h, max_h)
            self.resize(new_w, new_h)
            return

        # If dragging, move widget within parent coordinates
        if self._drag_active:
            parent = self.parentWidget()
            if parent is not None:
                # Compute new top-left in parent's coordinate space
                top_left_global = event.globalPos() - self._drag_offset
                new_pos = parent.mapFromGlobal(top_left_global)
                # clamp to parent rect so it remains visible
                x = max(0, min(new_pos.x(), parent.width() - self.width()))
                y = max(0, min(new_pos.y(), parent.height() - self.height()))
                self.move(x, y)
            else:
                # No parent: move using global coordinates
                self.move(event.globalPos() - self._drag_offset)
            return

        # Not dragging nor resizing: update cursor if hovering over grip
        if self._in_resize_area(event.pos()):
            self.setCursor(QCursor(Qt.SizeFDiagCursor))
        else:
            self.setCursor(QCursor(Qt.ArrowCursor))

        return super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:  # type: ignore[override]
        # Reset active flags and cursor
        self._drag_active = False
        if self._resizing:
            self._resizing = False
            self.setCursor(QCursor(Qt.ArrowCursor))
            return

        self.setCursor(QCursor(Qt.ArrowCursor))
        return super().mouseReleaseEvent(event)

    def _in_resize_area(self, pos: QPoint) -> bool:
        """Return True if a QPoint (widget-local) is inside the bottom-right grip."""
        return pos.x() >= self.width() - self._resize_grip_size and pos.y() >= self.height() - self._resize_grip_size

    def leaveEvent(self, event) -> None:  # type: ignore[override]
        # Restore default cursor when leaving widget bounds (unless actively dragging/resizing)
        if not (self._drag_active or self._resizing):
            self.setCursor(QCursor(Qt.ArrowCursor))
        return super().leaveEvent(event)