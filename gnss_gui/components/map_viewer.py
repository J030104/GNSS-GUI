"""Simple GNSS map display widget.

This widget provides a rudimentary map for showing the rover's current
position and heading.  In the prototype it draws a white background
with a red triangle indicating the rover’s location and orientation.
Future implementations could replace this with a more sophisticated
map (e.g. overlaying terrain images, waypoints or planned routes).
"""

from typing import Optional, Tuple

import numpy as np

from PyQt5.QtCore import Qt, QRectF, QPointF
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor
from PyQt5.QtWidgets import QWidget


class MapViewer(QWidget):
    """Widget for displaying rover position on a simple map."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setMinimumSize(300, 300)
        # Normalised coordinates of rover within the map (0..1)
        self.rover_pos: Tuple[float, float] = (0.5, 0.5)
        # Orientation in degrees (0 = pointing up)
        self.rover_heading: float = 0.0

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