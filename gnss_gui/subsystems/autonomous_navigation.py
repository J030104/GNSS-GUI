"""Placeholder widget for the autonomous navigation subsystem.

This tab provides a starting point for teams implementing the
autonomous navigation subsystem.  It contains a map view and a log
viewer as examples of how other components could be integrated.
"""

from typing import Optional

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel

from ..components import MapViewer, LogViewer


class AutonomousNavigationWidget(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout()
        layout.addWidget(QLabel("Autonomous Navigation (placeholder)"))
        self.map_viewer = MapViewer()
        layout.addWidget(self.map_viewer)
        self.log_viewer = LogViewer()
        layout.addWidget(self.log_viewer)
        self.setLayout(layout)
        self.log_viewer.append("Autonomous navigation subsystem not yet implemented.")