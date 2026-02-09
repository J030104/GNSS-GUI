"""Placeholder widget for the autonomous navigation subsystem.

This tab provides a starting point for teams implementing the
autonomous navigation subsystem.  It contains a map view and a log
viewer as examples of how other components could be integrated.
"""

from typing import Optional

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel

from ..components import MapViewer, LogViewer, ShellTabs


class AutonomousNavigationWidget(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout()
        layout.addWidget(QLabel("Autonomous Navigation (placeholder)"))
        # Use a placeholder in the layout and create the MapViewer as a
        # floating child so the user can move/resize it without the
        # layout resetting its geometry.
        self.map_placeholder = QWidget()
        layout.addWidget(self.map_placeholder)
        self.log_viewer = LogViewer()
        # Use a tabbed container that contains the log plus optional shells
        self.shell_tabs = ShellTabs(log_viewer=self.log_viewer)
        layout.addWidget(self.shell_tabs)
        self.setLayout(layout)
        # Create the floating MapViewer and position it after layout
        self.map_viewer = MapViewer(parent=self)
        from PyQt5.QtCore import QTimer

        QTimer.singleShot(0, lambda: self._place_map())
        self.log_viewer.append("Autonomous navigation subsystem not yet implemented.")

    def _place_map(self) -> None:
        try:
            geom = self.map_placeholder.geometry()
            self.map_viewer.setGeometry(geom)
            self.map_viewer.show()
            self.map_viewer.raise_()
        except Exception:
            pass