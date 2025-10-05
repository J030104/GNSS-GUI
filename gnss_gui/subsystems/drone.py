"""Placeholder widget for the drone subsystem.

This tab can be used to monitor and control an aerial drone that may be
used in the autonomous navigation or science missions.  The prototype
displays dummy altitude and battery readings.
"""

from typing import Optional

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel

from ..components import LogViewer


class DroneWidget(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout()
        layout.addWidget(QLabel("Drone (placeholder)"))
        self.altitude_label = QLabel("Altitude: -- m")
        layout.addWidget(self.altitude_label)
        self.battery_label = QLabel("Battery: -- %")
        layout.addWidget(self.battery_label)
        self.log_viewer = LogViewer()
        layout.addWidget(self.log_viewer)
        self.setLayout(layout)
        self.log_viewer.append("Drone subsystem not yet implemented.")

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_dummy)
        self.timer.start(2500)

    def _update_dummy(self) -> None:
        import random
        altitude = random.uniform(0, 100)
        battery = random.uniform(20, 100)
        self.altitude_label.setText(f"Altitude: {altitude:.1f} m")
        self.battery_label.setText(f"Battery: {battery:.0f} %")