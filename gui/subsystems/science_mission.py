"""Placeholder widget for the science mission subsystem.

Shows dummy sensor readings and a log viewer.  Teams can integrate
real instruments (e.g. spectrometers, microscopes) and control
interfaces in this tab.
"""

from typing import Optional

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel

from ..components import LogViewer


class ScienceMissionWidget(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout()
        layout.addWidget(QLabel("Science Mission (placeholder)"))
        self.sensor_label = QLabel("Temperature: -- °C, Humidity: -- %")
        layout.addWidget(self.sensor_label)
        self.log_viewer = LogViewer()
        layout.addWidget(self.log_viewer)
        self.setLayout(layout)
        self.log_viewer.append("Science mission subsystem not yet implemented.")

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_dummy)
        self.timer.start(3000)

    def _update_dummy(self) -> None:
        import random
        temp = 20.0 + random.uniform(-5, 5)
        humidity = 40.0 + random.uniform(-10, 10)
        self.sensor_label.setText(f"Temperature: {temp:.1f} °C, Humidity: {humidity:.1f} %")