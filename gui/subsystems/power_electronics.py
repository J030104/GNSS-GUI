"""Placeholder widget for the power & electronics subsystem.

Displays dummy battery levels and system voltages to illustrate how
power telemetry might be visualised.  Teams responsible for the
power/electronics subsystem can expand this with real data and
controls.
"""

from typing import Optional

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QProgressBar


class PowerElectronicsWidget(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout()
        layout.addWidget(QLabel("Power & Electronics (placeholder)"))

        # Simulate battery level
        self.battery_bar = QProgressBar()
        self.battery_bar.setRange(0, 100)
        self.battery_bar.setValue(75)
        self.battery_bar.setFormat("Battery: %p%")
        layout.addWidget(self.battery_bar)

        # Simulate system voltage
        self.voltage_label = QLabel("Voltage: 24.0 V")
        layout.addWidget(self.voltage_label)

        self.setLayout(layout)

        # Timer to animate dummy values
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_dummy)
        self.timer.start(2000)

    def _update_dummy(self) -> None:
        import random

        # Randomly vary battery and voltage to simulate updates
        val = max(0, min(100, self.battery_bar.value() + random.randint(-2, 2)))
        self.battery_bar.setValue(val)
        voltage = 24.0 + random.uniform(-0.1, 0.1)
        self.voltage_label.setText(f"Voltage: {voltage:.1f} V")