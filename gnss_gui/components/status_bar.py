"""Status bar widget.

Displays the current system time and connection status.
A timer updates the time every second.
"""

from typing import Optional

from PyQt5.QtCore import QTimer, QTime
from PyQt5.QtWidgets import QStatusBar, QLabel

from .telemetry_panel import TelemetryPanel


class StatusBar(QStatusBar):
    """Custom QStatusBar showing time and connection status."""

    def __init__(self, parent: Optional[QStatusBar] = None) -> None:
        super().__init__(parent)

        # Create labels
        self._time_label = QLabel()
        self._conn_label = QLabel("Connection: Disconnected")
        self._telemetry_panel = TelemetryPanel()

        # Add widgets to the status bar (aligned right by default)
        self.addPermanentWidget(self._telemetry_panel)
        self.addPermanentWidget(self._time_label)
        self.addPermanentWidget(self._conn_label)

        # Timer to update time every second
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._update_time)
        self._timer.start(1000)
        self._update_time()

    def _update_time(self) -> None:
        """Update the time label."""
        now = QTime.currentTime()
        self._time_label.setText(now.toString("hh:mm:ss"))

    def set_connection_status(self, connected: bool) -> None:
        """Update the connection status label."""
        text = "Connection: Connected" if connected else "Connection: Disconnected"
        self._conn_label.setText(text)

    def set_telemetry(
        self,
        rssi_dbm: Optional[float] = None,
        latency_ms: Optional[float] = None,
        battery_pct: Optional[float] = None,
    ) -> None:
        """Update the telemetry values displayed in the status bar."""
        self._telemetry_panel.set_telemetry(
            rssi_dbm=rssi_dbm,
            latency_ms=latency_ms,
            battery_pct=battery_pct,
        )
