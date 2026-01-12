"""Telemetry overlay panel for GNSSCommWidget."""

from typing import Optional

from PyQt5.QtWidgets import QFrame, QLabel, QHBoxLayout


class TelemetryPanel(QFrame):
    """Compact telemetry panel for RSSI, latency, and battery."""

    def __init__(self, parent: Optional[QFrame] = None) -> None:
        super().__init__(parent)
        self.setObjectName("telemetryPanel")
        self.setStyleSheet("""
            QFrame#telemetryPanel {
                background-color: rgba(0, 0, 0, 150);
                border: 1px solid #3a3a3a;
                border-radius: 6px;
            }
            QFrame#telemetryPanel QLabel {
                color: #ffffff;
                font-size: 11px;
            }
        """)
        layout = QHBoxLayout()
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(10)
        self._rssi_label = QLabel("RSSI: -- dBm")
        self._sep_one = QLabel("|")
        self._latency_label = QLabel("Latency: -- ms")
        self._sep_two = QLabel("|")
        self._battery_label = QLabel("Battery: -- %")
        layout.addWidget(self._rssi_label)
        layout.addWidget(self._sep_one)
        layout.addWidget(self._latency_label)
        layout.addWidget(self._sep_two)
        layout.addWidget(self._battery_label)
        self.setLayout(layout)

    def _format_rssi(self, rssi_dbm: Optional[float]) -> str:
        if rssi_dbm is None:
            return "RSSI: -- dBm"
        return f"RSSI: {rssi_dbm:.0f} dBm"

    def _format_latency(self, latency_ms: Optional[float]) -> str:
        if latency_ms is None:
            return "Latency: -- ms"
        return f"Latency: {latency_ms:.0f} ms"

    def _format_battery(self, battery_pct: Optional[float]) -> str:
        if battery_pct is None:
            return "Battery: -- %"
        return f"Battery: {battery_pct:.0f} %"

    def set_telemetry(
        self,
        rssi_dbm: Optional[float] = None,
        latency_ms: Optional[float] = None,
        battery_pct: Optional[float] = None,
    ) -> None:
        """Update the telemetry values displayed in the panel."""
        self._rssi_label.setText(self._format_rssi(rssi_dbm))
        self._latency_label.setText(self._format_latency(latency_ms))
        self._battery_label.setText(self._format_battery(battery_pct))
