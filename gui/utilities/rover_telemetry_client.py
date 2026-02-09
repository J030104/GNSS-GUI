"""ROS2 telemetry subscriber for rover diagnostics.

Subscribes to:
  - /rssi (dBm)
  - /latency_ms (ms)
  - /battery (%)
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy

try:
    from std_msgs.msg import Float32
except ImportError:
    Float32 = None

DEFAULT_TOPIC_RSSI = "/rssi"
DEFAULT_TOPIC_LATENCY = "/latency_ms"
DEFAULT_TOPIC_BATTERY = "/battery"
DEFAULT_QOS = 10 # Creates a queue that holds up to 10 messages before dropping 
                 # old ones if your code is processing them too slowly.
DEFAULT_MAX_AGE_SEC = 2.5


@dataclass
class TelemetrySample:
    rssi_dbm: Optional[float]
    latency_ms: Optional[float]
    battery_pct: Optional[float]


class TelemetryClient:
    """ROS2 subscriber client for rover telemetry topics."""

    _instance_lock = threading.Lock()
    _instance: Optional["TelemetryClient"] = None

    @classmethod
    def instance(cls) -> "TelemetryClient":
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    def __init__(
        self,
        node_name: str = "gui_telemetry_client",
        topic_rssi: str = DEFAULT_TOPIC_RSSI,
        topic_latency: str = DEFAULT_TOPIC_LATENCY,
        topic_battery: str = DEFAULT_TOPIC_BATTERY,
        qos_depth: int = DEFAULT_QOS,
        max_age_sec: float = DEFAULT_MAX_AGE_SEC,
    ) -> None:
        if Float32 is None:
            raise RuntimeError(
                "TelemetryClient: std_msgs not available. Is your ROS2 env sourced?"
            )

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node(node_name)
        self._lock = threading.Lock()
        self._rssi_dbm: Optional[float] = None
        self._latency_ms: Optional[float] = None
        self._battery_pct: Optional[float] = None
        self._rssi_ts = 0.0
        self._latency_ts = 0.0
        self._battery_ts = 0.0

        # Bandwidth tracking
        self._bytes_received = 0
        self._last_bw_time = time.time()
        self._bandwidth_kbps = 0.0

        self._topic_rssi = topic_rssi
        self._topic_latency = topic_latency
        self._topic_battery = topic_battery
        self._qos_depth = int(qos_depth)
        self._max_age_sec = float(max_age_sec)

        self._node.create_subscription(Float32, self._topic_rssi, self._on_rssi, self._qos_depth)
        self._node.create_subscription(Float32, self._topic_latency, self._on_latency, self._qos_depth)
        self._node.create_subscription(Float32, self._topic_battery, self._on_battery, self._qos_depth)

        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def get_bandwidth(self) -> float:
        """Return the current telemetry bandwidth usage in kbps."""
        with self._lock:
            # Force an update if idle
            self._update_bw_calc()
            return self._bandwidth_kbps

    def _update_bw_calc(self) -> None:
        """Update the bandwidth calculation based on time elapsed."""
        now = time.time()
        elapsed = now - self._last_bw_time
        if elapsed >= 1.0:
            # kbps = (bytes * 8) / (1000 * elapsed)
            self._bandwidth_kbps = (self._bytes_received * 8) / (1000.0 * elapsed)
            self._bytes_received = 0
            self._last_bw_time = now

    def _record_bytes(self, count: int) -> None:
        with self._lock:
            self._bytes_received += count
            self._update_bw_calc()

    def _spin(self) -> None:
        try:
            rclpy.spin(self._node)
        except Exception:
            pass

# --- Callbacks for each telemetry topic ---

    def _on_rssi(self, msg: Float32) -> None:
        self._record_bytes(4)
        with self._lock:
            self._rssi_dbm = float(msg.data)
            self._rssi_ts = time.time()

    def _on_latency(self, msg: Float32) -> None:
        self._record_bytes(4)
        with self._lock:
            self._latency_ms = float(msg.data)
            self._latency_ts = time.time()

    def _on_battery(self, msg: Float32) -> None:
        self._record_bytes(4)
        with self._lock:
            self._battery_pct = float(msg.data)
            self._battery_ts = time.time()

    def get_latest(self, max_age_sec: Optional[float] = None) -> Optional[TelemetrySample]:
        if max_age_sec is None:
            max_age_sec = self._max_age_sec
        now = time.time()
        with self._lock:
            rssi = self._rssi_dbm if now - self._rssi_ts <= max_age_sec else None
            latency = self._latency_ms if now - self._latency_ts <= max_age_sec else None
            battery = self._battery_pct if now - self._battery_ts <= max_age_sec else None

        if rssi is None and latency is None and battery is None:
            return None
        return TelemetrySample(rssi_dbm=rssi, latency_ms=latency, battery_pct=battery)

    def shutdown(self) -> None:
        """Optional: clean shutdown when GUI exits."""
        try:
            self._node.destroy_node()
        except Exception:
            pass
        # Intentionally do not call rclpy.shutdown(); other clients may share it.
