"""Manage network connections/bandwidth aggregation.

This module provides a simple abstraction for tracking bandwidth usage.
"""

from __future__ import annotations

import random
import time
from dataclasses import dataclass
from typing import Optional

# Import other utilities for bandwidth aggregation
# Use try-except to avoid circular imports if any, or import errors during testing
try:
    from .rover_telemetry_client import TelemetryClient
    from .video_streamer import CameraManager
except ImportError:
    TelemetryClient = None
    CameraManager = None



class ConnectionManager:

    """Manage connection status and bandwidth stats."""

    def __init__(self) -> None:
        self.connected: bool = True

    def connect(self) -> bool:
        """No-op for compatibility."""
        self.connected = True
        return True

    def disconnect(self) -> None:
        """Close the connection."""
        self.connected = False

    def is_connected(self) -> bool:
        return self.connected

    def get_bandwidth(self) -> float:
        """Return the aggregated bandwidth usage in kbps.
        
        If real telemetry/video data is detected, returns the actual usage.
        Otherwise falls back to simulation for demo purposes.
        """
        real_bandwidth = 0.0
        has_real_sources = False

        # Get telemetry bandwidth
        if TelemetryClient:
            try:
                client = TelemetryClient.instance()
                if client:
                    bw = client.get_bandwidth()
                    real_bandwidth += bw
                    if bw > 0:
                        has_real_sources = True
            except Exception:
                pass

        # Get video bandwidth
        if CameraManager:
            try:
                bw = CameraManager.get_total_bandwidth()
                real_bandwidth += bw
                if bw > 0:
                    has_real_sources = True
            except Exception:
                pass

        # If we detected any real data flow, return it
        if has_real_sources:
            return real_bandwidth

        if not self.connected:
            return 0.0
        # Simulate bandwidth between 500 kbps and 3 Mbps
        return random.uniform(500, 3000)