"""Manage network/SSH connections to the rover.

This module provides a simple abstraction for establishing an SSH
connection to the rover’s onboard computer (e.g. NVIDIA Jetson).  In
the prototype the connection is simulated.  Future implementations
could rely on ``paramiko`` or another SSH library to establish an
actual connection, run commands, and retrieve telemetry.
"""

from __future__ import annotations

import random
import time
from dataclasses import dataclass
from typing import Optional


@dataclass
class SSHConfig:
    host: str
    port: int = 22
    username: str = ""
    password: str = ""


class ConnectionManager:
    """Encapsulate an SSH connection to the rover.

    Parameters
    ----------
    config : SSHConfig
        Connection parameters such as host, port, username and password.
    """

    def __init__(self, config: SSHConfig) -> None:
        self.config = config
        self.connected: bool = False

    def connect(self) -> bool:
        """Attempt to establish the SSH connection.

        Returns
        -------
        bool
            ``True`` if the connection succeeds, ``False`` otherwise.
        """
        # In the prototype we simply simulate a connection attempt.
        time.sleep(0.5)  # simulate network delay
        self.connected = True  # always succeed
        return self.connected

    def disconnect(self) -> None:
        """Close the connection."""
        self.connected = False

    def is_connected(self) -> bool:
        return self.connected

    def get_bandwidth(self) -> float:
        """Return a simulated bandwidth usage in kbps."""
        if not self.connected:
            return 0.0
        # Simulate bandwidth between 500 kbps and 3 Mbps
        return random.uniform(500, 3000)