"""Video streaming utility.

Placeholder module that would interface with ``ffmpeg`` or other
streaming tools to send video from the rover to the base station.  It
defines an API with `start_stream` and `stop_stream` methods but does
not implement actual streaming in the prototype.
"""

from __future__ import annotations

import subprocess
import threading
import time
from typing import List, Optional


class VideoStreamer:
    """Manage the lifecycle of a video stream using ffmpeg.

    A real implementation would spawn an ``ffmpeg`` process to read
    from the camera device and forward the stream over UDP or TCP.  In
    this prototype the methods simply print log messages.
    """

    def __init__(self, camera_device: str = "/dev/video0", host: str = "127.0.0.1", port: int = 5000) -> None:
        self.camera_device = camera_device
        self.host = host
        self.port = port
        self._process: Optional[subprocess.Popen] = None

    def start_stream(self, bitrate: int = 2000) -> None:
        """Start streaming from the camera to the specified host/port.

        Parameters
        ----------
        bitrate : int
            Target bitrate in kilobits per second.
        """
        # In a future implementation call ffmpeg with appropriate args.
        print(f"[VideoStreamer] Starting stream from {self.camera_device} to {self.host}:{self.port} at {bitrate} kbps")
        # Example (commented out):
        # self._process = subprocess.Popen([
        #     "ffmpeg",
        #     "-f", "v4l2", "-i", self.camera_device,
        #     "-vcodec", "libx264",
        #     "-preset", "ultrafast",
        #     "-tune", "zerolatency",
        #     "-b:v", f"{bitrate}k",
        #     "-f", "mpegts",
        #     f"udp://{self.host}:{self.port}"
        # ])

    def stop_stream(self) -> None:
        """Stop the running stream."""
        print("[VideoStreamer] Stopping stream")
        if self._process is not None:
            self._process.terminate()
            self._process.wait()
            self._process = None