"""
RoverStreamClient: ROS2 client utilities for controlling rover video streams.

This module is designed to fit into the existing GNSS-GUI project
without requiring changes to other modules' public interfaces.

Usage pattern (typical):

    client = RoverStreamClient()
    url = client.start_stream("cam_front", bitrate_kbps=2000, codec="libx264")
    # url might be "udp://jetson.local:5000" or "rtsp://jetson.local:8554/cam_front"
    # You then pass that URL into NetworkStreamOptions/NetworkStreamCamera.

The client is synchronous for simplicity. Service calls are short,
so blocking the UI for ~100 ms is usually tolerable; if needed you
can move calls into a QThread later without changing this API.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
except ImportError:
    rclpy = None
    Node = object

try:
    # Adjust import paths to your package names if different
    from multi_cam_streamer.srv import StartCameraStream, StopCameraStream, GetCameraStreamStatus
except ImportError:
    StartCameraStream = None
    StopCameraStream = None
    GetCameraStreamStatus = None


@dataclass
class StreamRequestParams:
    camera_id: str
    bitrate_kbps: int = 2000
    codec: str = "libx264"
    resolution: str = "1280x720"
    transport: str = "udp"   # "rtsp" / "udp" / "srt"

class RoverStreamClient:
    """
    Thin wrapper around ROS2 services exposed by MultiCameraStreamNode.

    This is intentionally simple: one Node in a background thread with
    blocking service calls from the GUI thread.
    """

    _instance_lock = threading.Lock()
    _instance: Optional["RoverStreamClient"] = None

    @classmethod
    def instance(cls) -> "RoverStreamClient":
        """Singleton accessor (optional, but convenient)."""
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    def __init__(self, node_name: str = "gnss_gui_rover_client") -> None:
        # Check if we have ROS2 and the service definitions
        if rclpy is None or StartCameraStream is None:
            # Fallback to offline/mock mode
            self._mock_mode = True
            print(f"[{node_name}] ROS2 or service definitions missing. Running in MOCK mode.")
            return

        self._mock_mode = False

        # Ensure ROS2 global context exists
        if not rclpy.ok():
            rclpy.init()

        # Create a dedicated Node
        self._node = rclpy.create_node(node_name)

        # Service clients
        self._cli_start = self._node.create_client(StartCameraStream, "camera_manager/start_stream")
        self._cli_stop = self._node.create_client(StopCameraStream, "camera_manager/stop_stream")
        self._cli_status = self._node.create_client(GetCameraStreamStatus, "camera_manager/get_status")

        # Background spinning thread
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    # ------------------------------------------------------------------ internals

    def _spin(self) -> None:
        """Spin the node to process incoming service responses."""
        if self._mock_mode:
            while True:
                time.sleep(1)
            return

        try:
            rclpy.spin(self._node)
        except Exception:
            # In GUI context we don't crash; you can add logging here.
            pass

    def _call_service(self, client, req, timeout: float = 3.0):
        """Generic synchronous service call with timeout."""
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("Service not available")

        future = client.call_async(req)

        # Busy-wait with spin_once; we can't block the global spin() here.
        elapsed = 0.0
        step = 0.05
        while not future.done() and elapsed < timeout:
            rclpy.spin_once(self._node, timeout_sec=step)
            elapsed += step

        if not future.done():
            raise TimeoutError("Service call timed out")

        return future.result()

    # ------------------------------------------------------------------ API

    def start_stream(self, params: StreamRequestParams, timeout: float = 1.0) -> str:
        """
        Start the stream for the given camera.

        Returns
        -------
        str
            The stream URL (e.g. "udp://jetson.local:5000" or RTSP URL)
            to be used with NetworkStreamOptions/NetworkStreamCamera.
        """
        if self._mock_mode:
            # Use 0.0.0.0 to listen on all interfaces (local and network)
            print(f"[MOCK] start_stream({params.camera_id}) -> udp://0.0.0.0:5000")
            return "udp://0.0.0.0:5000"

        req = StartCameraStream.Request()
        req.camera_id = params.camera_id
        req.codec = params.codec
        req.resolution = params.resolution
        req.bitrate_kbps = int(params.bitrate_kbps)
        req.transport = params.transport

        resp = self._call_service(self._cli_start, req, timeout=timeout)
        if not getattr(resp, "success", False):
            raise RuntimeError(f"Failed to start stream: {getattr(resp, 'message', 'unknown error')}")
        return getattr(resp, "stream_url", "")

    def stop_stream(self, camera_id: str) -> None:
        """Stop streaming for a given camera."""
        if self._mock_mode:
            print(f"[MOCK] stop_stream({camera_id})")
            return

        req = StopCameraStream.Request()
        req.camera_id = camera_id
        resp = self._call_service(self._cli_stop, req)
        if not getattr(resp, "success", False):
            # Not fatal for GUI; we just log on caller side
            raise RuntimeError(getattr(resp, "message", "Failed to stop stream"))

    def get_status(self, camera_id: str):
        """Return status tuple (known, streaming, url, last_error)."""
        if self._mock_mode:
            return (True, False, "", "")

        req = GetCameraStreamStatus.Request()
        req.camera_id = camera_id
        resp = self._call_service(self._cli_status, req)
        return (
            getattr(resp, "known", False),
            getattr(resp, "streaming", False),
            getattr(resp, "stream_url", ""),
            getattr(resp, "last_error", ""),
        )

    def shutdown(self) -> None:
        """Optional: clean shutdown when GUI exits."""
        if self._mock_mode:
            return

        try:
            self._node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
