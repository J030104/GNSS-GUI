#!/usr/bin/env python3
"""
Mock Rover Camera Node for Testing GNSS-GUI.

This script simulates the 'multi_cam_streamer' ROS2 node that runs on the rover.
It streams synthetic video (test patterns) using FFmpeg.

REQUIREMENTS:
1. ROS2 (Humble or Jazzy) installed OPTIONAL.
2. FFmpeg installed (`sudo apt install ffmpeg`).
3. If ROS2 is present, 'multi_cam_streamer' package must be built.
   If ROS2 is NOT present, runs in Standalone Mode (starts streams auto).

USAGE:
1. Configure the `CONFIG` dictionary below.
2. Run this script:
   $ python3 mock_rover_node.py
"""

import subprocess
import threading
import time
import socket
import sys

# Try importing ROS2 libraries; if missing, we run in Standalone Mock Mode
try:
    import rclpy
    from rclpy.node import Node
    from multi_cam_streamer.srv import StartCameraStream, StopCameraStream, GetCameraStreamStatus
    HAS_ROS = True
except ImportError:
    rclpy = None
    Node = object
    StartCameraStream = None
    StopCameraStream = None
    GetCameraStreamStatus = None
    HAS_ROS = False
    print("WARNING: ROS2 or interfaces not found. Running in Standalone Mock Mode.")

# ==============================================================================
# CONFIGURATION PARAMETERS
# ==============================================================================
CONFIG = {
    # IP address of the machine running the GUI (where to send UDP video).
    # If running locally, use "127.0.0.1". 
    "CLIENT_IP": "127.0.0.1",

    # Starting port for video streams.
    "BASE_PORT": 5000,

    # Simulated cameras available on this "rover".
    "CAMERAS": {
        "cam_front": {"resolution": "1280x720", "fps": 30},
        "cam_down":  {"resolution": "640x480",  "fps": 30},
    },

    # FFmpeg executable (change if not in PATH)
    "FFMPEG_BIN": "ffmpeg",

    # Automatically start streams on launch (Required for Standalone Mode)
    "AUTO_START": True
}
# ==============================================================================

class SimpleLogger:
    def info(self, msg): print(f"[INFO] {msg}")
    def error(self, msg): print(f"[ERROR] {msg}")

class MockRoverNode(Node):
    def __init__(self):
        self._streams = {}
        self._ports = {}
        self._lock = threading.Lock()

        if HAS_ROS:
            super().__init__('mock_camera_manager')
            self.logger = self.get_logger()
            
            # Create Services
            self._srv_start = self.create_service(
                StartCameraStream, 
                'camera_manager/start_stream', 
                self._handle_start
            )
            self._srv_stop = self.create_service(
                StopCameraStream, 
                'camera_manager/stop_stream', 
                self._handle_stop
            )
            self._srv_status = self.create_service(
                GetCameraStreamStatus, 
                'camera_manager/get_status', 
                self._handle_status
            )
        else:
            self.logger = SimpleLogger()

        self.logger.info(f"Ready. Target Client IP: {CONFIG['CLIENT_IP']}")

        if CONFIG.get("AUTO_START"):
            self.logger.info("Auto-starting all streams...")
            for cid in CONFIG["CAMERAS"]:
                self._start_stream(cid)

    def _start_stream(self, cid):
        """Internal helper to start a stream."""
        with self._lock:
            if cid in self._streams:
                if self._streams[cid].poll() is None:
                    return # Already running
                else:
                    del self._streams[cid]

            port = self._assign_port(cid)
            self._ports[cid] = port
            try:
                cmd = self._build_ffmpeg_cmd(cid, port)
                self.logger.info(f"Launching: {' '.join(cmd)}")
                proc = subprocess.Popen(
                    cmd, 
                    stdout=subprocess.DEVNULL, 
                    stderr=subprocess.DEVNULL
                )
                self._streams[cid] = proc
            except Exception as e:
                self.logger.error(f"Failed to launch ffmpeg for {cid}: {e}")

    def _handle_start(self, request, response):
        cid = request.camera_id
        self.logger.info(f"Request start stream: {cid}")
        self._start_stream(cid)
        with self._lock:
             if cid in self._streams and self._streams[cid].poll() is None:
                 response.success = True
                 response.stream_url = self._get_url(cid)
                 response.message = "Stream started"
             else:
                 response.success = False
                 response.message = "Failed to start"
        return response

    def _handle_stop(self, request, response):
        cid = request.camera_id
        self.logger.info(f"Request stop stream: {cid}")
        with self._lock:
            if cid in self._streams:
                proc = self._streams[cid]
                proc.terminate()
                try:
                    proc.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
                del self._streams[cid]
        response.success = True
        response.message = "Stopped"
        return response

    def _handle_status(self, request, response):
        cid = request.camera_id
        response.known = cid in CONFIG["CAMERAS"]
        with self._lock:
            if cid in self._streams and self._streams[cid].poll() is None:
                response.streaming = True
                response.stream_url = self._get_url(cid)
            else:
                response.streaming = False
                response.stream_url = ""
        response.last_error = ""
        return response

    def _assign_port(self, cid):
        keys = sorted(list(CONFIG["CAMERAS"].keys()))
        try:
            idx = keys.index(cid)
        except ValueError:
            idx = 0
        return CONFIG["BASE_PORT"] + idx

    def _get_url(self, cid):
        port = self._ports.get(cid, self._assign_port(cid))
        # Use udp://@:port for listening binding
        return f"udp://@:{port}"

    def _build_ffmpeg_cmd(self, cid, port):
        """Generate FFmpeg command for synthetic video."""
        props = CONFIG["CAMERAS"][cid]
        res = props["resolution"]
        fps = props["fps"]
        
        return [
            CONFIG["FFMPEG_BIN"],
            "-re",
            "-f", "lavfi",  # Synthetic input
            "-i", f"testsrc=size={res}:rate={fps}",
            "-c:v", "libx264",
            "-preset", "ultrafast",
            "-tune", "zerolatency",
            "-pix_fmt", "yuv420p",
            "-f", "mpegts",
            f"udp://{CONFIG['CLIENT_IP']}:{port}?pkt_size=1316"
        ]

    def stop_all(self):
        with self._lock:
            for cid, proc in self._streams.items():
                if proc.poll() is None:
                    proc.terminate()

def main(args=None):
    if HAS_ROS:
        rclpy.init(args=args)
        node = MockRoverNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.stop_all()
            node.destroy_node()
            rclpy.shutdown()
    else:
        node = MockRoverNode()
        print("Running in Standalone Loop (Ctrl+C to stop)")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            node.stop_all()

if __name__ == '__main__':
    main()
