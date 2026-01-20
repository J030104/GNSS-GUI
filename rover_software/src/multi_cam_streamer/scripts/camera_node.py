#!/usr/bin/env python3
"""
Rover Camera Streaming Node.

This script manages FFmpeg processes to stream video from local USB cameras
to a remote GUI via UDP/MPEG-TS.

It can be run as a ROS2 node (if 'rclpy' is available) or as a standalone script.
It detects if proper video devices are present; if not, it falls back to
generating synthetic test patterns (unless disable_mock is set).

CONFIGURATION:
- Adjust `CONFIG` dictionary below to match your hardware setup.
- `CLIENT_IP` should match the IP of the computer running the GUI.
"""

import subprocess
import threading
import time
import os
import signal
import sys
import glob
import platform

# Try importing ROS2 libraries
try:
    import rclpy
    from rclpy.node import Node
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

# ==============================================================================
# CONFIGURATION PARAMETERS
# ==============================================================================
CONFIG = {
    # IP address of the machine running the GUI (where to send UDP video).
    # CHANGE THIS to the actual IP of your laptop when running on the rover.
    "CLIENT_IP": "127.0.0.1",

    # FFmpeg executable
    "FFMPEG_BIN": "ffmpeg",

    # Camera Configuration
    # Key: Camera Name (used for logging)
    # Value: Dict with:
    #   - device: Path to video device 
    #   - port: UDP port to stream to
    #   - resolution: WxH
    #   - framerate: FPS
    "CAMERAS": {
        "Left_USB": {
            # "device": "0", # Arducam (Updated from [2] to [1])
            "device": "/dev/video0",
            "port": 5000,
            "resolution": "640x480",
            "framerate": 30
        },
        "Right_USB": {
            # "device": "2", # USB Camera
            "device": "/dev/video2",
            "port": 5001,
            "resolution": "640x480",
            "framerate": 30
        },
        # Placeholders for future expansion
        "Dual_Cam_1": {
            "device": "/dev/video4", # Placeholder
            "port": 5002,
            "resolution": "640x480",
            "framerate": 30
        },
        "Dual_Cam_2": {
            "device": "/dev/video6", # Placeholder
            "port": 5003,
            "resolution": "640x480",
            "framerate": 30
        },
        "Insta360": {
            "device": "/dev/video8", # Placeholder
            "port": 8554,
            "resolution": "1920x1080",
            "framerate": 30
        }
    },

    # If True, falls back to 'testsrc' if device not found.
    "ALLOW_MOCK_FALLBACK": True
}
# ==============================================================================

class CameraStreamer:
    def __init__(self, name, config, client_ip):
        self.name = name
        self.device = config["device"]
        self.port = config["port"]
        self.resolution = config["resolution"]
        self.framerate = config["framerate"]
        self.client_ip = client_ip
        self.process = None

    def start(self):
        if self.process is not None:
            return

        cmd = self._build_command()
        # TODO: Send this to the GUI
        print(f"[{self.name}] Starting Stream: {' '.join(cmd)}")
        
        try:
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL # TODO: Set to subprocess.PIPE for debugging
            )
        except FileNotFoundError:
            print(f"[{self.name}] Error: FFmpeg not found.")
        except Exception as e:
            print(f"[{self.name}] Error starting stream: {e}")

    def stop(self):
        if self.process:
            print(f"[{self.name}] Stopping stream...")
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
            self.process = None

    def _build_command(self):
        system_os = platform.system()
        
        # Check if device exists logic
        use_mock = False
        
        # On Linux, we can check /dev/videoX existence.
        # On MacOS, hard to check without running ffmpeg, so we skip check or rely on fallback logic if we implemented a probe.
        if system_os == "Linux":
            if not os.path.exists(self.device):
                if CONFIG["ALLOW_MOCK_FALLBACK"]:
                    print(f"[{self.name}] Device {self.device} not found. Using synthetic source.")
                    use_mock = True
                else:
                    print(f"[{self.name}] Device {self.device} not found. Stream will fail.")
        elif system_os == "Darwin":
             # On MacOS, if configured with Linux paths, default to Mock validation
             # to ensure a visible stream for testing/showcase purposes.
             if CONFIG["ALLOW_MOCK_FALLBACK"] and self.device.startswith("/dev/"):
                 print(f"[{self.name}] Device {self.device} (Linux path) on MacOS. Using synthetic source for demo.")
                 use_mock = True
        
        cmd = [CONFIG["FFMPEG_BIN"], "-y"]

        # Input Options
        if use_mock:
            # Synthetic pattern
            # ffmpeg -re -f lavfi -i testsrc=size=640x480:rate=30 ...
            cmd += ["-re", "-f", "lavfi", "-i", f"testsrc=size={self.resolution}:rate={self.framerate}"]
        else:
            if system_os == "Darwin":
                # MacOS (AVFoundation)
                # Map typical linux paths to indices if needed for local testing
                device_index = "0"
                if "video1" in self.device: device_index = "0"
                elif "video3" in self.device: device_index = "1"
                elif self.device.isdigit(): device_index = self.device
                
                cmd += [
                    "-f", "avfoundation",
                    "-framerate", str(self.framerate),
                    "-video_size", self.resolution,
                    "-i", f"{device_index}:none" # Use 'none' for audio
                ]
            else:
                # Linux / Default (V4L2)
                cmd += [
                    "-f", "v4l2",
                    "-framerate", str(self.framerate),
                    "-video_size", self.resolution,
                    "-input_format", "mjpeg", # Try mjpeg for better usb bandwidth, or yuyv422
                    "-i", self.device
                ]

        # Encoding/Output Options for Low Latency
        # -preset ultrafast -tune zerolatency are key for low latency h264
        # We use mpegts container over UDP
        cmd += [
            "-c:v", "libx264",
            "-preset", "ultrafast",
            "-tune", "zerolatency",
            "-pix_fmt", "yuv420p", # Compatible with most players
            "-g", str(self.framerate), # Keyframe every 1 sec (or less for resilience)
            "-f", "mpegts",
            f"udp://{self.client_ip}:{self.port}?pkt_size=1316"
        ]

        return cmd

class StreamManager:
    def __init__(self):
        self.streamers = []
        for name, cfg in CONFIG["CAMERAS"].items():
            s = CameraStreamer(name, cfg, CONFIG["CLIENT_IP"])
            self.streamers.append(s)

    def start_all(self):
        for s in self.streamers:
            s.start()

    def stop_all(self):
        for s in self.streamers:
            s.stop()

# Wrapper for ROS2 Node compatibility
class CameraNode(Node if HAS_ROS else object):
    def __init__(self):
        if HAS_ROS:
            super().__init__('camera_manager')
            self.get_logger().info("Starting Camera Manager Node")
        
        self.manager = StreamManager()
        self.manager.start_all()

    def destroy_node(self):
        self.manager.stop_all()
        if HAS_ROS:
            super().destroy_node()

def main(args=None):
    if HAS_ROS:
        # ROS2 Mode
        print("Running in ROS2 Mode")
        rclpy.init(args=args)
        node = CameraNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        # Standalone Mode
        print("Running in Standalone Mode (No ROS2 detected)")
        manager = StreamManager()
        try:
            manager.start_all()
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nShutting down...")
            manager.stop_all()

if __name__ == "__main__":
    main()
