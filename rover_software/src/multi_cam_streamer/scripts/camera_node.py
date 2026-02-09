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
    from rclpy.executors import MultiThreadedExecutor
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
    # Import custom service definitions
    from multi_cam_streamer.srv import StartCameraStream, StopCameraStream, GetCameraStreamStatus
    import cv2
    import numpy as np
    HAS_ROS = True
    print("HAS_ROS = True")
except ImportError as e:
    HAS_ROS = False 
    print("HAS_ROS = False (or other dependency problems)")
    ROS_IMPORT_ERROR = e

# ==============================================================================
# CONFIGURATION PARAMETERS
# ==============================================================================
CONFIG = {
    # IP address of the machine running the GUI (where to send UDP video).
    # CHANGE THIS to the actual IP of your laptop when running on the rover.
    "CLIENT_IP": "192.168.53.124",

    # FFmpeg executable
    "FFMPEG_BIN": "ffmpeg",

    # Camera Configuration
    # Key: Camera Name (used for logging)
    # Value: Dict with:
    #   - device: Path to video device 
    #   - port: UDP port to stream to
    #   - resolution: WxH
    #   - framerate: FPS
    # NOTE: This should match ROVER_CAMERAS in gui/config.py
    "CAMERAS": {
        "Left_USB": {
            "enabled": True,
            "device": "/dev/video2",  # Swapped for Debug
            "port": 5000,
            "resolution": "640x480",
            "framerate": 30,
            "topic": "/rover/left_usb_camera"
        },
        "Right_USB": {
            "enabled": True,
            "device": "/dev/video0",  # Swapped for Debug
            "port": 5001,
            "resolution": "640x480",
            "framerate": 30,
            "topic": "/rover/right_usb_camera"
        },
        "Dual_Cam_1": {
            "enabled": False,
            "device": "-1",  # Webcam
            "port": 5002,
            "resolution": "640x480",
            "framerate": 30,
            "topic": "/rover/dual_camera_1"
        },
        "Dual_Cam_2": {
            "enabled": False,
            "device": "-1",
            "port": 5003,
            "resolution": "640x480",
            "framerate": 30,
            "topic": "/rover/dual_camera_2"
        },
        "Insta360": {
            "enabled": False,
            "device": "-1", # Mock
            "port": 8554,
            "resolution": "1920x1080",
            "framerate": 30,
            "topic": "/rover/insta360"
        }
    },

    # If True, falls back to 'testsrc' if device not found.
    "ALLOW_MOCK_FALLBACK": False
}

# ==============================================================================

class CameraStreamer:
    def __init__(self, name, config, client_ip):
        self.name = name
        self.config = config
        self.device = str(config["device"])
        self.port = config["port"]
        self.resolution = config["resolution"]
        self.framerate = config["framerate"]
        self.client_ip = client_ip
        self.process = None
        self.enabled = config.get("enabled", True)
        # Allow overriding input format (default mjpeg for USB bandwidth efficiency)
        self.input_format = config.get("input_format", "mjpeg") 

    def start(self):
        if self.is_running():
            print(f"[{self.name}] Stream already running.")
            return

        cmd = self._build_command()
        print(f"[{self.name}] Starting Stream: {' '.join(cmd)}")
        
        try:
            # STABILITY FIX: Do not use PIPE for long-running processes without a reader thread.
            # It will fill the OS buffer and deadlock the process. 
            # We use a temporary log file for stderr to capture startup errors safely.
            self.log_file = open(f"/tmp/ffmpeg_{self.name}.log", "w+")
            
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,  # Discard video binary output
                stderr=self.log_file        # Capture logs to file instead of PIPE
            )

            # Wait briefly to detect immediate startup failures (device busy, permission denied)
            try:
                exit_code = self.process.wait(timeout=0.5)
                # If we get here, the process exited immediately (Error)
                self.log_file.seek(0)
                err_msg = self.log_file.read()
                print(f"[{self.name}] FFmpeg failed to start (Code {exit_code}):\n{err_msg}")
                self.stop()
                raise RuntimeError(f"FFmpeg exited immediately with code {exit_code}")
            except subprocess.TimeoutExpired:
                # Timeout means process is still running, which is SUCCESS for a stream.
                print(f"[{self.name}] Stream started successfully (PID: {self.process.pid})")

        except FileNotFoundError:
            print(f"[{self.name}] Error: FFmpeg binary not found.")
            self.stop()
            raise
        except Exception as e:
            print(f"[{self.name}] Error starting stream: {e}")
            self.stop()
            raise

    def stop(self):
        """Gracefully stops the FFmpeg process."""
        if self.process:
            print(f"[{self.name}] Stopping stream...")
            
            # 1. Try graceful termination (SIGTERM)
            self.process.terminate()
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                # 2. Force kill (SIGKILL) if stuck
                print(f"[{self.name}] Force killing stream...")
                self.process.kill()
                try:
                    self.process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    print(f"[{self.name}] WARNING: Process could not be killed.")
            
            self.process = None

        # Clean up log file handle
        if hasattr(self, 'log_file') and self.log_file:
            try:
                self.log_file.close()
            except Exception:
                pass
            self.log_file = None

    def is_running(self):
        """Checks if the process is currently active."""
        if self.process is None:
            return False
        if self.process.poll() is not None:
            self.process = None # Process finished/died
            return False
        return True

    def _build_command(self):
        system_os = platform.system()
        cmd = [self.config.get("FFMPEG_BIN", "ffmpeg"), "-y"]
        
        # --- MACOS (Darwin) CONFIGURATION ---
        if system_os == "Darwin":
            # Heuristic: Try to convert linux-style paths to indices, or use raw index
            device_index = self.device
            
            # Handle typical linux paths if config is shared
            if "video" in self.device:
                # Extract digits: /dev/video2 -> 2
                digits = ''.join(filter(str.isdigit, self.device))
                if digits:
                    # simplistic mapping: often video0 on linux is index 0 on mac
                    device_index = digits 
                else:
                    # Fallback for "video1" -> "0" mapping seen in your original code
                    if "video1" in self.device: device_index = "0"
                    elif "video3" in self.device: device_index = "1"
            
            print(f"[{self.name}] MacOS: Mapping '{self.device}' -> AVFoundation Index '{device_index}'")
            
            cmd += [
                "-f", "avfoundation",
                "-framerate", str(self.framerate),
                "-video_size", self.resolution,
                "-i", f"{device_index}:none" # 'none' prevents looking for audio device
            ]

        # --- LINUX CONFIGURATION ---
        elif system_os == "Linux":
            # Sanity check for device existence
            if not os.path.exists(self.device) and not self.device.startswith("/dev/"):
                 # Relaxed check: Only raise if it looks like a file path but is missing
                 # This allows passing network streams or unusual device names if needed
                 raise RuntimeError(f"Device {self.device} not found.")

            cmd += [
                "-f", "v4l2",
                "-framerate", str(self.framerate),
                "-video_size", self.resolution,
                "-input_format", self.input_format, # Use 'mjpeg' or 'yuyv422'
                "-i", self.device
            ]
        
        # --- GENERIC FALLBACK ---
        else:
            print(f"[{self.name}] Warning: Unknown OS '{system_os}'. Attempting generic inputs.")
            cmd += ["-i", self.device]

        # --- OUTPUT CONFIGURATION (Low Latency UDP) ---
        cmd += [
            "-c:v", "libx264",
            "-preset", "ultrafast",   # Critical for low CPU/latency
            "-tune", "zerolatency",   # Critical for streaming
            "-pix_fmt", "yuv420p",    # Ensure compatibility with all players
            "-x264-params", "repeat-headers=1",
            # "-g", str(self.framerate),# GOP size = 1 sec recovery time
            "-g", str(int(self.framerate / 2)), # Shorter GOP for lower latency (may increase CPU and bitrate)
            "-f", "mpegts",           # Robust container for UDP
            f"udp://{self.client_ip}:{self.port}?pkt_size=1316"
        ]

        return cmd

    def __del__(self):
        """Destructor to ensure processes don't become zombies."""
        self.stop()

class StreamManager:
    def __init__(self):
        self.streamers = {}
        for name, cfg in CONFIG["CAMERAS"].items():
            self.streamers[name] = CameraStreamer(name, cfg, CONFIG["CLIENT_IP"])

    def start_all(self):
        print("Initializing all enabled streams...")
        for s in self.streamers.values():
            if s.enabled:
                try:
                    s.start()
                except Exception as e:
                    print(f"[{s.name}] Failed to auto-start: {e}")
            else:
                print(f"[{s.name}] Skipped (Disabled)")

    def stop_all(self):
        for s in self.streamers.values():
            s.stop()

    def start_stream(self, camera_id, resolution=None, framerate=None, bitrate_kbps=None, codec=None):
        if camera_id not in self.streamers:
            return False, f"Unknown camera: {camera_id}", ""
        
        s = self.streamers[camera_id]
        
        if s.process:
            print(f"[{camera_id}] Restarting stream with new settings...")
            s.stop()
            # Brief pause to ensure device handle is released by OS
            time.sleep(2.0)
        
        if resolution: s.resolution = resolution
        if framerate: s.framerate = int(framerate)
        
        try:
            s.start()
            url = f"udp://{CONFIG['CLIENT_IP']}:{s.port}"
            return True, f"Started on device {s.device}", url
        except Exception as e:
            return False, str(e), ""

    def stop_stream(self, camera_id):
        if camera_id in self.streamers:
            self.streamers[camera_id].stop()
            return True, "Stopped"
        return False, "Unknown camera"

# Only define the ROS service node class if ROS is available
if HAS_ROS:
    class CameraServiceNode(Node):
        def __init__(self, stream_manager):
            super().__init__('camera_manager_service')
            self.manager = stream_manager
            
            self.srv_start = self.create_service(StartCameraStream, 'camera_manager/start_stream', self.handle_start)
            self.srv_stop = self.create_service(StopCameraStream, 'camera_manager/stop_stream', self.handle_stop)
            self.srv_status = self.create_service(GetCameraStreamStatus, 'camera_manager/get_status', self.handle_status)
            
            self.get_logger().info("Camera Manager Services Ready")

        def handle_start(self, request, response):
            try:
                # Handle framerate if available in request (it should be if .srv is updated)
                fps = request.framerate if hasattr(request, 'framerate') else 30

                success, msg, url = self.manager.start_stream(
                    request.camera_id, 
                    resolution=request.resolution, 
                    framerate=fps
                )
                response.success = success
                response.message = msg
                response.stream_url = url
            except Exception as e:
                response.success = False
                response.message = str(e)
            return response

        def handle_stop(self, request, response):
            success, msg = self.manager.stop_stream(request.camera_id)
            response.success = success
            response.message = msg
            return response

        def handle_status(self, request, response):
            response.known = request.camera_id in self.manager.streamers
            if response.known:
                s = self.manager.streamers[request.camera_id]
                response.streaming = s.process is not None
                if response.streaming:
                    response.stream_url = f"udp://{CONFIG['CLIENT_IP']}:{s.port}"
            return response

def main(args=None):
    if HAS_ROS:
        print("Running in ROS2 Mode (Service Enabled)")
        print("Press 'q' + ENTER to quit gracefully.")
        rclpy.init(args=args)
        
        executor = MultiThreadedExecutor()
        
        manager = StreamManager()
        # Auto-start streams on boot?
        manager.start_all()
        
        service_node = CameraServiceNode(manager)
        executor.add_node(service_node)
        
        # Run ROS spin in a separate thread
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        
        try:
            while rclpy.ok():
                # Use select on stdin to be non-blocking or just try input()
                if sys.stdin.isatty():
                    try:
                        cmd = input()
                        if cmd.strip().lower() == 'q':
                            break
                    except EOFError:
                        break
                else:
                    # If no TTY, just wait normally
                    time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            print("\nShutting down...")
            manager.stop_all()
            # Stop the executor and node
            try:
                service_node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass
            spin_thread.join(timeout=2.0)
    # else:
    #     # Standalone Mode
    #     print("Running in Standalone Mode (No ROS2 detected)")
    #     print("Press 'q' + ENTER to quit gracefully.")
    #     if 'ROS_IMPORT_ERROR' in globals():
    #         print(f"Debug: ROS2 Import Failed: {ROS_IMPORT_ERROR}")
    #     manager = StreamManager()
    #     try:
    #         manager.start_all()
    #         while True:
    #             if sys.stdin.isatty():
    #                 try:
    #                      cmd = input()
    #                      if cmd.strip().lower() == 'q':
    #                          break
    #                 except EOFError:
    #                      break
    #             else:
    #                 time.sleep(1)
    #     except KeyboardInterrupt:
    #         pass
    #     finally:
    #         print("\nShutting down...")
    #         manager.stop_all()

if __name__ == "__main__":
    main()