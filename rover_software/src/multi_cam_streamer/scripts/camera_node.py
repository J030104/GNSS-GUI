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
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
    import cv2
    import numpy as np
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
            "device": "1", # USB Camera
            # "device": "/dev/video2",
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

class VideoPublisherNode(Node):
    """
    ROS2 Node that captures video from a device and publishes it
    to a camera-specific topic using sensor_msgs/Image.
    
    Topic naming: /rover/<camera_name_snake_case>
    Example: "Right_USB" -> /rover/right_usb
    """
    def __init__(self, camera_name, device_path, resolution="640x480", framerate=30, topic=None):
        node_name = 'video_publisher_' + camera_name.lower().replace(" ", "_")
        super().__init__(node_name)
        self.camera_name = camera_name
        self.device_path = device_path
        
        # Generate topic name from camera name if not provided
        if topic is None:
            topic = '/rover/' + camera_name.lower().replace(" ", "_").replace("-", "_")
        self.topic_name = topic
        
        # QoS Profile: Best Effort, Volatile for low latency sensor data
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # Publisher
        self.publisher_ = self.create_publisher(Image, self.topic_name, qos_profile)
        
        # Timer
        self.framerate = framerate
        self.timer_period = 1.0 / self.framerate
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # OpenCV Setup
        self.bridge = CvBridge()
        
        # Handle device path for MacOS/OpenCV compatibility
        # On Linux/ROS, /dev/video0 works. On MacOS, we usually need an integer index.
        capture_source = self.device_path
        if platform.system() == "Darwin":
             if str(self.device_path).startswith("/dev/video"):
                 try:
                     # Extract number from /dev/videoX
                     index_str = str(self.device_path).replace("/dev/video", "")
                     if index_str.isdigit():
                         capture_source = int(index_str)
                 except:
                     capture_source = 0 # Fallback
             elif str(self.device_path).isdigit():
                  capture_source = int(self.device_path)
             
        self.cap = cv2.VideoCapture(capture_source)
        
        # Resolution parsing
        try:
            w, h = map(int, resolution.split('x'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        except ValueError:
            pass
            
        self.cap.set(cv2.CAP_PROP_FPS, self.framerate)
        
        if not self.cap.isOpened():
             self.get_logger().error(f"Could not open video device: {self.device_path}")
        else:
             self.get_logger().info(f"Publishing video from {self.device_path} to {self.topic_name}")

    def timer_callback(self):
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                # Convert to ROS MSG
                # Determine encoding. OpenCV uses BGR by default.
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.publisher_.publish(msg)
            else:
                # Try to reconnect or log?
                pass
                
    def destroy_node(self):
        if self.cap:
             self.cap.release()
        super().destroy_node()

# Wrapper for ROS2 Node compatibility
class CameraNode(Node if HAS_ROS else object):
    def __init__(self):
        if HAS_ROS:
            super().__init__('camera_manager')
            self.get_logger().info("Starting Camera Manager Node")
            
            # We will use the first enabled camera's config for the publisher for now
            # as per the instruction to publish to /rover/camera_feed (singular)
            # Find a suitable camera
            target_cam_config = None
            target_cam_name = None
            
            for name, cfg in CONFIG["CAMERAS"].items():
                # Simple logic: pick the first one or specific
                target_cam_config = cfg
                target_cam_name = name
                break
            
            if target_cam_config:
                # We spawn the publisher node logic here. 
                # Ideally, we should spin multiple nodes if we want multiple cams, 
                # but CameraNode is the main node. 
                # Let's attach the publishing logic to this node or create a helper.
                # Since we need to spin, we can use MultiThreadedExecutor or just instantiate the helper class separate from this manager
                # However, to keep it simple and "within existing client logic":
                
                # I will create the VideoPublisherNode separately in the main function or 
                # integrate it here. To avoid MultiThreadedExecutor complexity in this simple script:
                # I'll just Make VideoPublisherNode instance logic part of the loop?
                # No, cleaner to have separate object.
                pass

        self.manager = StreamManager()
        # If we are in ROS mode, we might WANT to use the ROS publisher 
        # INSTEAD of the ffmpeg streamer for the active camera.
        
        # But StreamManager runs ffmpeg.
        # The user says "Refactor ... to replace ... with ROS 2".
        # So I should disable StreamManager if I'm using ROS publisher?
        # Or modify StreamManager to support ROS publisher.

    def destroy_node(self):
        self.manager.stop_all()
        if HAS_ROS:
            super().destroy_node()

def main(args=None):
    if HAS_ROS:
        # ROS2 Mode
        print("Running in ROS2 Mode")
        rclpy.init(args=args)
        
        # Priority: Right_USB
        camera_name = "Right_USB"
        camera_config = CONFIG["CAMERAS"].get(camera_name)
        
        if not camera_config:
            # Fallback
            camera_name = "Left_USB"
            camera_config = CONFIG["CAMERAS"].get(camera_name)
            
        if not camera_config:
             # Just pick the first one
             camera_name = list(CONFIG["CAMERAS"].keys())[0]
             camera_config = CONFIG["CAMERAS"][camera_name]
        
        # Topic naming convention matching GUI config
        # Right_USB -> /rover/right_usb_camera
        # Left_USB -> /rover/left_usb_camera
        # TODO: Make robust
        topic = '/rover/' + camera_name.lower().replace("_", "_") + "_camera"
        print(f"Publishing to topic: {topic}")
             
        # Create the publisher node
        publisher_node = VideoPublisherNode(
            camera_name=camera_name, 
            device_path=camera_config["device"],
            resolution=camera_config["resolution"],
            framerate=camera_config["framerate"],
            topic=topic
        )
        
        try:
            rclpy.spin(publisher_node)
        except KeyboardInterrupt:
            pass
        finally:
            publisher_node.destroy_node()
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