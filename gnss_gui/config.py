"""
Configuration for GNSS GUI.

This file contains connection details, camera definitions, and network settings.
"""

# Connection Settings
ROVER_IP = "127.0.0.1"
SSH_USER = "egg"  # Placeholder or development user
SSH_KEY_PATH = None  # None for default agent or password search

# Camera Definitions
# These names should appear in the Control Panel dropdown.
CAMERA_NAMES = {
    "LOCAL": "Local",
    "DUAL_1": "Dual Camera 1",
    "DUAL_2": "Dual Camera 2",
    "USB_LEFT": "Left USB Camera",
    "USB_RIGHT": "Right USB Camera",
    "INSTA360": "Insta360"
}

# Network Stream Configuration
# Port assignments for direct UDP/RTSP streams
CAMERA_PORTS = {
    "USB_LEFT": 5000,
    "USB_RIGHT": 5001,
    "DUAL_1": 5002,     # Placeholder
    "DUAL_2": 5003,     # Placeholder
    "INSTA360": 8554    # Standard RTSP/SRT port or custom UDP
}

# Camera Registration Configuration
# ===================================
# Defines how each camera source is configured in the GUI.
#
# In the future, ALL cameras will stream via ROS2 publishers from the rover.
# The camera_node.py publishes to /rover/camera_feed (or per-camera topics).
#
# Transport Types:
#   "ros2"     - Subscribe to a ROS2 Image topic (preferred for rover cameras)
#   "udp"      - Legacy UDP/MPEG-TS stream via OpenCV
#   "rtsp"     - RTSP stream via OpenCV
#   "local"    - Local webcam (for development/testing only)
#   "disabled" - Camera not available / placeholder
#
CAMERA_CONFIG = {
    # Local webcam - for development testing on your Mac
    "LOCAL": {
        "transport": "local",
        "device_index": "auto",  # "auto" detects MacBook camera, or set integer
    },
    
    # Left USB Camera on Rover
    "USB_LEFT": {
        # "transport": "udp",      # TODO: Change to "ros2" when rover publishes this
        # "host": "0.0.0.0",
        # "port": 5000,
        "transport": "ros2",
        "topic": "/rover/left_usb_camera",
    },
    
    # Right USB Camera on Rover - Currently active ROS2 stream
    "USB_RIGHT": {
        "transport": "ros2",
        "topic": "/rover/right_usb_camera",
    },
    
    # Dual Camera 1 - Placeholder
    "DUAL_1": {
        "transport": "disabled",
        # "transport": "ros2",
        # "topic": "/rover/dual_camera_1",
    },
    
    # Dual Camera 2 - Placeholder
    "DUAL_2": {
        "transport": "disabled",
        # "transport": "ros2",
        # "topic": "/rover/dual_camera_2",
    },
    
    # Insta360 Camera - Placeholder
    "INSTA360": {
        "transport": "disabled",
        # "transport": "ros2",
        # "topic": "/rover/insta360",
    },
}

# Configuration for Rover Software mirroring
# (Used if generating remote config)
REMOTE_CONFIG = {
    "CLIENT_IP": "127.0.0.1",  # Where to send video (The GUI's IP)
}
