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

# Configuration for Rover Software mirroring
# (Used if generating remote config)
REMOTE_CONFIG = {
    "CLIENT_IP": "127.0.0.1",  # Where to send video (The GUI's IP)
}
