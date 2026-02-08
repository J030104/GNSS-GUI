"""
Configuration for GNSS GUI.

This file contains connection details, camera definitions, and network settings.
"""

# Connection Settings
ROVER_IP = "192.168.53.136"


# ==============================================================================
# Camera Definitions (Centralized Source of Truth)
# ==============================================================================
# All other maps (CAMERA_NAMES, CAMERA_CONFIG) are derived from this.

ROVER_CAMERAS = {
    "LOCAL": {
        "name": "Local",
        "id": "LOCAL",
        "transport": "local", 
        "device_index": 1, # Explicitly MacBook Pro Camera
        "port": None,
        "default_res": "640x480",
        "default_fps": 30,
        "enabled": True
    },
    "USB_LEFT": {
        "name": "Left Side Camera",
        "id": "Left_USB",
        "transport": "udp",
        "topic": "/rover/left_usb_camera",
        "host": ROVER_IP,
        "device": "2",  # From camera_node.py
        "port": 5000,
        "default_res": "640x480",
        "default_fps": 30,
        "enabled": True
    },
    "USB_RIGHT": {
        "name": "Right Side Camera",
        "id": "Right_USB",
        "transport": "udp",
        "topic": "/rover/right_usb_camera",
        "host": ROVER_IP,
        "device": "0",  # From camera_node.py
        "port": 5001,
        "default_res": "640x480",
        "default_fps": 30,
        "enabled": True
    },
    "DUAL_1": {
        "name": "Dual Camera 1",
        "id": "Dual_Cam_1",
        "transport": "disabled",
        "topic": "/rover/dual_camera_1",
        "port": 5002,
        "default_res": "640x480",
        "default_fps": 30,
        "enabled": False
    },
    "DUAL_2": {
        "name": "Dual Camera 2",
        "id": "Dual_Cam_2",
        "transport": "disabled",
        "topic": "/rover/dual_camera_2",
        "port": 5003,
        "default_res": "640x480",
        "default_fps": 30,
        "enabled": False
    },
    "INSTA360": {
        "name": "Insta360",
        "id": "Insta360",
        "transport": "disabled", # Placeholder
        "topic": "/rover/insta360",
        "port": 8554,
        "default_res": "1920x1080",
        "default_fps": 30,
        "enabled": False
    }
}

# ==============================================================================
# Helpers & Derived Configurations (Do Not Edit Manually)
# ==============================================================================

# 1. Names for GUI Dropdowns
CAMERA_NAMES = {k: v["name"] for k, v in ROVER_CAMERAS.items()}

# 2. Port Mappings
CAMERA_PORTS = {k: v["port"] for k, v in ROVER_CAMERAS.items() if v.get("port")}

# 3. Transport Config (for VideoStreamer)
CAMERA_CONFIG = ROVER_CAMERAS # Alias, as the structure is now compatible

def get_rover_id_by_name(display_name: str) -> str:
    """Case-insensitive lookup for Rover ID from Display Name."""
    if not display_name: return None
    target = display_name.strip().lower()
    
    # Check names
    for key, cfg in ROVER_CAMERAS.items():
        if cfg["name"].strip().lower() == target:
            return cfg["id"]
            
    # Check IDs (fallback) or Keys
    for key, cfg in ROVER_CAMERAS.items():
         if cfg["id"].strip().lower() == target:
             return cfg["id"]
         if key.strip().lower() == target:
             return cfg["id"]
    return None

def get_camera_config_by_name(display_name: str) -> dict:
    """Get full config dictionary by display name."""
    rid = get_rover_id_by_name(display_name)
    if rid:
        for k, v in ROVER_CAMERAS.items():
            if v["id"] == rid:
                return v
    return None

