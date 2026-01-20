# Showcase Instructions

This document outlines how to run the GNSS-GUI and the Camera node for the showcase demonstration.

## 1. Prerequisites

Ensure you have the following installed on your demonstration machine (macOS):

*   **Python 3.10+**
*   **FFmpeg** (required for video streaming):
    ```bash
    brew install ffmpeg
    ```
*   **Python Dependencies**:
    ```bash
    pip install -r gnss_gui/requirements.txt
    ```

## 2. Project Structure

*   `gnss_gui/`: The main GUI application.
*   `rover_software/`: The code meant to run on the Rover (Jetson/RPi).
    *   `scripts/camera_node.py`: Simulates or runs the actual camera streams.

## 3. Running the Showcase

You will need two terminal windows.

### Terminal 1: Camera Source (Rover Side)

This mimics the rover sending video feeds from its USB cameras.

1.  Navigate to the script directory:
    ```bash
    cd rover_software/src/multi_cam_streamer/scripts
    ```
2.  Run the camera node:
    ```bash
    python3 camera_node.py
    ```
    *   **Behavior**: This script detects connected USB cameras.
        *   **Left USB**: Index 1 (Arducam) -> Streamed to Port 5000.
        *   **Right USB**: Index 0 (USB Cam) -> Streamed to Port 5001.
        *   If a camera is missing, it may fallback to a synthetic test pattern if configured.

### Terminal 2: The GNSS GUI (Base Station)

1.  Navigate to the project root.
2.  Run the GUI:
    ```bash
    python3 -m gnss_gui.main
    ```

## 4. Demonstrating Features

The GUI starts in the **"6-Cam Layout"**.

1.  **Local Camera**:
    *   Select **Camera: Local** in the Control Panel.
    *   Click **Start Stream**.
    *   **Result**: Your MacBook Pro's built-in webcam stream appears in the top-middle "Local" box.

2.  **USB Cameras**:
    *   Select **Camera: Left USB Camera**.
    *   Click **Start Stream**.
    *   **Result**: The feed from the Arducam appears in the bottom-left box.
    *   Select **Camera: Right USB Camera**.
    *   Click **Start Stream**.
    *   **Result**: The feed from the Generic USB cam appears in the bottom-right box.

3.  **Placeholders**:
    *   Selecting "Dual Camera 1", "Dual Camera 2", or "Insta360" will log a placeholder message, as these streams are not yet physically connected in this demo setup.

## 5. Troubleshooting configuration

If camera indices change (e.g., unplugging/replugging):
*   **GUI**: `gnss_gui/config.py` defines the names and ports.
*   **Rover**: Edit `rover_software/src/multi_cam_streamer/scripts/camera_node.py` to adjust `device` indices (e.g., "0", "1", "2") to match your `ffmpeg -list_devices true -f avfoundation -i ""` output.
