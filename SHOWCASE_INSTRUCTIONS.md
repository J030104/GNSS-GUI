# Showcase Instructions

This document outlines how to run the GNSS-GUI and the simulated Rover Server for the showcase.

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
    (Ensure `opencv-python` is installed)

## 2. Project Structure

*   `gnss_gui/`: The main GUI application.
*   `rover_software/`: The code meant to run on the Rover (Jetson/RPi).
    *   Contains `multi_cam_streamer` ROS2 package.
    *   Contains `mock_rover_node.py` which simulates cameras without hardware.

## 3. Running the Showcase

You will need two terminal windows.

### Terminal 1: Application Server (Rover Simulation)

This emulates the rover sending video feeds.

**Connecting to a VM (Remote Rover):**
If you are running this on a VM (like VMware Fusion/Ubuntu) and want to see the video on your Host Mac:
1.  Find your Host Mac's IP address (e.g., run `ifconfig` on Mac, look for `en0` or `vmnet` IP, often `192.168.x.x` or `172.16.x.1`).
2.  Edit `rover_software/src/multi_cam_streamer/scripts/mock_rover_node.py` **ON THE VM**.
3.  Change `CLIENT_IP` to your Mac's IP address:
    ```python
    CONFIG = {
        "CLIENT_IP": "192.168.1.5",  # <--- YOUR MAC HOST IP HERE
        # ...
    }
    ```
4.  Run the script on the VM.

**Running Locally (Mac Only):**
1.  Navigate to the project root.
2.  Run the mock node (works without ROS installed):
    ```bash
    python3 rover_software/src/multi_cam_streamer/scripts/mock_rover_node.py
    ```
    *   **Note:** This will automatically start streaming a test pattern to `udp://127.0.0.1:5000` (Front Cam) and `:5001` (Down Cam).
    *   *Troubleshooting:* If you see "ImportError: No module named rclpy", the script will fail if strict ROS dependencies are checked. However, checking the code, it imports `rclpy`. 
    *   **IF YOU DO NOT HAVE ROS:** You may need to comment out `import rclpy` and `Node` inheritance in `mock_rover_node.py` and run it as a standalone script for the demo, OR ensure you have a basic ROS environment (e.g. `robostack` on Mac).
    *   *Alternative:* If you cannot run the python script due to missing ROS, you can manually run ffmpeg:
        ```bash
        ffmpeg -f lavfi -i testsrc=size=1280x720:rate=30 -preset ultrafast -tune zerolatency -f mpegts udp://127.0.0.1:5000?pkt_size=1316
        ```

### Terminal 2: The GNSS GUI

1.  Navigate to the project root.
2.  Run the GUI:
    ```bash
    python3 -m gnss_gui.main
    ```

## 4. Demonstrating Features

1.  **Local Camera**:
    *   In the "Comms" tab (right side Control Panel), select **Camera: Local**.
    *   Click **Start Stream**.
    *   Verify your webcam appears in the `Dual Camera 1` or primary viewer window.

2.  **Remote Camera (Rover)**:
    *   Ensure the Mock Rover Node (Terminal 1) is running.
    *   Select **Camera: Camera 1** or **Insta 360**.
    *   Click **Start Stream**.
    *   Verify the synthesized test pattern (colored bars/noise) appears in the Viewer.

## 5. Deploying to Rover

To deploy the `rover_software` to the real rover:
1.  Copy the `rover_software` folder to the rover's `src` workspace.
2.  Build with `colcon build`.
3.  Source setup: `. install/setup.bash`.
4.  Run the real nodes or the mock node as needed.
