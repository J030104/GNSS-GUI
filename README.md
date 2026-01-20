# GNSS-GUI Project

This project provides a modular graphical user interface (GUI) for the University Rover Challenge (URC). It is designed to control and monitor the rover, specifically focusing on the **GNSS & Communication** subsystem, but extensible to others like Autonomous Navigation, Robotic Arm, and Science Mission.

The system features a **6-Camera Layout** capable of displaying multiple video streams simultaneously, including local USB cameras, network streams from the rover, and a local webcam feed.

## Key Features

*   **Multi-Camera Streaming**: Supports up to 6 simultaneous video feeds key functionality.
    *   **USB Cameras**: Direct integration with local USB webcams (Left/Right) via UDP.
    *   **Local Feed**: Automatically detects and streams the host machine's webcam (e.g., MacBook Pro Camera).
    *   **Network Streams**: Integration with remote streams (Dual Cam 1/2, Insta360) via standard protocols (UDP/RTSP).
*   **Modular GUI**: Built with PyQt5, organized into tabs for different subsystems.
*   **Configurable Layouts**: Dynamic 6-camera grid layout (2x3) plus alternative viewing modes.
*   **Centralized Configuration**: `gnss_gui/config.py` manages all camera names, ports, and IP addresses.
*   **Rover Simulation**: Includes `camera_node.py` to simulate rover camera feeds using local hardware or synthetic patterns.

## Project Structure

```
.
├── gnss_gui/                   # Main GUI Application Package
│   ├── config.py               # Central configuration (Ports, IPs, Names)
│   ├── main.py                 # Application entry point
│   ├── components/             # Reusable GUI widgets (Video Viewer, Control Panel)
│   ├── subsystems/             # Subsystem specific tabs (GNSS Comm, etc.)
│   └── utilities/              # Helper scripts (Video Streamer, Connection Manager)
├── rover_software/             # Code running on the Rover (Sender)
│   └── src/multi_cam_streamer/ # ROS2/Python camera streaming package
│       └── scripts/
│           └── camera_node.py  # Main camera streaming node
├── tests/                      # Unit tests and temporary test scripts
├── ROVER_TEST_PROMPT.md        # Instructions for generating rover-side telemetry tests
├── SHOWCASE_INSTRUCTIONS.md    # Guide for running the showcase demonstration
└── TODO.md                     # Project roadmap and active tasks
```

## Getting Started

### Prerequisites

*   **Python 3.10+**
*   **FFmpeg** must be installed and accessible in your system PATH.
    *   macOS: `brew install ffmpeg`
    *   Linux: `sudo apt install ffmpeg`
*   **Python Dependencies**:
    ```bash
    pip install -r gnss_gui/requirements.txt
    ```

### Running the GUI

To start the control interface:
```bash
python3 -m gnss_gui.main
```

### Running the Camera Source (Simulation/Rover)

To simulate the rover's video feeds (using your local USB cameras):
```bash
cd rover_software/src/multi_cam_streamer/scripts
python3 camera_node.py
```
This will start streaming video from your attached devices to the ports defined in `gnss_gui/config.py` (Default: Left=5000, Right=5001).

## Configuration

Edit `gnss_gui/config.py` to adjust:
*   **ROVER_IP**: IP address of the rover (default `127.0.0.1` for local testing).
*   **CAMERA_PORTS**: UDP/RTSP ports for each video feed.
*   **CAMERA_NAMES**: Display names used in the GUI dropdowns.

## Development

*   **GUI**: The main logic resides in `gnss_gui/subsystems/gnss_comm.py`.
*   **Layouts**: Video grid layouts are defined in `gnss_gui/components/video_layout_tabs.py` and instantiated in `gnss_comm.py`.
*   **Streaming**: `gnss_gui/utilities/video_streamer.py` handles the FFmpeg process management and Local camera access.
