# GNSS-GUI

A PyQt5 control interface for the URC Rover featuring GNSS monitoring, multi-camera streaming, and subsystem control.

## Quick Start

1. **Install Dependencies**:
   ```bash
   brew install ffmpeg  # macOS
   pip install -r gui/requirements.txt
   ```

2. **Run GUI**:
   ```bash
   python3 -m gui.main
   ```

3. **(Optional) Run Camera Sim**:
   ```bash
   python3 rover_software/src/multi_cam_streamer/scripts/camera_node.py
   ```

## Configuration

Settings in `gui/config.py` control:
- **Rover IP**: Target address.
- **Camera Ports**: Video feed ports.
- **Names**: Camera labels.

## Structure

- **`gui/`**: Main application code.
- **`rover_software/`**: ROS 2 packages for the rover.
- **`tests/`**: Test scripts.
