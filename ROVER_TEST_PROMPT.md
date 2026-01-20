
## Rover Side Simulation (Camera Node)

You can run the camera node locally on your laptop to simulate the rover's video feeds using your own webcam(s).

### Requirements
- **Python 3**
- **FFmpeg** installed and in your PATH.
- `opencv-python` and `netifaces` installed.

### Setup & Run
1. Navigate to the script folder:
   ```bash
   cd rover_software/src/multi_cam_streamer/scripts
   ```
2. (Optional) Edit `camera_node.py` to hardcode specific device indices if the auto-detection is failing, or to change the destination IP if not running locally.
   - Look for `DESTINATION_IP = "127.0.0.1"`
3. Run the node:
   ```bash
   python3 camera_node.py
   ```
4. The node will:
   - Detect available video devices (using `ffmpeg -list_devices`).
   - Assign them to configured slots (Left Camera -> Port 5000, Right Camera -> Port 5001).
   - Start streaming via UDP to the GUI.

### Testing Telemetry
To test telemetry (GPS data, etc.), you will need to run the separate telemetry simulation scripts located in `rover_software/...` (to be implemented/documented).

## Rover Code Debugging & Build Fix Prompt

Use the following prompt in a separate session (with the rover codebase open) to fix the build issues in the `multi_cam_streamer` package.

### Prompt text

> I have a ROS 2 package named `multi_cam_streamer` that contains both **custom interfaces** (.msg/.srv) and **Python source code** (a module also named `multi_cam_streamer`).
> 
> The `CMakeLists.txt` calls both:
> 1. `rosidl_generate_interfaces(${PROJECT_NAME} ...)`
> 2. `ament_python_install_package(${PROJECT_NAME})`
> 
> This is causing a target name collision error during `colcon build`:
> `add_custom_target cannot create target "ament_cmake_python_symlink_multi_cam_streamer" because another target with the same name already exists.`
> 
> Please analyze the correct way to structure a mixed Python/Message package in ROS 2 Humble/Jazzy to avoid this collision. I need to:
> 1. Generate the Python bindings for the messages.
> 2. Install the Python source code for the node.
> 3. Ensure `setup.py` is removed if it conflicts with `ament_cmake_python`.
> 
> Here is my current `CMakeLists.txt`:
> [Paste content here]


Use this prompt with your AI assistant to generate rover-side testing code.

## Prompt

You are writing rover-side ROS2 test code to publish telemetry for a GUI.
Generate a Python ROS2 node (rclpy) that simulates and publishes:

- RSSI in dBm on `/rssi`
- latency in ms on `/latency_ms`
- battery percentage on `/battery`

Use `std_msgs/msg/Float32` for all three topics. The node must be easy to
configure, with all parameters grouped at the top of the file. Provide a
clean, single-file script with a `main()` entry point, and include a short
README section in comments at the top that describes how to run it.

### Required configuration block at the top

```
# --- Config ---
PUBLISH_HZ = 2.0
RSSI_DBM_RANGE = (-90.0, -40.0)
LATENCY_MS_RANGE = (20.0, 120.0)
BATTERY_PCT_RANGE = (10.0, 100.0)
RSSI_TOPIC = "/rssi"
LATENCY_TOPIC = "/latency_ms"
BATTERY_TOPIC = "/battery"
SEED = 42  # optional: set to None to disable deterministic output
```

### Behavior requirements

- Publish all three topics at the same rate (`PUBLISH_HZ`).
- Simulated values should vary smoothly (not pure random jumps).
- Battery should trend downward slowly, then wrap or clamp at the low end.
- RSSI and latency should have mild noise but stay within their ranges.
- Provide a clean shutdown on Ctrl+C.

### Output expectations

Return only the Python code file (no extra explanation). The code should
run with:

```
python3 rover_telemetry_publisher.py
```
