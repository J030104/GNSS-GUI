# URC Rover Control GUI - GNSS Subsystem

This package contains the source code for the GNSS & Communication GUI.

## Directory Structure

```
gnss_gui/
├── main.py                # Application entry point
├── config.py              # Central Configuration (IPs, Ports, Names)
├── requirements.txt       # Python dependencies
├── README.md              # This file
├── components/            # Reusable GUI components
│   ├── video_viewer.py    # Video viewing widget
│   ├── video_layout_tabs.py # Manages grid layouts (e.g., 6-Cam)
│   ├── map_viewer.py      # GNSS map placeholder
│   └── control_panel.py   # Camera selection and stream control
├── subsystems/            # Specific tab implementations
│   └── gnss_comm.py       # Main Logic for GNSS/Video tab
└── utilities/             # Backend logic
    ├── video_streamer.py  # FFmpeg management & Local Camera Logic
    └── rover_stream_client.py # Client for remote stream requests
```

## Configuration

All configurable parameters (Camera names, Ports, IP addresses) are located in `config.py`. 
Changes made there are automatically reflected in the GUI Dropdowns and logic.

## Usage

Run from the project root:
```bash
python -m gnss_gui.main
```
