# URC Rover Control GUI

This project provides a modular graphical user interface (GUI) written in
Python using **PyQt**.  It is intended for teams participating in the
University Rover Challenge (URC) and in particular supports the
``GNSS & Communication`` subsystem.  The design emphasises reliability,
flexibility and future expansion so that additional subsystems (such as
autonomous navigation, power management, robotic arm & delivery, science
mission instrumentation and drone operations) can easily be integrated.

## Features

The GUI is organised as a series of tabs, one for each subsystem.  In the
``GNSS & Communication`` tab the following functionality is provided:

- **Video viewer** – Displays one or more live video streams from the rover.
  In the prototype the viewer renders a static placeholder image, but the
  class is designed so it can be connected to a stream via `ffmpeg`/`ffplay`
  or another backend in the future.  Controls are provided for toggling
  different cameras, adjusting zoom/quality/brightness and monitoring
  bandwidth usage.
- **Map viewer** – Presents the rover’s current GNSS position and heading
  on a simple map.  The widget exposes methods for updating the marker
  location; in the prototype a blank canvas is used, but the design allows
  integration with mapping libraries (e.g. Folium, Qt Maps or ROS2 RViz).
- **Control panel** – Contains sliders and buttons to adjust video
  properties such as bitrate, brightness and zoom.  A drop‑down menu
  selects which camera feed is active.  Connection status and current
  bandwidth utilisation are displayed here as well.
- **Log viewer** – Shows status messages, connection logs and system
  diagnostics.  Developers can easily append messages to this widget from
  elsewhere in the code.
- **Status bar** – A persistent bar at the bottom of the window displays
  the current time and indicates whether the GUI is connected to the rover.

Each subsystem tab has been implemented as a separate class to encourage
modularity.  Additional widgets can be added or swapped out without
affecting the rest of the system.  To add a new subsystem simply
implement a new subclass of `QWidget` inside the `subsystems` package and
register it in `main.py`.

## Directory Structure

```
gnss_gui/
├── main.py                # Application entry point with tabbed interface
├── README.md              # This file
├── requirements.txt       # Python dependencies
├── components/            # Reusable GUI components
│   ├── __init__.py
│   ├── video_viewer.py    # Video viewing and control widget
│   ├── map_viewer.py      # GNSS map display widget
│   ├── control_panel.py   # Sliders and buttons for video and comm settings
│   ├── status_bar.py      # Displays current time and connection status
│   └── log_viewer.py      # Scrollable text area for logs
├── subsystems/            # High‑level subsystem tabs
│   ├── __init__.py
│   ├── gnss_comm.py       # Main GNSS & Communication tab
│   ├── autonomous_navigation.py  # Placeholder for autonomy subsystem
│   ├── power_electronics.py       # Placeholder for power & electronics
│   ├── robotic_arm_delivery.py    # Placeholder for robotic arm & delivery
│   ├── science_mission.py         # Placeholder for science mission
│   └── drone.py                   # Placeholder for drone subsystem
└── utilities/             # Helper classes and stubs
    ├── __init__.py
    ├── connection_manager.py  # Abstraction for SSH and network connection
    └── video_streamer.py      # Stub for interfacing with ffmpeg/ffplay
```

## Usage

Install the dependencies listed in ``requirements.txt`` (most notably
``PyQt5``) and run the application with:

```bash
python -m gnss_gui.main
```

The prototype will open a window with multiple tabs.  Only the
``GNSS & Communication`` tab contains functional widgets; the others are
placeholders that can be expanded by subsystem teams.

## Design Philosophy

URC guidelines stress that a reliable communication system is critical to
success【915565265420546†L61-L77】.  Additionally, teams must design their
communications to work over both near and far distances and be prepared
to operate from inside a metal building where Wi‑Fi signals do not
propagate【915565265420546†L79-L94】.  The GUI therefore includes bandwidth
and connection status indicators, camera controls to manage video quality
and bandwidth, and modular components so that additional sensors and
subsystems can be integrated without destabilising existing features.

Furthermore, URC encourages teams to ensure adequate situational
awareness because on‑board cameras provide limited visual context and
obstacles and hazards must be detected【915565265420546†L167-L171】.  The
video viewer and map components address this requirement and can be
expanded with more sophisticated visualisation in future iterations.
