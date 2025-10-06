"""Reusable GUI components for the URC control interface.

This package exposes classes such as `VideoViewer`, `MapViewer`,
`ControlPanel`, `StatusBar` and `LogViewer`.  Each component is
implemented as a PyQt widget and can be integrated into different
subsystems with minimal modification.
"""

from .video_viewer import VideoViewer
from .map_viewer import MapViewer
from .control_panel import ControlPanel
from .status_bar import StatusBar
from .log_viewer import LogViewer
from .shell_tabs import ShellTabs

__all__ = [
    "VideoViewer",
    "MapViewer",
    "ControlPanel",
    "StatusBar",
    "LogViewer",
    "ShellTabs",
]