"""Main entry point for the URC control GUI.

This module creates the top‑level window, sets up the tabbed
subsystem interface and runs the Qt event loop.  To launch the
application simply run:

    python -m gnss_gui.main
"""

import sys
import os
from pathlib import Path
from typing import Optional

# Load .env file (stdlib-only, no python-dotenv needed)
_env_path = Path(__file__).resolve().parent.parent / ".env"
if _env_path.is_file():
    with open(_env_path) as _f:
        for _line in _f:
            _line = _line.strip()
            if _line and not _line.startswith("#") and "=" in _line:
                _key, _, _val = _line.partition("=")
                os.environ.setdefault(_key.strip(), _val.strip())

from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont

from .components import StatusBar
from .subsystems import (
    GNSSCommWidget,
    AutonomousNavigationWidget,
    PowerElectronicsWidget,
    RoboticArmDeliveryWidget,
    ScienceMissionWidget,
    DroneWidget,
)


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("URC Rover Control GUI")
        self.resize(1200, 800)
        
        # Set minimum size constraint
        self.setMinimumSize(800, 600)

        # Create tab widget for subsystems
        self.tabs = QTabWidget()
        self.tabs.addTab(GNSSCommWidget(), "GNSS & Communication")
        self.tabs.addTab(AutonomousNavigationWidget(), "Autonomous Navigation")
        self.tabs.addTab(PowerElectronicsWidget(), "Power & Electronics")
        self.tabs.addTab(RoboticArmDeliveryWidget(), "Robotic Arm & Delivery")
        self.tabs.addTab(ScienceMissionWidget(), "Science Mission")
        self.tabs.addTab(DroneWidget(), "Drone")
        self.setCentralWidget(self.tabs)

        # Status bar
        self.status_bar = StatusBar()
        self.setStatusBar(self.status_bar)

    def closeEvent(self, event):  # type: ignore[override]
        """Perform any necessary cleanup on close."""
        # Could stop threads, close connections etc.
        super().closeEvent(event)


def main() -> None:
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps)
    app = QApplication(sys.argv)
    app.setFont(QFont("Segoe UI", 11))
    window = MainWindow()
    window.showMaximized()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()