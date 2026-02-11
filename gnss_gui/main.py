"""Main entry point for the URC control GUI.

This module creates the top‑level window, sets up the tabbed
subsystem interface and runs the Qt event loop.  To launch the
application simply run:

    python -m gnss_gui.main
"""

import sys
from typing import Optional

from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget

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
        
        # Set reasonable size constraints to prevent window from growing too large
        self.setMinimumSize(800, 600)
        self.setMaximumSize(1920, 1080)

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
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()