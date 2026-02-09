"""Placeholder widget for the robotic arm & delivery subsystem.

This tab contains simple sliders representing joint angles of a
robotic arm and a log area.  Real implementations should expose
inverse kinematics, gripper control, force feedback and camera feeds
from the end effector.
"""

from typing import Optional

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider

from ..components import LogViewer


class RoboticArmDeliveryWidget(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout()
        layout.addWidget(QLabel("Robotic Arm & Delivery (placeholder)"))

        # Simulate three joints
        joint_layout = QVBoxLayout()
        for i in range(1, 4):
            row = QHBoxLayout()
            label = QLabel(f"Joint {i}:")
            slider = QSlider(Qt.Horizontal)
            slider.setRange(-180, 180)
            slider.setValue(0)
            row.addWidget(label)
            row.addWidget(slider)
            joint_layout.addLayout(row)
        layout.addLayout(joint_layout)

        self.log_viewer = LogViewer()
        layout.addWidget(self.log_viewer)
        self.setLayout(layout)
        self.log_viewer.append("Robotic arm subsystem not yet implemented.")