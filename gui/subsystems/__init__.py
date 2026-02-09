"""Subsystem tabs for the URC control GUI.

Each module in this package defines a subclass of ``QWidget`` that
represents a high‑level subsystem.  See ``gnss_comm.py`` for a full
implementation and the other modules for placeholders that teams can
extend.
"""

from .gnss_comm import GNSSCommWidget
from .autonomous_navigation import AutonomousNavigationWidget
from .power_electronics import PowerElectronicsWidget
from .robotic_arm_delivery import RoboticArmDeliveryWidget
from .science_mission import ScienceMissionWidget
from .drone import DroneWidget

__all__ = [
    "GNSSCommWidget",
    "AutonomousNavigationWidget",
    "PowerElectronicsWidget",
    "RoboticArmDeliveryWidget",
    "ScienceMissionWidget",
    "DroneWidget",
]