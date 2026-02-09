from __future__ import annotations

import threading
import math
import time
from typing import Optional, List

from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QTimer
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
    QLineEdit, QPushButton, QGroupBox, QFormLayout,
    QFrame
)

import rclpy
from rclpy.node import Node
from rclpy.time import Time

try:
    from sensor_msgs.msg import NavSatFix, NavSatStatus
except ImportError:
    NavSatFix = None
    NavSatStatus = None


class GNSSNode(Node):
    def __init__(self):
        super().__init__('gnss_side_panel_node')
        
        if NavSatFix is None:
            self.get_logger().error("sensor_msgs not found. ROS 2 functionality disabled.")
            return

        # Subscriber
        self.subscription = self.create_subscription(
            NavSatFix,
            '/ublox_gps_node/fix',
            self.listener_callback,
            10
        )
        
        # Publisher
        self.publisher = self.create_publisher(
            NavSatFix,
            'GNSS_input',
            10
        )
        
        self.latest_msg: Optional[NavSatFix] = None
        self.new_data_available = False
        self._lock = threading.Lock()

    def listener_callback(self, msg: NavSatFix):
        with self._lock:
            self.latest_msg = msg
            self.new_data_available = True

    def publish_manual_override(self, lat: float, lon: float, alt: float):
        if NavSatFix is None:
            return

        msg = NavSatFix()
        # header.stamp: Current ROS time
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps_link"
        
        # status
        # Per user request: "0 (STATUS_NO_FIX) or 1 (STATUS_FIX)"
        # We select 1 to indicate a valid FIX for the manual override.
        msg.status.status = 1 

        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.altitude = float(alt)
        
        # position_covariance
        msg.position_covariance = [0.0] * 9
        msg.position_covariance_type = 0 # COVARIANCE_TYPE_UNKNOWN

        self.publisher.publish(msg)


class GNSSSidePanel(QWidget):
    """
    Side panel for GNSS monitoring and manual override.
    """
    data_received = pyqtSignal(object) # Emit NavSatFix object (or dict)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.node: Optional[GNSSNode] = None
        self.spin_thread: Optional[threading.Thread] = None

        self._init_ui()
        self._init_ros()

        # Timer to poll for new data from ROS thread
        self.timer = QTimer()
        self.timer.timeout.connect(self._check_ros_data)
        self.timer.start(100) # 10Hz UI update check

    def _init_ui(self):
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)

        # --- Section 1: GNSS Monitor ---
        monitor_group = QGroupBox("GNSS Monitor")
        monitor_layout = QFormLayout()
        
        self.lbl_lat = QLabel("Waiting...")
        self.lbl_lon = QLabel("Waiting...")
        self.lbl_alt = QLabel("Waiting...")
        self.lbl_acc_h = QLabel("Waiting...")
        self.lbl_acc_v = QLabel("Waiting...")

        monitor_layout.addRow("Latitude:", self.lbl_lat)
        monitor_layout.addRow("Longitude:", self.lbl_lon)
        monitor_layout.addRow("Altitude:", self.lbl_alt)
        monitor_layout.addRow("H. Accuracy:", self.lbl_acc_h)
        monitor_layout.addRow("V. Accuracy:", self.lbl_acc_v)
        
        monitor_group.setLayout(monitor_layout)
        layout.addWidget(monitor_group)

        # --- Section 2: GNSS Navigation Point ---
        override_group = QGroupBox("GNSS Navigation Point")
        override_layout = QFormLayout()

        self.input_lat = QLineEdit()
        self.input_lat.setPlaceholderText("Lat")
        self.input_lon = QLineEdit()
        self.input_lon.setPlaceholderText("Lon")
        self.input_alt = QLineEdit()
        self.input_alt.setPlaceholderText("Alt")
        
        self.btn_set = QPushButton("Set")
        self.btn_set.clicked.connect(self._on_set_clicked)

        override_layout.addRow("Latitude:", self.input_lat)
        override_layout.addRow("Longitude:", self.input_lon)
        override_layout.addRow("Altitude:", self.input_alt)
        override_layout.addWidget(self.btn_set)

        override_group.setLayout(override_layout)
        layout.addWidget(override_group)
        
        layout.addStretch() # Push everything up
        self.setLayout(layout)

    def _init_ros(self):
        if not rclpy.ok():
            try:
                rclpy.init()
            except Exception:
                pass # Already initialized
        
        try:
            self.node = GNSSNode()
            self.spin_thread = threading.Thread(target=self._spin_node, daemon=True)
            self.spin_thread.start()
        except Exception as e:
            print(f"Failed to initialize GNSS ROS Node: {e}")

    def _spin_node(self):
        if self.node:
            rclpy.spin(self.node)

    def _check_ros_data(self):
        if self.node and self.node.new_data_available:
            with self.node._lock:
                msg = self.node.latest_msg
                self.node.new_data_available = False # Consumed
            
            if msg:
                self._update_display(msg)

    def _update_display(self, msg: NavSatFix):
        self.lbl_lat.setText(f"{msg.latitude:.6f}")
        self.lbl_lon.setText(f"{msg.longitude:.6f}")
        self.lbl_alt.setText(f"{msg.altitude:.2f} m")

        # Accuracy Calculation
        # position_covariance is row-major 9 elements (3x3)
        # [0] [1] [2] -> E E, E N, E U
        # [3] [4] [5] -> N E, N N, N U
        # [6] [7] [8] -> U E, U N, U U
        cov = msg.position_covariance
        
        # User Logic:
        # H Acc: sqrt((cov[0] + cov[4]) / 2)
        # V Acc: sqrt(cov[8])
        # IF position_covariance_type == 0 (Unknown) -> N/A

        if msg.position_covariance_type == 0:
            self.lbl_acc_h.setText("N/A")
            self.lbl_acc_v.setText("N/A")
        else:
            try:
                h_var = (cov[0] + cov[4]) / 2.0
                h_acc = math.sqrt(h_var) if h_var >= 0 else 0.0
                self.lbl_acc_h.setText(f"{h_acc:.2f} m")
                
                v_var = cov[8]
                v_acc = math.sqrt(v_var) if v_var >= 0 else 0.0
                self.lbl_acc_v.setText(f"{v_acc:.2f} m")
            except ValueError:
                self.lbl_acc_h.setText("Err")
                self.lbl_acc_v.setText("Err")

    def _on_set_clicked(self):
        if not self.node:
            print("ROS Node not initialized")
            return

        try:
            lat = float(self.input_lat.text())
            lon = float(self.input_lon.text())
            alt = float(self.input_alt.text())
            
            self.node.publish_manual_override(lat, lon, alt)
            print(f"Published GNSS Override: {lat}, {lon}, {alt}")
        except ValueError:
            print("Invalid input for GNSS Override")
