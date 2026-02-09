#!/usr/bin/env python3
import time
import random
import threading
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32

class GNSSTelemetryNode(Node):
    def __init__(self):
        super().__init__('gnss_telemetry_node')
        self.get_logger().info("Starting GNSS & Telemetry Node...")

        # --- Publishers ---
        
        # GNSS Fix (Simulated)
        self.gnss_pub = self.create_publisher(NavSatFix, '/ublox_gps_node/fix', 10)
        
        # Telemetry
        self.rssi_pub = self.create_publisher(Float32, '/rssi', 10)
        self.latency_pub = self.create_publisher(Float32, '/latency_ms', 10)
        self.battery_pub = self.create_publisher(Float32, '/battery', 10)

        # --- Subscribers ---
        
        # Manual Override Input
        self.create_subscription(NavSatFix, 'GNSS_input', self.gnss_input_callback, 10)

        # --- State ---
        self.current_lat = 38.3976  # Hanksville-ish
        self.current_lon = -110.7025
        self.current_alt = 1350.0

        # --- Timers ---
        self.create_timer(1.0, self.publish_telemetry) # 1Hz telemetry
        self.create_timer(0.2, self.publish_gnss)      # 5Hz GNSS

    def gnss_input_callback(self, msg: NavSatFix):
        """Handle manual override inputs from the GUI."""
        self.get_logger().info(f"Received Manual GNSS Override: Lat={msg.latitude}, Lon={msg.longitude}, Alt={msg.altitude}")
        # Update simulated position to match override
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude

    def publish_gnss(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps_link"
        
        # Simulate slight drift
        drift_lat = random.uniform(-0.00001, 0.00001)
        drift_lon = random.uniform(-0.00001, 0.00001)
        
        msg.latitude = self.current_lat + drift_lat
        msg.longitude = self.current_lon + drift_lon
        msg.altitude = self.current_alt + random.uniform(-0.1, 0.1)

        # Status
        msg.status.status = 0 # STATUS_FIX
        msg.status.service = 1 # SIGNAL_GPS

        # Covariance (Simulate good accuracy)
        # [E, N, U] variance
        # 0.5m variance -> 0.25 covariance
        msg.position_covariance = [
            0.25, 0.0, 0.0,
            0.0, 0.25, 0.0,
            0.0, 0.0, 0.5
        ]
        msg.position_covariance_type = 2 # COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.gnss_pub.publish(msg)

    def publish_telemetry(self):
        # RSSI
        rssi_msg = Float32()
        rssi_msg.data = -45.0 + random.uniform(-5, 5)
        self.rssi_pub.publish(rssi_msg)

        # Latency
        lat_msg = Float32()
        lat_msg.data = 25.0 + random.uniform(0, 10)
        self.latency_pub.publish(lat_msg)

        # Battery
        bat_msg = Float32()
        bat_msg.data = 98.5 - (time.time() % 100) / 10.0 # Slowly draining
        self.battery_pub.publish(bat_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GNSSTelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
