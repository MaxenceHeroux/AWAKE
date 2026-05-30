#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

class ScanBridge(Node):
    def __init__(self):
        super().__init__('scan_bridge')

        # 1. Écouter le LiDAR en mode "Best Effort" (Compatible LD19)
        qos_input = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 2. Publier pour le SLAM en mode "Reliable" (Standard ROS 2)
        qos_output = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            LaserScan,
            '/ldlidar_node/scan',  # <-- Le topic actuel de votre LiDAR
            self.listener_callback,
            qos_input
        )

        self.pub = self.create_publisher(
            LaserScan,
            '/scan_reliable',      # <-- Le nouveau topic propre pour le SLAM
            qos_output
        )

    def listener_callback(self, msg):
        # On republie le message tel quel
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanBridge()
    node.get_logger().info("Pont LiDAR activé : /ldlidar_node/scan (BestEffort) -> /scan_reliable (Reliable)")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()