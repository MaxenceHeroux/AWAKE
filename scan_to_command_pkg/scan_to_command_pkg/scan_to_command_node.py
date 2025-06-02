import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

import math

class ScanToCommandNode(Node):
    def __init__(self):
        super().__init__('scan_to_command_node')

        # Souscription au scan LIDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/ldlidar_node/scan',
            self.scan_callback,
            10
        )

        # Publication sur /Moteur et /Direction
        self.motor_pub = self.create_publisher(Float32, '/Moteur', 10)
        self.direction_pub = self.create_publisher(Float32, '/Direction', 10)

    def scan_callback(self, msg: LaserScan):
        angle_min = msg.angle_min  # radians
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        angles_deg = [math.degrees(angle_min + i * angle_increment) for i in range(len(ranges))]

        # Filtrer les données dans l'intervalle [-90°, 90°]
        filtered = [
            r for r, angle in zip(ranges, angles_deg)
            if -90.0 <= angle <= 90.0 and not math.isinf(r)
        ]

        if not filtered:
            self.get_logger().warn("Pas de données valides entre -90° et 90°.")
            return

        min_dist = min(filtered)
        self.get_logger().info(f"Min distance dans [-90°, 90°] = {min_dist:.2f} m")

        # Logique simple de commande
        moteur = 0.5 if min_dist > 1.0 else 0.0
        direction = 0.0  # pas de virage ici

        # Publier les commandes
        self.motor_pub.publish(Float32(data=float(moteur)))
        self.direction_pub.publish(Float32(data=float(direction)))

def main(args=None):
    rclpy.init(args=args)
    node = ScanToCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()