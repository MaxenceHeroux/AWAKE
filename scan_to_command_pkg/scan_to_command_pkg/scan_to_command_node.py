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

        # Tableau pour stocker les distances sur 360°
        self.visu = [0.0] * 360

    def scan_callback(self, msg: LaserScan):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        # Mettre à jour self.visu avec les distances valides
        for i in range(len(ranges)):
            angle_deg = int(math.degrees(angle_min + i * angle_increment)) % 360
            distance = ranges[i]
            if not math.isinf(distance) and not math.isnan(distance):
                self.visu[angle_deg] = distance
            else:
                self.visu[angle_deg] = 0.0

        # Coefficients de commande
        alpha_direction = 0.1
        alpha_moteur = 10

        # Sommes pour direction (gauche et droite)
        somme_droite_direction = sum(self.visu[:90])     # 0° à 89°
        somme_gauche_direction = sum(self.visu[270:])    # 270° à 359°

        # Somme pour la vitesse : 0° à 19° et 340° à 359° → total = 40 valeurs
        front_view = self.visu[:10] + self.visu[350:]
        moyenne_moteur = sum(front_view) / len(front_view) if front_view else 0.0

        # Logique de recul si obstacle très proche
        if moyenne_moteur < 0.20:
            moteur = -1.0  # Marche arrière d'urgence
        else:
            somme_totale = somme_droite_direction + somme_gauche_direction
            moteur = (moyenne_moteur / 20.0) * alpha_moteur

        # Calcul direction
        direction = (somme_droite_direction - somme_gauche_direction) * alpha_direction

        # Clamp des valeurs
        moteur = max(min(moteur, 1.0), -1.0)
        direction = max(min(direction, 1.0), -1.0)

        # Publication
        self.motor_pub.publish(Float32(data=moteur))
        self.direction_pub.publish(Float32(data=direction))

        self.get_logger().info(f"Moteur: {moteur:.2f}, Direction: {direction:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = ScanToCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
