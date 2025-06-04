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

        self.moteur = 0
        self.direction = 0
        self.moteur_prec = 0
        self.direction_prec = 0

    def scan_callback(self, msg: LaserScan):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        intensities = msg.intensities

        # Mettre à jour self.visu avec les distances valides
        for i in range(len(ranges)):
            angle_deg = int(math.degrees(angle_min + i * angle_increment)) % 360
            distance = ranges[i]
            intensity = intensities[i]
            if not math.isinf(distance) and not math.isnan(distance) and (intensity> 1.0):
                self.visu[angle_deg] = distance
            else:
                self.visu[angle_deg] = 5.0

        # Coefficients de commande
        P_direction = 0.6
        D_direction = 0.02
        P_moteur = 8.0
        D_moteur = 0.01

        # backup
        # P_direction = 1.0
        # D_direction = 0.15
        # P_moteur = 6.0
        # D_moteur = 0.5

        # Sommes pour direction (gauche et droite)
        nb_droite = 0
        somme_droite_direction = 0
        for i in self.visu[25:90]:
            somme_droite_direction += i
            nb_droite += 1

        moyenne_droite = somme_droite_direction/nb_droite
        
        nb_gauche = 0
        somme_gauche_direction = 0
        for i in self.visu[270:335]:
            somme_gauche_direction += i
            nb_gauche += 1

        moyenne_gauche = somme_gauche_direction/nb_gauche

        if moyenne_droite > moyenne_gauche:
            front_view = self.visu[325:295]
        elif moyenne_gauche > moyenne_droite:
            front_view = self.visu[35:65]
        else:
            front_view = self.visu[:10] + self.visu[350:]

        moyenne_moteur = sum(front_view) / len(front_view) if front_view else 0.0


        front_au = self.visu[:10] + self.visu[350:]




        # Somme pour la vitesse : 0° à 19° et 340° à 359° → total = 40 valeurs


        # Logique de recul si obstacle très proche
        if (sum(front_au)/len(front_au)) < 0.25:
            self.moteur = -1.0
        else:
            somme_totale = somme_droite_direction + somme_gauche_direction
            self.moteur_prec = self.moteur
            err_mot = (moyenne_moteur / 20.0)
            acc_moteur = err_mot - self.moteur_prec
            self.moteur = err_mot * P_moteur + acc_moteur * D_moteur
            #vitesse constante
            # self.moteur = 0.4

        # Calcul direction
        self.direction_prec = self.direction
        err_direction = (moyenne_droite - moyenne_gauche)
        acc_direction = self.direction - self.direction_prec
        
        self.direction = err_direction * P_direction + acc_direction * D_direction

        

        # Clamp des valeurs
        self.moteur = max(min(self.moteur, 1.0), -1.0)
        self.direction = max(min(self.direction, 1.0), -1.0)

        # Publication
        self.motor_pub.publish(Float32(data=self.moteur))
        self.direction_pub.publish(Float32(data=self.direction))

        self.get_logger().info(f"Moteur: {self.moteur:.2f}, Direction: {self.direction:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = ScanToCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
