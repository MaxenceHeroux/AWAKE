import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math

class ScanToCommandNode(Node):
    def __init__(self):
        super().__init__('scan_to_command_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/ldlidar_node/scan',
            self.scan_callback,
            10
        )

        self.motor_pub = self.create_publisher(Float32, '/Moteur', 10)
        self.direction_pub = self.create_publisher(Float32, '/Direction', 10)

        self.visu = [0.0] * 360

        self.moteur = 0.0
        self.direction = 0.0
        self.moteur_prec = 0.0
        self.direction_prec = 0.0

    def scan_callback(self, msg: LaserScan):
        # 1. Mise à jour de self.visu
        # On remet tout à 0.0 avant de remplir pour éviter de garder de vieux "fantômes"
        self.visu = [0.0] * 360 
        
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        intensities = msg.intensities

        for i in range(len(ranges)):
            # Calcul de l'angle en degrés (0-359)
            angle_deg = int(math.degrees(angle_min + i * angle_increment)) % 360
            
            dist = ranges[i]
            intensity = intensities[i]
            
            # FILTRAGE : Si valide, on garde la distance, sinon on laisse 0.0
            if not math.isinf(dist) and not math.isnan(dist) and (intensity > 1.0):
                self.visu[angle_deg] = dist
            else:
                self.visu[angle_deg] = 0.0 

        # --- Fonction interne pour calculer la moyenne en ignorant les 0.0 ---
        def calculer_moyenne_zone(indices):
            somme = 0.0
            compteur = 0
            for i in indices:
                val = self.visu[i]
                if val > 0.01: # On ignore les valeurs nulles ou très proches de 0 (bruit)
                    somme += val
                    compteur += 1
            
            if compteur > 0:
                return somme / compteur
            return 0.0 # Si la zone est vide ou invisible, on renvoie 0

        # --- Définition des zones (Indices) ---
        # Droite : 25° à 89°
        indices_droite = range(25, 90)
        # Gauche : 270° à 334°
        indices_gauche = range(270, 335)
        
        # Calcul des moyennes dynamiques
        moyenne_droite = calculer_moyenne_zone(indices_droite)
        moyenne_gauche = calculer_moyenne_zone(indices_gauche)

        # --- Logique de sélection de la vue avant (Front View) ---
        # Correction du bug de slicing [325:295] -> On utilise l'addition de listes pour le wrap
        if moyenne_droite > moyenne_gauche:
            # On regarde un peu vers la droite (ex: 295° à 325° n'est pas logique pour droite ?)
            # Logiquement si on va à droite, on regarde devant-droite : 330° à 30° ?
            # Je corrige selon ta logique probable : Regarder là où on va tourner.
            indices_front = list(range(325, 360)) + list(range(0, 10)) # Devant large
        elif moyenne_gauche > moyenne_droite:
             indices_front = list(range(350, 360)) + list(range(0, 35)) # Devant large
        else:
            # Tout droit : petit cône devant
            indices_front = list(range(350, 360)) + list(range(0, 10))

        moyenne_moteur = calculer_moyenne_zone(indices_front)

        # --- Zone de sécurité stricte (Freinage d'urgence) ---
        # Cône très étroit devant le robot : 355° à 5°
        indices_securite = list(range(359, 360)) + list(range(0, 1))
        dist_securite = calculer_moyenne_zone(indices_securite)


        # --- Paramètres PID ---
        P_direction = 0.9#0.6
        D_direction = 0.04
        P_moteur = 6.0#8.0
        D_moteur = 0.01

        # --- Logique Moteur ---
        # Si un obstacle est détecté très proche (et que la distance n'est pas 0/invisible), AU arrêt
        if 0.01 < dist_securite < 0.08:
            self.moteur = -1.0 # Recul
        else:
            self.moteur_prec = self.moteur
            # On utilise la moyenne calculée sur la zone choisie
            err_mot = (moyenne_moteur / 20.0)
            acc_moteur = err_mot - self.moteur_prec
            self.moteur = err_mot * P_moteur + acc_moteur * D_moteur

        # --- Logique Direction ---
        self.direction_prec = self.direction
        
        # Astuce : Si un côté est 0 (invisible), on considère qu'il est très loin ? 
        # Ou on l'ignore ? Ici si moyenne_gauche est 0, err_direction sera grande vers la droite.
        # C'est un comportement acceptable : "Je ne vois rien à gauche, donc c'est peut-être libre".
        
        err_direction = (moyenne_droite - moyenne_gauche)
        acc_direction = self.direction - self.direction_prec
        self.direction = err_direction * P_direction + acc_direction * D_direction

        # Clamp (Limitation des valeurs entre -1 et 1)
        self.moteur = max(min(self.moteur, 1.0), -1.0)
        self.direction = max(min(self.direction, 1.0), -1.0)

        # Publication
        self.motor_pub.publish(Float32(data=self.moteur))
        self.direction_pub.publish(Float32(data=self.direction))

        self.get_logger().info(
            f"Mot: {self.moteur:.2f}, Dir: {self.direction:.2f} | "
            f"D: {moyenne_droite:.2f}, G: {moyenne_gauche:.2f}, Front: {dist_securite:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ScanToCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()