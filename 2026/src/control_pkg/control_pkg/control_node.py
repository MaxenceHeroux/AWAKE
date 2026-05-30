import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import numpy as np
import cv2
import math
import time
from scipy.interpolate import splprep, splev

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
# Import du message Ackermann standardisé
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # --- Configuration de la QoS (Indispensable pour tes contrôleurs moteurs) ---
        cmd_qos = QoSProfile(depth=10)
        cmd_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        cmd_qos.reliability = ReliabilityPolicy.RELIABLE
        
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE

        # --- Publishers & Subscribers ---
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', cmd_qos)
        self.path_pub = self.create_publisher(Path, '/planned_trajectory', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/ldlidar_node/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        
        # --- TF Listener ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # --- Variables d'état ---
        self.state = 'EXPLORE' # 'EXPLORE', 'PROCESS_MAP', 'RACE'
        self.start_time = time.time()
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.latest_map_msg = None
        self.optimized_trajectory = [] 
        self.has_left_origin = False 
        
        # =========================================================================
        # --- PARAMÈTRES GÉOMÉTRIQUES ET DYNAMIQUES ACKERMANN ---
        # =========================================================================
        self.wheelbase = 0.21          # Empattement (L) en mètres (à ajuster selon ton châssis)
        self.max_steering = 0.61       # Braquage max des roues avant (en radians, environ 35°)
        
        # Paramètres Pure Pursuit
        self.lookahead_distance = 1.5  # Distance de visée (m)
        self.max_speed = 0.6           # Vitesse max en ligne droite (m/s)
        self.min_speed = 0.1           # Vitesse min en courbe (m/s)
        # =========================================================================
        
        # --- Paramètres PID (Wall-following) ---
        self.kp = 1.4   
        self.ki = 0.0   
        self.kd = 0.1   
        
        self.integral_error = 0.0
        self.prev_error = 0.0
        self.max_integral = 2.0  
        self.last_time = self.get_clock().now()
        
        self.pose_timer = self.create_timer(0.1, self.check_pose_and_state)
        self.get_logger().info("ControlNode Ackermann initialisé. État : EXPLORATION.")

    def euler_from_quaternion(self, x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def publish_trajectory_to_ros(self):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for px, py in self.optimized_trajectory:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(px) 
            pose.pose.position.y = float(py)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)

    def odom_callback(self, msg):
        pass 

    def check_pose_and_state(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'ldlidar_base', rclpy.time.Time())
        except TransformException:
            return

        self.current_x = t.transform.translation.x
        self.current_y = t.transform.translation.y
        q = t.transform.rotation
        self.current_yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
        
        if self.state == 'EXPLORE':
            distance_to_origin = math.sqrt(self.current_x**2 + self.current_y**2)
            time_elapsed = time.time() - self.start_time
            
            if not self.has_left_origin and distance_to_origin > 0.5:
                self.has_left_origin = True
                self.get_logger().info("Le robot a quitté la zone de départ.")

            self.get_logger().info(f"Exploration: Dist = {distance_to_origin:.2f} m, Temps = {time_elapsed:.1f} s", throttle_duration_sec=2.0)
            
            if self.has_left_origin and distance_to_origin < 0.3 and time_elapsed > 10.0:
                self.get_logger().info("BOUCLE TERMINÉE ! Arrêt et traitement de la carte...")
                self.stop_robot()
                self.state = 'PROCESS_MAP'
                self.process_map_to_trajectory()

    def map_callback(self, msg):
        self.latest_map_msg = msg

    # --- Mode Exploration : Wall-Following PID converti en Ackermann ---
    def scan_callback(self, msg):
        if self.state != 'EXPLORE':
            return
            
        ranges_360 = np.full(360, np.nan)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
                continue
            angle_rad = angle_min + i * angle_inc
            angle_deg = int(math.degrees(angle_rad)) % 360
            if np.isnan(ranges_360[angle_deg]) or r < ranges_360[angle_deg]:
                ranges_360[angle_deg] = r

        left_sector = ranges_360[10:91]
        right_sector = ranges_360[270:351]
        
        def get_robust_distance(sector, default_val=3.0):
            valid_data = sector[~np.isnan(sector)] 
            if len(valid_data) == 0: return default_val 
            return np.median(valid_data) 

        left_dist = get_robust_distance(left_sector)
        right_dist = get_robust_distance(right_sector)
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0: dt = 0.001 
            
        error = left_dist - right_dist
        
        P = self.kp * error
        self.integral_error += error * dt
        self.integral_error = max(min(self.integral_error, self.max_integral), -self.max_integral)
        I = self.ki * self.integral_error
        derivative = (error - self.prev_error) / dt
        D = self.kd * derivative
        
        virtual_omega = P + I + D
        self.prev_error = error
        self.last_time = current_time
        
        explore_speed = 0.1 # Vitesse constante en exploration
        
        # Conversion mathématique de la vitesse angulaire virtuelle en angle de braquage réel (Ackermann)
        steering_angle = math.atan2(virtual_omega * self.wheelbase, explore_speed)
        steering_angle = max(min(steering_angle, self.max_steering), -self.max_steering)
        
        # Publication de la commande Ackermann
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = self.get_clock().now().to_msg()
        ack_msg.header.frame_id = 'ldlidar_base'
        ack_msg.drive.speed = explore_speed
        ack_msg.drive.steering_angle = steering_angle
        self.cmd_pub.publish(ack_msg)

    # --- Traitement d'image et Planification ---
    def process_map_to_trajectory(self):
        if self.latest_map_msg is None:
            self.get_logger().error("Aucune carte reçue.")
            return

        width = self.latest_map_msg.info.width
        height = self.latest_map_msg.info.height
        resolution = self.latest_map_msg.info.resolution
        origin_x = self.latest_map_msg.info.origin.position.x
        origin_y = self.latest_map_msg.info.origin.position.y

        map_array_1d = np.array(self.latest_map_msg.data, dtype=np.int8)
        map_image_2d = map_array_1d.reshape((height, width))
        
        walls_img = np.zeros((height, width), dtype=np.uint8)
        walls_img[map_image_2d > 50] = 255 
        
        bridge_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (21, 21))
        walls_img = cv2.morphologyEx(walls_img, cv2.MORPH_CLOSE, bridge_kernel)
        
        kernel = np.ones((3,3), np.uint8)
        walls_img = cv2.dilate(walls_img, kernel, iterations=1)
        num_labels, labels = cv2.connectedComponents(walls_img)

        u0 = max(0, min(width - 1, int((0.0 - origin_x) / resolution)))
        v0 = max(0, min(height - 1, int((0.0 - origin_y) / resolution)))

        target_labels = set()
        for u in range(u0, 0, -1):
            if labels[v0, u] > 0: target_labels.add(labels[v0, u]); break
        for u in range(u0, width):
            if labels[v0, u] > 0: target_labels.add(labels[v0, u]); break
        for v in range(v0, 0, -1):
            if labels[v, u0] > 0: target_labels.add(labels[v, u0]); break
        for v in range(v0, height):
            if labels[v, u0] > 0: target_labels.add(labels[v, u0]); break

        clean_walls = np.zeros((height, width), dtype=np.uint8)
        for lbl in target_labels:
            clean_walls[labels == lbl] = 255

        track_img = np.zeros((height, width), dtype=np.uint8)
        ff_mask = np.zeros((height+2, width+2), dtype=np.uint8)
        ff_mask[1:-1, 1:-1] = clean_walls 
        cv2.floodFill(track_img, ff_mask, (u0, v0), 255)

        cv_image = cv2.morphologyEx(track_img, cv2.MORPH_CLOSE, kernel)
        skeleton = cv2.ximgproc.thinning(cv_image)
        skel_bin = (skeleton // 255).astype(np.uint8)
        
        kernel_prune = np.array([[1, 1, 1], [1, 10, 1], [1, 1, 1]], dtype=np.uint8)
        for _ in range(50):
            filtered = cv2.filter2D(skel_bin, -1, kernel_prune)
            skel_bin[filtered == 11] = 0
            
        skeleton = skel_bin * 255
        contours, _ = cv2.findContours(skeleton, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if not contours: return
        main_contour = max(contours, key=len)
        if len(main_contour) < 10: return

        sorted_points = [(pt[0][0], pt[0][1]) for pt in main_contour][::3]

        world_points_x = []
        world_points_y = []
        for (u, v) in sorted_points:
            world_points_x.append(origin_x + (u * resolution))
            world_points_y.append(origin_y + (v * resolution))

        tck, u = splprep([world_points_x, world_points_y], s=0.0, per=1)
        u_new = np.linspace(u.min(), u.max(), 200)
        smooth_x, smooth_y = splev(u_new, tck)
        self.optimized_trajectory = list(zip(smooth_x, smooth_y))
        
        # Sauvegarde de l'image de debug
        try:
            debug_img = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            for px, py in self.optimized_trajectory:
                u = int((px - origin_x) / resolution)
                v = int((py - origin_y) / resolution)
                if 0 <= u < width and 0 <= v < height:
                    cv2.circle(debug_img, (u, v), 1, (0, 0, 255), -1) 
            save_path = os.path.expanduser('~/debug_trajectory.png')
            cv2.imwrite(save_path, debug_img)
        except Exception:
            pass

        self.get_logger().info("Passage au mode RACE (Ackermann Pure Pursuit) !")
        self.state = 'RACE'
        self.timer = self.create_timer(0.05, self.race_control_loop)

    # --- Mode Course : Pure Pursuit purement géométrique pour Ackermann ---
    def race_control_loop(self):
        if self.state != 'RACE' or not self.optimized_trajectory:
            return

        self.publish_trajectory_to_ros()

        # Recherche du point le plus proche
        min_dist = float('inf')
        closest_idx = 0
        for i, (px, py) in enumerate(self.optimized_trajectory):
            dist = math.hypot(px - self.current_x, py - self.current_y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Recherche du point de visée (Lookahead Target)
        target_idx = closest_idx
        for i in range(closest_idx, closest_idx + len(self.optimized_trajectory)):
            idx = i % len(self.optimized_trajectory)
            px, py = self.optimized_trajectory[idx]
            dist = math.hypot(px - self.current_x, py - self.current_y)
            if dist >= self.lookahead_distance:
                target_idx = idx
                break
                
        target_x, target_y = self.optimized_trajectory[target_idx]

        # Calcul de l'angle d'erreur local (alpha)
        alpha = math.atan2(target_y - self.current_y, target_x - self.current_x) - self.current_yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        # --- LOI DE COMMANDE ACKERMANN PURE PURSUIT NATIVE ---
        steering_angle = math.atan2(2.0 * self.wheelbase * math.sin(alpha), self.lookahead_distance)
        steering_angle = max(min(steering_angle, self.max_steering), -self.max_steering)

        # Profil de vitesse adaptatif basé sur la sévérité du virage à venir
        speed_factor = 1.0 - (abs(alpha) / 1.5)
        speed_factor = max(min(speed_factor, 1.0), 0.0)
        current_speed = self.min_speed + (self.max_speed - self.min_speed) * speed_factor

        # Publication de la consigne Ackermann
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = self.get_clock().now().to_msg()
        ack_msg.header.frame_id = 'ldlidar_base'
        ack_msg.drive.speed = current_speed
        ack_msg.drive.steering_angle = steering_angle
        self.cmd_pub.publish(ack_msg)

    def stop_robot(self):
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = self.get_clock().now().to_msg()
        ack_msg.header.frame_id = 'ldlidar_base'
        ack_msg.drive.speed = 0.0
        ack_msg.drive.steering_angle = 0.0
        self.cmd_pub.publish(ack_msg)
        self.get_logger().info("Moteurs et direction réinitialisés.")

        if hasattr(self, 'path_pub'):
            empty_path = Path()
            empty_path.header.frame_id = 'map'
            empty_path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(empty_path)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()