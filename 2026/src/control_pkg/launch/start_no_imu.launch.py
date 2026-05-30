import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_name = 'robot_pkg'
    pkg_share = get_package_share_directory(pkg_name)

    # --- CONFIGURATIONS ---
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    slam_config_path = os.path.join(pkg_share, 'config', 'my_params_slam.yaml')

    # --- 1. TRANSFORMS STATIQUES (TF) ---
    # Syntaxe : x y z yaw pitch roll frame_id child_frame_id
    # CORRECTION : On attache base_link -> base_laser car le node lidar publie désormais sur base_laser
    tf_laser = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.1', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'base_laser']
        )
    
    tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'imu_link']
    )

# --- 2. DRIVERS MATÉRIEL (Via Inclusion) ---
    
    # On cherche le dossier du package lidar
    # (Remplace 'ldlidar_stl_ros2' par le nom EXACT du dossier dans /install/share/)
    lidar_pkg_dir = get_package_share_directory('ldlidar_node')

    # On remplace l'inclusion du launch file par la définition directe du node
    # Cela évite le conflit de TF (doublon base_link -> base_laser)
    
    lidar_node = Node(
        package='ldlidar_node',
        executable='ldlidar_slam',
        name='LD19',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD19'},
            {'topic_name': 'scan'},
            {'frame_id': 'base_laser'},
            {'port_name': '/dev/ttyUSB0'},
            {'port_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0}
        ]
    )

    motor_driver = Node(
        package='motor_control',
        executable='motor_driver',
        output='screen'
    )


    # scan_bridge = ExecuteProcess(
    #     cmd=['python3', os.path.join(pkg_share, 'launch', 'scan_bridge.py')],
    #     output='screen'
    # )



    # --- 6. VISUALISATION (FOXGLOVE BRIDGE) ---
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'tls': False,
            'send_buffer_limit': 10000000, # Augmenté à 10MB pour éviter le lag de la map
            'topic_whitelist': ['.*']      # Autorise tous les topics
        }]
    )

    # --- LANCEMENT ---
    return LaunchDescription([
        lidar_node, # <-- Remplacé lidar_launch par lidar_node
        # scan_bridge, # Désactivé car le lidar publie directement sur /scan maintenant
        motor_driver,
        slam_toolbox,
        foxglove_bridge  # <-- Ajouté ici
    ])