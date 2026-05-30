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
    lidar_pkg_dir = get_package_share_directory('ldlidar_stl_ros2')

    # On remplace l'inclusion du launch file par la définition directe du node
    # Cela évite le conflit de TF (doublon base_link -> base_laser)
    
    lidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
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

    imu_driver = Node(
        package='IMU_read',
        executable='IMU_node',
        output='screen'
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

# --- 3. ODOMETRIE LASER (Via son Launch File) ---
    # On appelle le launch file que vous avez modifié à l'étape 1
    rf2o_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rf2o_laser_odometry'), 'launch', 'rf2o_laser_odometry.launch.py')
        )
    )

    # --- 4. FUSION DE CAPTEURS (EKF) ---
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[('odometry/filtered', '/odom')]
    )

    # --- 5. SLAM (SLAM TOOLBOX) ---
    slam_toolbox = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'slam_params_file': slam_config_path, # <--- CORRECTION : params_file -> slam_params_file
                'use_sim_time': 'false',
                # Ajoutez ceci pour dire "Je m'en fiche de la fiabilité, prends les données !"
                # Note : Tous les launch files ne supportent pas cet argument, mais essayons.
            }.items()
        )

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
        tf_laser,
        tf_imu,
        lidar_node, # <-- Remplacé lidar_launch par lidar_node
        imu_driver, # <-- Ajouté
        rf2o_launch,
        ekf_node,
        # scan_bridge, # Désactivé car le lidar publie directement sur /scan maintenant
        motor_driver,
        slam_toolbox,
        foxglove_bridge  # <-- Ajouté ici
    ])