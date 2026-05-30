import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    lc_mgr_config_path = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params',
        'lifecycle_mgr_slam.yaml'
    )

    slam_config_path = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params',
        'slam_toolbox.yaml'
    )

    # 1. Le chef d'orchestre (gère UNIQUEMENT le ldlidar_node)
    lc_mgr_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[lc_mgr_config_path]
    )

    # 2. Le LiDAR
    ldlidar_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('ldlidar_node'),
            '/launch/ldlidar_bringup.launch.py'
        ]),
        launch_arguments={'node_name': 'ldlidar_node'}.items()
    )

    # 3. Le SLAM via son fichier officiel (avec paramètres forcés)
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('slam_toolbox'),
            '/launch/online_async_launch.py'
        ]),
        launch_arguments={
            'slam_params_file': slam_config_path,
            'use_sim_time': 'false' # On force pour être sûr !
        }.items()
    )

    # 4. TF : odom -> ldlidar_base (Publication continue)
    fake_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_odom',
        output='screen',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1', '--frame-id', 'odom', '--child-frame-id', 'ldlidar_base']
    )

    # 5. TF : ldlidar_base -> ldlidar_link (Publication continue)
    base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_laser',
        output='screen',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1', '--frame-id', 'ldlidar_base', '--child-frame-id', 'ldlidar_link']
    )

    # 6. Moteur
    motor_node = Node(
        package='motor_control',
        executable='motor_driver',
        name='motor_driver',
        output='screen'
    )

    # 7. Control node (Actuellement désactivé)
    control_node = Node(
        package='control_pkg',
        executable='control_node',
        name='control_node',
        output='screen'
    )

    ld = LaunchDescription()
    
    # Ajout de toutes les actions au LaunchDescription
    ld.add_action(lc_mgr_node)
    ld.add_action(ldlidar_launch)
    ld.add_action(slam_toolbox_launch) # Lancement officiel
    ld.add_action(fake_odom)
    ld.add_action(base_to_laser)
    ld.add_action(motor_node)
    
    # Pour activer le control_node plus tard, il suffira de décommenter la ligne ci-dessous :
    # ld.add_action(control_node)

    return ld