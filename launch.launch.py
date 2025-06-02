from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Chemin vers le launch file du lidar
    ldlidar_launch_path = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'launch',
        'ldlidar_rviz2.launch.py'
    )

    return LaunchDescription([
        # Lancement du lidar + RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ldlidar_launch_path)
        ),

        # Lancement du node scan_to_command_node (Python)
        Node(
            package='scan_to_command_pkg',
            executable='scan_to_command_node',
            name='scan_to_command_node',
            output='screen'
        ),

        # Lancement du node controle_node (C++)
        Node(
            package='controle_pkg',
            executable='controle_node',
            name='controle_node',
            output='screen'
        )
    ])
