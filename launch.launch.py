from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. On prépare la commande pour passer sudo en gardant l'environnement ROS
    # C'est nécessaire pour que le node trouve les librairies ROS même en root
    sudo_prefix = [
        "sudo -E env PATH=", os.environ['PATH'],
        " LD_LIBRARY_PATH=", os.environ.get('LD_LIBRARY_PATH', ''),
        " PYTHONPATH=", os.environ.get('PYTHONPATH', ''),
        " "
    ]

    # Path to the lidar launch file
    ldlidar_launch_path = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'launch',
        'ldlidar_rviz2.launch.py'
    )

    return LaunchDescription([
        # Launch lidar + RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ldlidar_launch_path)
        ),

        # Launch scan_to_command_node (Python)
        Node(
            package='scan_to_command_pkg',
            executable='scan_to_command_node',
            name='scan_to_command_node',
            output='screen'
        ),

        # Launch controle_node (C++) AVEC SUDO
        # C'est le seul qui a besoin des droits root pour wiringPi
        Node(
            package='controle_pkg',
            executable='controle_node',
            name='controle_node',
            output='screen',
        )
    ])