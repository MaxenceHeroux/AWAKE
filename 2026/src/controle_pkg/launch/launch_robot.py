# Copyright 2024 Walter Lucetti
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###########################################################################

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    
    node_name = LaunchConfiguration('node_name', default='ldlidar_node')

    # Chemin vers ton fichier de configuration SLAM Toolbox dédié
    slam_config_path = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params',
        'slam_toolbox.yaml'
    )

    # Lifecycle manager : Configure et Active automatiquement le Lidar et le SLAM
# Lifecycle manager : Configure et Active automatiquement le Lidar et le SLAM
    lc_mgr_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['ldlidar_node', 'slam_toolbox'],
            'bond_timeout': 0.0  # <--- AJOUTE CETTE LIGNE : Désactive le timeout des heartbeats
        }]
    )

    # Nœud SLAM Toolbox (Restauration des paramètres via le fichier YAML original)
    slam_toolbox_node = LifecycleNode(
          package='slam_toolbox',
          executable='async_slam_toolbox_node',
          namespace='',
          name='slam_toolbox',
          output='screen',
          parameters=[
              slam_config_path  # <-- Utilise à nouveau ton fichier .yaml dédié
          ],
          remappings=[
              ('/scan', '/ldlidar_node/scan')
          ]          
    )

    # Inclusion du Bringup du Lidar
    ldlidar_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('ldlidar_node'),
            '/launch/ldlidar_bringup.launch.py'
        ]),
        launch_arguments={
            'node_name': node_name
        }.items()
    )

    # TF Statique : Liaison odom -> ldlidar_base
    fake_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'ldlidar_base']
    )

    # Le Cerveau (Python) : Exploration PID / Mode Course Pure Pursuit Ackermann
    control_strategy_node = Node(
        package='control_pkg',
        executable='control_node',
        name='control_node',
        output='screen'
    )

    # L'Actuateur (C++) : Traduction des messages Ackermann en signaux PWM via WiringPi
    hardware_pwm_node = Node(
        package='controle_pkg', 
        executable='controle_node',
        name='controle_node',
        output='screen'
    )

    # Assemblage de la description de lancement
    ld = LaunchDescription()

    ld.add_action(lc_mgr_node)
    ld.add_action(slam_toolbox_node)
    ld.add_action(fake_odom)
    ld.add_action(ldlidar_launch)
    ld.add_action(control_strategy_node)
    ld.add_action(hardware_pwm_node)

    return ld