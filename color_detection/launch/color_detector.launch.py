import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('color_detection')

    # Argument für das Terminal deklarieren (Standard ist false = echte Kamera)
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Set to true to use Webots simulation topics'
    )

    use_sim = LaunchConfiguration('use_sim')

    # Pfade zu den YAML Dateien
    sim_params = os.path.join(pkg_dir, 'config', 'sim_topics.yaml')
    real_params = os.path.join(pkg_dir, 'config', 'real_topics.yaml')

    # Node-Konfiguration für Simulation
    sim_node = Node(
        package='color_detection',
        executable='color_detector',
        name='color_detector_node',
        output='screen',
        parameters=[sim_params],
        condition=IfCondition(use_sim)
    )

    # Node-Konfiguration für reale Hardware
    real_node = Node(
        package='color_detection',
        executable='color_detector',
        name='color_detector_node',
        output='screen',
        parameters=[real_params],
        condition=UnlessCondition(use_sim)
    )

    return LaunchDescription([
        use_sim_arg,
        sim_node,
        real_node
    ])