import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('color_detection')

    # Create LaunchConfiguration variable
    use_sim = LaunchConfiguration('use_sim')

    # Declare the launch argument so it can be passed via terminal
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Set to true to use Webots simulation topics'
    )

    # Paths to the YAML parameter files
    sim_params = os.path.join(pkg_dir, 'config', 'sim_params.yaml')
    real_params = os.path.join(pkg_dir, 'config', 'real_params.yaml')
    hsv_bounds = os.path.join(pkg_dir, 'config', 'hsv_bounds.yaml')

    # Node configuration for simulation
    sim_node = Node(
        package='color_detection',
        executable='hsv_tuner',
        name='color_detector_node',
        output='screen',
        parameters=[sim_params, hsv_bounds],
        condition=IfCondition(use_sim)
    )

    # Node configuration for real hardware
    real_node = Node(
        package='color_detection',
        executable='hsv_tuner',
        name='color_detector_node', 
        output='screen',
        parameters=[real_params, hsv_bounds],
        condition=UnlessCondition(use_sim)
    )

    # --- Add Terminal Logging Output ---
    log_launch_info = LogInfo(
        msg=[
            '\n=========================================\n',
            'Starting HSV-Tuner Node\n',
            '- Simulation Mode (use_sim): ', use_sim, '\n',
            '\nValid Arguments:\n',
            '- use_sim (default: false) - Set to true to use Webots simulation topics \n',
            '========================================='
        ]
    )

    return LaunchDescription([
        use_sim_arg,
        log_launch_info,
        sim_node,
        real_node
    ])