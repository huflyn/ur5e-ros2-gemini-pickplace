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
    sort_method = LaunchConfiguration('sort_method')

    # Declare the launch argument so it can be passed via terminal
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Set to true to use Webots simulation topics'
    )

    sort_method_arg = DeclareLaunchArgument(
        'sort_method',
        default_value='closest',
        description='Method to sort detected objects: "closest" (default, deterministic) or "random" (prevents endless loops)'
    )


    # Paths to the YAML parameter files
    sim_params = os.path.join(pkg_dir, 'config', 'sim_params.yaml')
    real_params = os.path.join(pkg_dir, 'config', 'real_params.yaml')
    hsv_bounds = os.path.join(pkg_dir, 'config', 'hsv_bounds.yaml')


    # Node configuration for simulation
    sim_node = Node(
        package='color_detection',
        executable='color_detector',
        name='color_detector_node',
        output='screen',
        parameters=[sim_params, hsv_bounds, {'sort_method': sort_method}],
        condition=IfCondition(use_sim)
    )

    # Node configuration for real hardware
    real_node = Node(
        package='color_detection',
        executable='color_detector',
        name='color_detector_node',
        output='screen',
        parameters=[real_params, hsv_bounds, {'sort_method': sort_method}],
        condition=UnlessCondition(use_sim)
    )


    # --- Add Terminal Logging Output ---
    # These will print the evaluated launch arguments right after the command is executed
    log_launch_info = LogInfo(
        msg=[
            '\n=========================================\n',
            'Starting Color Detector Node \n',
            '- Simulation Mode (use_sim): ', use_sim, '\n',
            '- Sorting Method: ', sort_method, '\n',
            '\nValid Arguments:\n',
            '- use_sim (default: false) - Set to true to use Webots simulation topics \n',
            '- sort_method (default: closest) - Can be set to "random" to prevent endless loops on edge cases\n',
            '========================================='
        ]
    )


    return LaunchDescription([
        use_sim_arg,
        sort_method_arg,
        log_launch_info,
        sim_node,
        real_node
    ])
