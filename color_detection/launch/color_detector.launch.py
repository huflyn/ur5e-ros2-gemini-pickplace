import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('color_detection')

    # Create LaunchConfiguration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch argument using the ROS standard naming convention
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Set to true to use Webots simulation topics and clock'
    )

    # Paths to the YAML parameter files
    sim_params = os.path.join(pkg_dir, 'config', 'sim_params.yaml')
    real_params = os.path.join(pkg_dir, 'config', 'real_params.yaml')
    hsv_bounds = os.path.join(pkg_dir, 'config', 'hsv_bounds.yaml')

    # Evaluate which parameter file to load based on use_sim_time
    param_file = PythonExpression([
        "'", sim_params, "' if '", use_sim_time, "'.lower() == 'true' else '", real_params, "'"
    ])

    # Node configuration
    color_detector_node = Node(
        package='color_detection',
        executable='color_detector',
        name='color_detector_node',
        output='screen',
        parameters=[
            param_file, 
            hsv_bounds, 
            {
                'use_sim_time': use_sim_time,
            }
        ],
    )

    # --- Add Terminal Logging Output ---
    log_launch_info = LogInfo(
        msg=[
            "\n" + "="*60 + "\n",
            'Starting Color Detector Node \n',
            '- Simulation Time (use_sim_time): ', use_sim_time, '\n',
            '\nLaunch Arguments:\n',
            '- use_sim_time (default: false): Set to true to use Simulation Parameters\n',
            "="*60
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        log_launch_info,
        color_detector_node,
    ])