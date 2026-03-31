import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('color_detection')
    bringup_pkg_dir = get_package_share_directory('workcell_bringup')


    # Create LaunchConfiguration variable
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch argument using the ROS standard naming convention
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Set to true to use Webots simulation topics and clock'
    )

    
    # Paths to the YAML parameter files
    sim_parameters = os.path.join(bringup_pkg_dir, 'config', 'sim_workspace_parameters.yaml')
    real_parameters = os.path.join(bringup_pkg_dir, 'config', 'real_workspace_parameters.yaml')

    # Evaluate which parameter file to load based on use_sim_time
    param_file = PythonExpression([
        "'", sim_parameters, "' if '", use_sim_time, "'.lower() == 'true' else '", real_parameters, "'"
    ])

    # Node configuration
    hsv_tuner_node = Node(
        package='color_detection',
        executable='hsv_tuner',
        name='hsv_tuner_node', # Keeps the namespace so YAMLs map correctly
        output='screen',
        parameters=[
            param_file, 
            {'use_sim_time': use_sim_time} # Crucial: pass the simulation time flag
        ]
    )

    # --- Add Terminal Logging Output ---
    log_launch_info = LogInfo(
        msg=[
            '\n=========================================\n',
            'Starting HSV-Tuner Node\n',
            '- Simulation Time (use_sim_time): ', use_sim_time, '\n',
            '\nValid Arguments:\n',
            '- use_sim_time (default: false) - Set to true to use Webots simulation topics \n',
            '========================================='
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        log_launch_info,
        hsv_tuner_node
    ])