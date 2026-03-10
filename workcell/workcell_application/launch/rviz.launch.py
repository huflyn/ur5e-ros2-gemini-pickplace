import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Define the package name
    pkg_name = 'workcell_application'
    
    # Get the path to the package share directory
    pkg_share = get_package_share_directory(pkg_name)
    
    # Define the path to the saved RViz configuration file
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'workcell.rviz')

    # Setup the use_sim_time launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch argument to make it accessible via CLI
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation topics and parameters'
    )

    # Define the RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    # --- Add Terminal Logging Output ---
    log_launch_info = LogInfo(
        msg=[
            "\n" + "="*60 + "\n",
            'RViz2 Node \n',
            '- Simulation Time (use_sim_time): ', use_sim_time, '\n',
            '\nLaunch Arguments:\n',
            '- use_sim_time (default: false): Set to "true" if you use Simulation\n',
            "="*60
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_node
    ])