import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'workcell_application'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Define paths to both RViz configuration files
    rviz_sim_file = os.path.join(pkg_share, 'rviz', 'workcell_sim.rviz')
    rviz_real_file = os.path.join(pkg_share, 'rviz', 'workcell_real.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    # --- SIMULATION NODE ---
    # Loads workcell_sim.rviz when use_sim_time:=true
    rviz_node_sim = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_sim_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_sim_time)
    )

    # --- REAL HARDWARE NODE ---
    # Loads workcell_real.rviz when use_sim_time:=false (default)
    rviz_node_real = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_real_file],
        parameters=[{'use_sim_time': False}],
        condition=UnlessCondition(use_sim_time)
    )

    # --- TERMINAL LOGGING ---
    log_launch_info = LogInfo(
        msg=[
            "\n" + "="*60 + "\n",
            'RViz2 Node Initialized\n',
            '- Loading workcell_sim.rviz if use_sim_time:=true\n',
            '- Loading workcell_real.rviz if use_sim_time:=false\n',
            "="*60
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_node_sim,
        rviz_node_real,
        log_launch_info
    ])