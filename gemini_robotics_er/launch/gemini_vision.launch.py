import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('gemini_robotics_er')

    # Create LaunchConfiguration variable
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation topics and parameters'
    )

    # Paths to the YAML parameter files
    sim_params = os.path.join(pkg_dir, 'config', 'sim_params.yaml')
    real_params = os.path.join(pkg_dir, 'config', 'real_params.yaml')

    # Evaluate which parameter file to load based on use_sim_time
    param_file = PythonExpression([
        "'", sim_params, "' if '", use_sim_time, "'.lower() == 'true' else '", real_params, "'"
    ])

    gemini_vision_node = Node(
        package='gemini_robotics_er',
        executable='gemini_vision', 
        name='gemini_vision_node',
        parameters=[
            param_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        gemini_vision_node
    ])