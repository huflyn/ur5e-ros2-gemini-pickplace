import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('gemini_vision')

    bringup_pkg_dir = get_package_share_directory('workcell_bringup')

    # Create LaunchConfiguration variable
    use_sim_time = LaunchConfiguration('use_sim_time')
    gemini_model = LaunchConfiguration('model')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation topics and parameters'
    )

    gemini_model_arg = DeclareLaunchArgument(
        'model',
        default_value='',
        description='Override Gemini API model (leave empty to use Python script default)'
    )

    # Paths to the YAML parameter files
    sim_parameters = os.path.join(bringup_pkg_dir, 'config', 'sim_workspace_parameters.yaml')
    real_parameters = os.path.join(bringup_pkg_dir, 'config', 'real_workspace_parameters.yaml')

    # Evaluate which parameter file to load based on use_sim_time
    param_file = PythonExpression([
        "'", sim_parameters, "' if '", use_sim_time, "'.lower() == 'true' else '", real_parameters, "'"
    ])

    # Node configuration
    gemini_vision_node = Node(
        package='gemini_vision',
        executable='gemini_vision', 
        name='gemini_vision_node',
        parameters=[
            param_file,
            {
                'use_sim_time': use_sim_time,
                'gemini_model': gemini_model
            }
        ],
        output='screen'
    )

    # --- Add Terminal Logging Output ---
    log_launch_info = LogInfo(
        msg=[
            "\n" + "="*60 + "\n",
            'Gemini Vision Node Launcher\n',
            '- Simulation Time (use_sim_time): ', use_sim_time, '\n',
            '- Override Model: [', gemini_model, '] (If empty, node uses internal default)\n',
            '\nLaunch Arguments:\n',
            '- use_sim_time (default: false): Set to "true" if you use Simulation\n',
            '- model (default: empty): Override the Gemini API model\n',
            '  Available Models: gemini-3-flash-preview, gemini-3.1-flash-lite-preview, gemini-robotics-er-1.5\n',
            '  Other models may work but have not been tested\n',
            "="*60
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        gemini_model_arg,
        log_launch_info,
        gemini_vision_node
    ])