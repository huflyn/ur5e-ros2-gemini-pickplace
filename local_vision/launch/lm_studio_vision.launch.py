import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# Keep in sync with LM_STUDIO_MODELS in lm_studio_vision.py
AVAILABLE_MODELS = [
    "google/gemma-4-e2b",        # [0]  5.95 GB, Q8
    "google/gemma-4-e4b",        # [1]  6.33 GB, Q8
    "google/gemma-4-26b-a4b",    # [2] 17.99 GB, Q4
    "qwen/qwen3.5-9b",           # [3] 10.45 GB, Q8
]


def generate_launch_description():
    bringup_pkg_dir = get_package_share_directory('workcell_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    lm_model     = LaunchConfiguration('model')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation topics and parameters'
    )
    lm_model_arg = DeclareLaunchArgument(
        'model',
        default_value='',
        description='Override LM Studio model identifier '
                    '(empty = use script default)'
    )

    sim_params  = os.path.join(
        bringup_pkg_dir, 'config', 'sim_workspace_parameters.yaml')
    real_params = os.path.join(
        bringup_pkg_dir, 'config', 'real_workspace_parameters.yaml')

    param_file = PythonExpression([
        "'", sim_params, "' if '",
        use_sim_time, "'.lower() == 'true' else '",
        real_params, "'"
    ])

    lm_vision_node = Node(
        package='local_vision',
        executable='lm_studio_vision',
        name='lm_studio_vision_node',
        parameters=[
            param_file,
            {
                'use_sim_time':    use_sim_time,
                'lm_studio_model': lm_model,
            }
        ],
        output='screen',
    )

    models_str = '\n'.join(
        f'  [{i}] {m}' for i, m in enumerate(AVAILABLE_MODELS))

    log_info = LogInfo(msg=[
        "\n" + "=" * 60 + "\n",
        "LM Studio Vision Node Launcher\n",
        "- Simulation Time : ", use_sim_time, "\n",
        "- Model override  : [", lm_model, "] "
        "(empty → script default)\n",
        "\nLaunch arguments:\n",
        "  use_sim_time  (default: false)\n",
        "  model         (default: empty)\n",
        f"\nAvailable models:\n{models_str}\n",
        "\nExamples:\n",
        "  ros2 launch local_vision lm_studio_vision.launch.py\n",
        "  ros2 launch local_vision lm_studio_vision.launch.py use_sim_time:=true\n",
        "  ros2 launch local_vision lm_studio_vision.launch.py model:=2\n",
        "  ros2 launch local_vision lm_studio_vision.launch.py model:=qwen/qwen3.5-9b\n",
        "=" * 60,
    ])

    return LaunchDescription([
        use_sim_time_arg,
        lm_model_arg,
        log_info,
        lm_vision_node,
    ])