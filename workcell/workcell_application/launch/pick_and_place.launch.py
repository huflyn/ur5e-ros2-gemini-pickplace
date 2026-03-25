import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Webots) clock if true, hardware clock if false'
    )

    # 1. Load the MoveIt configuration (Crucial for MoveItPy!)
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ur5e", package_name="workcell_moveit_config"
        )
        .robot_description(file_path="config/workcell.urdf.xacro")
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("workcell_application"),
                "config",
                "planning_parameters.yaml"
            )
        )
        .to_moveit_configs()
    )

    # 2. Path to your custom drop-off parameters YAML
    pick_and_place_params_file = os.path.join(
        get_package_share_directory('workcell_application'),
        'config',
        'pick_and_place_parameters.yaml'
    )

    # 3. Define the main sorting node
    pick_and_place_node = Node(
        package="workcell_application",
        executable="pick_and_place",
        name="pick_and_place_node",
        parameters=[
            moveit_config.to_dict(),      # Loads URDF, SRDF, Kinematics, etc.
            pick_and_place_params_file,           # Loads your custom drop-off coordinates
            {
                "use_sim_time": use_sim_time,
                "use_sim_gripper": use_sim_time,  # Use sim gripper if using sim time (Webots)
            }
        ],
        output="screen",
    )

    # --- Add Terminal Logging Output ---
    log_launch_info = LogInfo(
        msg=[
            "\n" + "="*60 + "\n",
            'Pick and Place Node \n',
            '- Simulation Time (use_sim_time): ', use_sim_time, '\n',
            '\nLaunch Arguments:\n',
            '- use_sim_time (default: false): Set to true to use Simulation Parameters\n',
            '\nTo manually stop the Pick and Place process execute in new terminal:\n',
            'ros2 topic pub --once /stop std_msgs/msg/Empty\n',
            "="*60
        ]
    )

    return LaunchDescription([use_sim_time_arg, log_launch_info, pick_and_place_node])