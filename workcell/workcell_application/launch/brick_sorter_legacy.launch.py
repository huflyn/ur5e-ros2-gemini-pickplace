import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
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
                get_package_share_directory("workcell_control"),
                "config",
                "planning_parameters.yaml"
            )
        )
        .to_moveit_configs()
    )

    # 2. parameters YAML with pick and place specific parameters (e.g. drop-off coordinates)
    bringup_pkg_dir = get_package_share_directory('workcell_bringup')
    # Paths to the YAML parameter files
    sim_parameters = os.path.join(bringup_pkg_dir, 'config', 'sim_workspace_parameters.yaml')
    real_parameters = os.path.join(bringup_pkg_dir, 'config', 'real_workspace_parameters.yaml')
    # Evaluate which parameter file to load based on use_sim_time
    param_file = PythonExpression([
        "'", sim_parameters, "' if '", use_sim_time, "'.lower() == 'true' else '", real_parameters, "'"
    ])

    # 3. Define the main sorting node
    pick_and_place_node = Node(
        package="workcell_application",
        executable="brick_sorter_legacy",
        name="pick_and_place_node",
        parameters=[
            moveit_config.to_dict(),      # Loads URDF, SRDF, Kinematics, etc.
            param_file,                           # Loads the appropriate workspace parameters
            {"use_sim_time": use_sim_time}   # Important to sync with Webots clock
        ],
        output="screen",
    )

    return LaunchDescription([use_sim_time_arg, pick_and_place_node])