import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Webots) clock if true, hardware clock if false'
    )
    
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="ur5e", package_name="workcell_moveit_config"
        )
        .robot_description(file_path="config/workcell.urdf.xacro")
        .moveit_cpp(
            file_path=get_package_share_directory("workcell_application")
            + "/config/planning_parameters.yaml"
        )
        .to_moveit_configs()
    )

    # Python Test-Node starten
    test_node = Node(
        package="workcell_application",
        executable="test_moveit_api",
        name="moveit_py_test_node",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time}
        ],
        output="screen",
    )

    return LaunchDescription([use_sim_time_arg, test_node])