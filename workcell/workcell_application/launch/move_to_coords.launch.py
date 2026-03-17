import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # Declare Launch Arguments
    named_pose_arg = DeclareLaunchArgument('named_pose', default_value='', description='Named pose to move to (e.g. "ready").')
    coords_arg = DeclareLaunchArgument('coords', default_value='', description='XYZ string, e.g. "0 0.5 0.1" or "[0, 0.5, 0.1]".')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Target Yaw rotation in degrees')
    use_sim_time = LaunchConfiguration('use_sim_time')


    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Webots) clock if true'
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

    move_to_coords_node = Node(
        package='workcell_application',
        executable='move_to_coords',
        name='move_to_coords_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'use_sim_time': use_sim_time,
                'named_pose': LaunchConfiguration('named_pose'),
                'coords': LaunchConfiguration('coords'),
                'yaw': LaunchConfiguration('yaw')
            }
        ]
    )

    return LaunchDescription([
        named_pose_arg,
        coords_arg,
        yaw_arg,
        use_sim_time_arg,
        move_to_coords_node
    ])