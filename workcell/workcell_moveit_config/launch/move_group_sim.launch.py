from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "workcell", package_name="workcell_moveit_config"
    ).to_moveit_configs()

    ld = generate_move_group_launch(moveit_config)

    ld.entities.insert(
        0,
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time from /clock",
        ),
    )
    ld.entities.insert(
        1,
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
    )

    return ld