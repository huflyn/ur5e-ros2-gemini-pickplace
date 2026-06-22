import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.actions import DeclareLaunchArgument
import xacro
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('workcell_simulation')
    workcell_desc_dir = get_package_share_directory('workcell_description')
    ros2_controllers_yaml_dir = get_package_share_directory('workcell_moveit_config')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world_config = LaunchConfiguration('world')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Webots) clock if true, hardware clock if false'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='workcell.wbt',
        description='Name of the Webots world file to load from the worlds directory'
    )

    world_path = PathJoinSubstitution([
        package_dir,
        'worlds',
        world_config
    ])

    # ===== 1) URDFs verarbeiten (xacro → XML-String) =====

    # Workcell URDF (komplett, mit ros2_control)
    workcell_xacro = os.path.join(workcell_desc_dir, 'urdf', 'workcell.urdf.xacro')
    workcell_description = xacro.process_file(workcell_xacro).toxml()

    # UR5e Webots URDF (für WebotsController) und ros2_controllers.yaml
    ur5e_xacro = os.path.join(package_dir, 'config', 'ur5e_webots.urdf.xacro')
    ur5e_control_params = os.path.join(ros2_controllers_yaml_dir, 'config', 'ros2_controllers.yaml')

    # Realsense Webots URDF (für WebotsController)
    realsense_xacro = os.path.join(package_dir, 'config', 'realsense_d415_webots.urdf.xacro')


    # ===== 2) Nodes =====

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': workcell_description}, {'use_sim_time': use_sim_time}],
    )

    # Spawner
    controller_manager_timeout = ['--controller-manager-timeout', '100']

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'] + controller_manager_timeout,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    trajectory_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scaled_joint_trajectory_controller', '--controller-manager', '/controller_manager'] + controller_manager_timeout,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Start Webots using the dynamic world path
    webots = WebotsLauncher(
        world=world_path,
        ros2_supervisor=True
    )

    # UR5e Controller
    ur5e_driver = WebotsController(
        robot_name='ur5e',
        parameters=[
            {'robot_description': ur5e_xacro},
            {'use_sim_time': use_sim_time},
            ur5e_control_params,
        ],
    )

    # Realsense Controller
    realsense_driver = WebotsController(
        robot_name='realsense',
        parameters=[
            {'robot_description': realsense_xacro},
            {'use_sim_time': use_sim_time},
        ],
    )


    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        robot_state_publisher,
        webots,
        webots._supervisor,
        ur5e_driver,
        realsense_driver,
        joint_state_spawner,
        trajectory_spawner,

        # Shutdown Handler
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])