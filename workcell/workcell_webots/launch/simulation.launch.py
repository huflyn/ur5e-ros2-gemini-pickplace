import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
import xacro
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('workcell_webots')
    workcell_desc_dir = get_package_share_directory('workcell_description')
    ros2_controllers_yaml_dir = get_package_share_directory('workcell_moveit_config')

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

    set_sim_time = SetParameter(name='use_sim_time', value=True)

    # TF-Tree + /robot_description Topic (mit ros2_control in workcell.urdf.xacro definiert)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': workcell_description}],
    )

    # Spawner
    controller_manager_timeout = ['--controller-manager-timeout', '100']
    spawner_params = [{'use_sim_time': True}]

    trajectory_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scaled_joint_trajectory_controller', '--controller-manager', '/controller_manager'] + controller_manager_timeout,
        parameters=spawner_params,
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'] + controller_manager_timeout,
        parameters=spawner_params,
    )

    # Webots starten
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'workcell.wbt')
    )

    # UR5e Controller – XML-String, NICHT Dateipfad!
    ur5e_driver = WebotsController(
        robot_name='ur5e',
        parameters=[
            {'robot_description': ur5e_xacro},
            ur5e_control_params
        ],
    )

    # Realsense Controller – XML-String, NICHT Dateipfad!
    realsense_driver = WebotsController(
        robot_name='realsense',
        parameters=[{'robot_description': realsense_xacro}],
    )


    return LaunchDescription([

        set_sim_time,        
        robot_state_publisher,
        webots,
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