import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_name = 'workcell_webots'
    package_dir = get_package_share_directory(package_name)
    
    # ---------------------------------------------------------
    # 1. PFADE DEFINIEREN
    # ---------------------------------------------------------
    ur5e_urdf_path = os.path.join(package_dir, 'config', 'ur5e_webots.urdf.xacro')
    ur5e_control_params = os.path.join(package_dir, 'config', 'ros2_controllers.yaml')
    realsense_urdf_path = os.path.join(package_dir, 'config', 'realsense_d415_webots.urdf.xacro')

    ur5e_robot_description = ParameterValue(
        Command(["xacro ", ur5e_urdf_path]),
        value_type=str,
    )

    workcell_description_package = get_package_share_directory('workcell_description')
    workcell_description_path = os.path.join(workcell_description_package, 'urdf', 'workcell.urdf.xacro')
    workcell_robot_description = ParameterValue(
        Command(["xacro ", workcell_description_path]),
        value_type=str,
    )

    # ---------------------------------------------------------
    # 2. ARGUMENTE
    # ---------------------------------------------------------
    world_handler = DeclareLaunchArgument(
        'world',
        default_value='workcell.wbt', 
        description='Name der Welt-Datei in worlds/'
    )

    # ---------------------------------------------------------
    # 3. WEBOTS STARTEN
    # ---------------------------------------------------------
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', LaunchConfiguration('world')]),
        ros2_supervisor=True,
        mode='realtime'
    )

    # ---------------------------------------------------------
    # 4. ROBOTER 1: UR5e (Arm)
    # ---------------------------------------------------------
    ur5e_driver = WebotsController(
        robot_name='ur5e', 
        parameters=[
            {'robot_description': ur5e_urdf_path}, # Hier übergeben wir die URDF des Arms, damit der Controller die Gelenke kennt. Die statischen TFs kommen aus der Workcell-URDF, die wir im State Publisher verwenden.
            {'use_sim_time': True},
            {'set_robot_state_publisher': False}, # False, da wir den robot_state_publisher manuell starten, damit er die komplette Workcell-URDF bekommt und nicht nur die Arm-URDF.
            ur5e_control_params
        ]
    )

    ur5e_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': workcell_robot_description, # Hier verwenden wir die komplette Workcell-URDF, damit auch die statischen TFs korrekt veröffentlicht werden
            'use_sim_time': True
        }],
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

    # ---------------------------------------------------------
    # 5. ROBOTER 2: RealSense (Kamera)
    # ---------------------------------------------------------
    # HIER WAR DER FEHLER: Dieser Block muss EINZIGARTIG sein.
    realsense_driver = WebotsController(
        robot_name='realsense',
        parameters=[
            {'robot_description': realsense_urdf_path}, # Hier übergeben wir die URDF der RealSense, damit der Controller die Kamera-Frames kennt. Die statischen TFs kommen aus der Workcell-URDF, die wir im State Publisher verwenden.
            {'set_robot_state_publisher': False},
            {'use_sim_time': True}
        ]
    )


    return LaunchDescription([
        world_handler,
        webots,
        webots._supervisor,
        
        # UR5e Nodes
        ur5e_driver,
        ur5e_state_publisher,
        trajectory_spawner,
        joint_state_spawner,

        # RealSense Nodes
        realsense_driver,

        # Shutdown Handler
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])