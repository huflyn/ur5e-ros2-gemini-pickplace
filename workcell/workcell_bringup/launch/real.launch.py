import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():

    # --- Arguments ---

    vision_arg = DeclareLaunchArgument(
        'vision',
        default_value='gemini',
        description='Choose the perception pipeline: "gemini" or "hsv"'
    )
    vision_mode = LaunchConfiguration('vision')

    # --- Package Directories ---
    realsense_dir = get_package_share_directory('realsense2_camera')
    workcell_ctrl_dir = get_package_share_directory('workcell_control')
    workcell_app_dir = get_package_share_directory('workcell_application')
    gemini_vision_dir = get_package_share_directory('gemini_vision')
    color_detection_dir = get_package_share_directory('color_detection')

    # --- 1. Hardware Drivers ---
    
    # RealSense Camera
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_dir, 'launch', 'rs_launch.py')),
        launch_arguments={
            'depth_module.depth_profile': '1280x720x6',
            'rgb_camera.color_profile': '1280x720x6',
            'camera_name': 'd415',
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'spatial_filter.enable': 'true',
            'pointcloud.enable': 'false'
        }.items()
    )

    # UR5e Robot Driver
    robot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(workcell_ctrl_dir, 'launch', 'start_robot.launch.py')),
    )

    # Automated Play Trigger for Teach Pendant Robot Program (Wait 10 seconds for driver to boot)
    play_program_cmd = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg=">>> Triggering /dashboard_client/play to start robot program..."),
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/dashboard_client/play', 'std_srvs/srv/Trigger'],
                output='screen'
            )
        ]
    )

    # --- 2. Vision Pipeline (Conditional) ---
    
    gemini_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gemini_vision_dir, 'launch', 'gemini_vision.launch.py')),
        condition=LaunchConfigurationEquals('vision', 'gemini')
    )

    hsv_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(color_detection_dir, 'launch', 'color_detector.launch.py')),
        condition=LaunchConfigurationEquals('vision', 'hsv')
    )

    # --- 3. Application & Visualization ---

    pick_and_place_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(workcell_app_dir, 'launch', 'pick_and_place.launch.py'))
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(workcell_app_dir, 'launch', 'rviz.launch.py'))
    )

    # --- 4. TimerActions to Stagger Application Launches ---

    delayed_application_launch_1 = TimerAction(
        period=5.0, # Adjust this delay (in seconds)
        actions=[
            LogInfo(msg=">>> Robot Driver launched. Launching vision applications now..."),
            gemini_launch,
            hsv_launch,
            rviz_launch
        ]
    )

    delayed_application_launch_2 = TimerAction(
        period=10.0, # Adjust this delay (in seconds)
        actions=[
            LogInfo(msg=">>> Vision applications launched. Launching Pick and Place application now..."),
            pick_and_place_launch,
        ]
    )

    # --- Terminal Info ---
    log_info = LogInfo(msg=["\n=== STARTING REAL HARDWARE WORKCELL ===\nVision Mode: ", vision_mode, "\n======================================="])

    return LaunchDescription([
        vision_arg,
        log_info,
        realsense_launch,
        robot_driver_launch,
        play_program_cmd,
        delayed_application_launch_1,
        delayed_application_launch_2
    ])