import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals

def generate_launch_description():

    # --- Arguments ---
    vision_arg = DeclareLaunchArgument(
        'vision',
        default_value='gemini',
        description='Choose the perception pipeline: "gemini", "hsv", or "legacy"'
    )
    vision_mode = LaunchConfiguration('vision')

    # --- Package Directories ---
    workcell_sim_dir = get_package_share_directory('workcell_simulation')
    workcell_app_dir = get_package_share_directory('workcell_application')
    gemini_vision_dir = get_package_share_directory('gemini_vision')
    color_detection_dir = get_package_share_directory('color_detection')

    # --- 1. Base Simulation & Visualization ---
    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(workcell_sim_dir, 'launch', 'simulation.launch.py'))
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(workcell_app_dir, 'launch', 'rviz.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # --- 2. Vision Pipeline (Conditional) ---
    gemini_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gemini_vision_dir, 'launch', 'gemini_vision.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=LaunchConfigurationEquals('vision', 'gemini')
    )

    hsv_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(color_detection_dir, 'launch', 'color_detector.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=LaunchConfigurationEquals('vision', 'hsv')
    )

    legacy_vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(color_detection_dir, 'launch', 'color_detector_legacy.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=LaunchConfigurationEquals('vision', 'legacy')
    )

    # --- 3. Application / State Machine (DELAYED) ---
    
    # Runs for both "gemini" and "hsv"
    pick_and_place_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(workcell_app_dir, 'launch', 'pick_and_place.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=LaunchConfigurationNotEquals('vision', 'legacy') 
    )

    # Runs ONLY for "legacy"
    legacy_app_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(workcell_app_dir, 'launch', 'brick_sorter_legacy.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=LaunchConfigurationEquals('vision', 'legacy')
    )

    # --- 4. Timer Actions to Stagger Application Launches ---

    delayed_application_launch_1 = TimerAction(
        period=5.0, # Adjust this delay (in seconds)
        actions=[
            LogInfo(msg=">>> Simulation launched. Launching vision applications now..."),
            gemini_launch,
            hsv_launch,
            legacy_vision_launch,
            rviz_launch
        ]
    )

    delayed_application_launch_2 = TimerAction(
        period=8.0, # Adjust this delay (in seconds)
        actions=[
            LogInfo(msg=">>> Vision applications launched. Launching Pick and Place application now..."),
            pick_and_place_launch,
            legacy_app_launch
        ]
    )

    # --- Terminal Info ---
    log_info = LogInfo(msg=["\n=== STARTING WORKCELL SIMULATION ===\nVision Mode: ", vision_mode, "\nWait until you see '🟢 PickAndPlaceNode (Client Node) ready'....\n===================================="])

    return LaunchDescription([
        vision_arg,
        log_info,
        webots_launch,
        delayed_application_launch_1,
        delayed_application_launch_2
    ])