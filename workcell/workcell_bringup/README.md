# Workcell Bringup Package (`workcell_bringup`) <!-- omit from toc -->

[![jazzy][jazzy-badge]][jazzy-link]
[![ubuntu24][ubuntu24-badge]][ubuntu24-link]
[![gemini][gemini-badge]][gemini-link]

[jazzy-badge]: https://img.shields.io/badge/-ROS%202%20JAZZY-orange?style=flat-square&logo=ros&logoColor=white
[jazzy-link]: https://docs.ros.org/en/jazzy/index.html
[ubuntu24-badge]: https://img.shields.io/badge/-UBUNTU%2024%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu24-link]: https://releases.ubuntu.com/noble/
[gemini-badge]: https://img.shields.io/badge/-GEMINI%20API-7C4DFF?style=flat-square&logo=googlegemini&logoColor=white
[gemini-link]: https://ai.google.dev/gemini-api/docs

This package acts as the centralized "glue" for the entire robotic workspace. It contains the global hardware configurations and the Master Launch Files, allowing you to bring up the complete perception-to-action pipeline (drivers, vision, and application) with a single command.

---

- [I) Package Structure](#i-package-structure)
- [II) Workspace Configuration (YAML)](#ii-workspace-configuration-yaml)
- [III) Master Launch Files](#iii-master-launch-files)
  - [Simulation (`sim.launch.py`)](#simulation-simlaunchpy)
    - [Option A: Gemini Vision (Default)](#option-a-gemini-vision-default)
    - [Option B: OpenCV HSV Masking](#option-b-opencv-hsv-masking)
    - [Option C: Legacy (ROS 1 Port)](#option-c-legacy-ros-1-port)
  - [Real Hardware (`real.launch.py`)](#real-hardware-reallaunchpy)
    - [Option A: Gemini Vision (Default)](#option-a-gemini-vision-default-1)
    - [Option B: OpenCV HSV Masking](#option-b-opencv-hsv-masking-1)
- [IV) Triggering the Application](#iv-triggering-the-application)
  - [Test the vision system (no robot movement)](#test-the-vision-system-no-robot-movement)
  - [Trigger the full Pick-and-Place cycle](#trigger-the-full-pick-and-place-cycle)

---

# I) Package Structure

* **`launch/`**: Contains the master launch files (`sim.launch.py` and `real.launch.py`) that orchestrate the startup sequence using staggered timers.
* **`config/`**: Contains the centralized `sim_workspace_parameters.yaml` and `real_workspace_parameters.yaml` files.

---

# II) Workspace Configuration (YAML)

To prevent misconfigurations across multiple nodes, all environment-specific variables are stored centrally in this package. 

Nodes like `gemini_vision`, `color_detection`, and `workcell_application` automatically pull their parameters from these files depending on whether the system is launched in simulation or real hardware mode.

Example snippet (`sim_workspace_parameters.yaml`):

```yaml
/**:
  ros__parameters:
    # --- Safe Workspace Boundaries ---
    # Set to 'true' to strictly enforce table boundaries (recommended for real hardware)
    enable_workspace_safety: true
    # Physical dimensions of the table (in meters relative to robot base), check robot_base_frame TF in RViz for orientation
    workspace_min_x: -0.325
    workspace_max_x:  0.325
    workspace_min_y: -0.24
    workspace_max_y:  0.76
    # Allowed tolerance outside the table (e.g. for dropping items off the edge)
    workspace_safety_tolerance: 0.10

    # --- Camera Topics ---
    camera_info_topic: '/webots_realsense/depth/image_rect_raw/camera_info'
    color_image_topic: '/webots_realsense/color/image_raw/image_color'
    depth_image_topic: '/webots_realsense/depth/image_rect_raw/image'

    # --- TF Frames ---
    # Frame of the camera for TF transformations
    camera_frame: 'd415_sim_optical_frame'
    # Target frame for the 3D coordinates
    robot_base_frame: 'ur5e_base_link'
```

---

# III) Master Launch Files

Instead of opening multiple terminals to start hardware, vision, and the application separately, you can use these master launch files.

> [!NOTE]
> **Staggered Startup:** To ensure all controllers and drivers are fully loaded before the robot attempts its initial homing sequence, these launch files use delayed timers. **Please wait roughly 10-15 seconds** after launching for everything to boot up completely\!

> [!IMPORTANT]
> **API Key Required:** Because the Gemini Vision pipeline is the default mode, ensure you have exported your `GEMINI_API_KEY` in your terminal before running these launch files. **Instructions:** [GEMINI_API.md](/GEMINI_API.md)

## Simulation (`sim.launch.py`)

This file launches Webots, the selected vision system, RViz, and the MoveIt Pick-and-Place orchestrator. It automatically applies `use_sim_time:=true` to all sub-nodes.

### Option A: Gemini Vision (Default)

```bash
ros2 launch workcell_bringup sim.launch.py
```

### Option B: OpenCV HSV Masking

```bash
ros2 launch workcell_bringup sim.launch.py vision:=hsv
```

### Option C: Legacy (ROS 1 Port)

```bash
ros2 launch workcell_bringup sim.launch.py vision:=legacy
```

## Real Hardware (`real.launch.py`)

This file connects to the physical UR5e, starts the RealSense camera stream, triggers the robot program remotely via the dashboard client, and launches the selected vision and application nodes.

> [!CAUTION]
> Follow all safety precautions when working with real robots. Keep the emergency stop button within reach.

> [!WARNING]
> **Hardware Specificity:** This project and its configurations are strictly designed and tested for the **UR5e**. Attempting to use this workspace with other UR models will cause issues.

> [!IMPORTANT]
> Ensure the robot is powered on and the **external control** program is loaded on the teach pendant. The launch file will attempt to automatically "play" the program via the dashboard client.

### Option A: Gemini Vision (Default)

```bash
# Replace <ROBOT_IP_ADDRESS> with the actual IP, can also be set in the launch file in the workcell_control package
ros2 launch workcell_bringup real.launch.py robot_ip:=<YOUR_ROBOT_IP>
```

### Option B: OpenCV HSV Masking

```bash
# Replace <ROBOT_IP_ADDRESS> with the actual IP, can also be set in the launch file in the workcell_control package
ros2 launch workcell_bringup real.launch.py robot_ip:=<YOUR_ROBOT_IP> vision:=hsv
```

---

# IV) Triggering the Application

Once the master launch file has finished its sequence, the robot will move to its `ready` pose and wait in STANDBY mode. 

> [!NOTE]
> If you launched the system in **`legacy`** vision mode, the robot will not wait for a manual trigger. It will automatically start the pick-and-place cycle as soon as bricks appear in the camera's view and match the configured HSV color bounds.

For the `gemini` and `hsv` modes, open a **new terminal** to interact with the system. You can trigger actions using standard ROS 2 CLI commands or the convenient bash shortcuts (if you configured them as shown in the **[Main README](/README.md#v-workflow-tips-bash-shortcuts)**).

## Test the vision system (no robot movement)

This calls the active vision service directly to test the object detection and target calculation without moving the physical or simulated arm.

**Using standard ROS 2 commands:**

```bash
# Default call
ros2 service call /detect_objects object_interfaces/srv/DetectObjects

# Custom natural language instruction (Gemini Vision only)
ros2 service call /detect_objects object_interfaces/srv/DetectObjects "{user_prompt: 'Pick the red and blue bricks'}"
```

**Using bash shortcuts:**

```bash
testscan
testscan Pick the red bricks
```

## Trigger the full Pick-and-Place cycle

This publishes to the trigger topic to start the actual physical or simulated pick-and-place cycle.

**Using standard ROS 2 commands:**

```bash
# Default sorting
ros2 topic pub --once /pick_and_place/scan std_msgs/msg/String "{data: ''}"

# Custom natural language instruction (Gemini Vision only)
ros2 topic pub --once /pick_and_place/scan std_msgs/msg/String "{data: 'Pick the red and blue bricks.'}"
```

**Using bash shortcuts:**

```bash
scan
scan Pick the red and blue bricks
```

---