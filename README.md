
# AI-Driven Pick-and-Place with Google Gemini and Universal Robots UR5e in ROS 2 Jazzy <!-- omit from toc -->

[![jazzy][jazzy-badge]][jazzy-link]
[![ubuntu24][ubuntu24-badge]][ubuntu24-link]
[![gemini][gemini-badge]][gemini-link]

[jazzy-badge]: https://img.shields.io/badge/-ROS%202%20JAZZY-orange?style=flat-square&logo=ros&logoColor=white
[jazzy-link]: https://docs.ros.org/en/jazzy/index.html
[ubuntu24-badge]: https://img.shields.io/badge/-UBUNTU%2024%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu24-link]: https://releases.ubuntu.com/noble/
[gemini-badge]: https://img.shields.io/badge/-GEMINI%20API-7C4DFF?style=flat-square&logo=googlegemini&logoColor=white
[gemini-link]: https://ai.google.dev/gemini-api/docs

This repository demonstrates how to integrate state-of-the-art vision-language models (like Google Gemini Robotics-ER 1.5 and Gemini 3 Flash) with a Universal Robots UR5e manipulator in a ROS 2 Jazzy environment. 

The project features a complete perception-to-action pipeline. It utilizes the Gemini API for advanced spatial reasoning and natural language processing, and MoveIt 2 for hybrid motion planning (OMPL + Pilz LIN/PTP).

https://github.com/user-attachments/assets/ba4a9620-bd2b-4814-b3d8-9d6d37d2f78e

---

- [I) Prerequisites](#i-prerequisites)
- [II) Installation and Setup](#ii-installation-and-setup)
  - [ROS 2 Packages and Tools](#ros-2-packages-and-tools)
  - [Google Gemini API and SDK](#google-gemini-api-and-sdk)
- [III) Workspace Overview](#iii-workspace-overview)
- [IV) Workflow Tips (Bash Shortcuts)](#iv-workflow-tips-bash-shortcuts)
- [V) Quick Start: Pick-and-Place with Gemini Vision](#v-quick-start-pick-and-place-with-gemini-vision)
  - [Step 1: Start the Robot and Camera (Real or Simulated)](#step-1-start-the-robot-and-camera-real-or-simulated)
    - [Option A: Simulation (Webots)](#option-a-simulation-webots)
    - [Option B: Real Hardware (UR5e \& RealSense)](#option-b-real-hardware-ur5e--realsense)
  - [Step 2: Start Gemini Vision \& RViz](#step-2-start-gemini-vision--rviz)
  - [Step 3: Start Pick-and-Place Application](#step-3-start-pick-and-place-application)
    - [Option A: Simulation (Webots)](#option-a-simulation-webots-1)
    - [Option B: Real Hardware (UR5e \& RealSense)](#option-b-real-hardware-ur5e--realsense-1)
  - [Step 4: Trigger Pick-and-Place Cycle](#step-4-trigger-pick-and-place-cycle)
    - [Trigger Option A: Default Mode](#trigger-option-a-default-mode)
    - [Trigger Option B: Custom Prompt Mode - Gemini ONLY](#trigger-option-b-custom-prompt-mode---gemini-only)
- [VI) Hardware Testing Tools](#vi-hardware-testing-tools)
- [VII) Documentation and References](#vii-documentation-and-references)

---

# I) Prerequisites

**Software:**
- [Ubuntu 24.04 LTS](https://releases.ubuntu.com/noble/)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/index.html)
  
**Hardware (Real or Simulated):**
- [Universal Robot UR5e](https://www.universal-robots.com/)
- [Robotiq EPick Vakuumpumpe](https://robotiq.com/products/vacuum-grippers#EPick)
- [Piab piSOFTGRIP](https://www.piab.com/suction-cups-and-soft-grippers/soft-grippers/pisoftgrip-vacuum-driven-soft-gripper-/sg.x)
- [Intel RealSense D415 camera](https://www.intel.com/content/www/us/en/products/sku/128256/intel-realsense-depth-camera-d415/specifications.html)

---

# II) Installation and Setup

To make sure all components of this workspace function correctly, you need to install and set up the required ROS 2 packages, the Google GenAI SDK, and configure access to the Gemini API.

## ROS 2 Packages and Tools

**Instructions:** [INSTALL.md](INSTALL.md)

- [ros2_control](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)
- [Universal Robots ROS2 Driver](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/installation/installation.html)
- [Universal Robots ROS2 Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/jazzy)
- [MoveIt 2](https://moveit.ai/install-moveit2/binary/)
- [Realsense ROS Wrapper](https://github.com/realsenseai/realsense-ros)
- [Webots](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html)

## Google Gemini API and SDK

**Instructions:** [GEMINI_API.md](GEMINI_API.md)

- [Google Gen AI SDK](https://ai.google.dev/gemini-api/docs/quickstart)

---

# III) Workspace Overview

This repository uses a modular architecture, strictly separating hardware drivers, perception systems, and high-level application logic.

```bash
ros2_ws/src/
├── brick_interfaces              # Custom ROS 2 messages and services
├── color_detection               # Alternative classic perception (OpenCV HSV masking)
├── descriptions                  # URDF/Xacro models for the robot, gripper, and environment
├── gemini_vision                 # Main AI perception (Gemini API)
└── workcell                      
    ├── workcell_application      # Main state machine, MoveIt 2 orchestrator (pick_and_place.py)
    ├── workcell_bringup          # Master launch files and centralized configuration YAMLs (cameras, TF frames)
    ├── workcell_control          # Launch files for real UR5e hardware drivers
    ├── workcell_moveit_config    # MoveIt 2 SRDFs, kinematics, and controllers
    └── workcell_simulation       # Webots simulation environment
```

## Key Packages <!-- omit from toc -->

For detailed instructions, please refer to the `README.md` files located inside each specific package folder.

  * **[`gemini_vision`](https://www.google.com/search?q=gemini_vision/)**: The AI perception pipeline. Uses the Gemini API to analyze RGB-D streams, maps bounding boxes to 3D coordinates, and interprets natural language prompts.
  * **[`workcell_application`](https://www.google.com/search?q=workcell/workcell_application/)**: The core MoveIt 2 orchestrator. Contains the main `pick_and_place.py` state machine and hardware alignment tools.
  * **[`color_detection`](https://www.google.com/search?q=color_detection/)**: The alternative classic perception pipeline. Uses OpenCV HSV masking to detect Lego bricks.
  * **[`workcell_bringup`](https://www.google.com/search?q=workcell/workcell_bringup/)**: Contains centralized YAML parameters for cameras and frames to ensure consistency across the entire workcell.
  * **[`workcell_control`](https://www.google.com/search?q=workcell/workcell_control/)**: Contains launch files to initiate the connection with the real UR5e hardware via `ur_robot_driver`.
  * **[`workcell_simulation`](https://www.google.com/search?q=workcell/workcell_simulation/)**: Launch files and worlds for the Webots digital twin environment.
  * **[`workcell_moveit_config`](https://www.google.com/search?q=workcell/workcell_moveit_config/)**: The MoveIt 2 setup package, containing SRDFs, kinematics, and Pilz/OMPL controller configurations.

---

# IV) Workflow Tips (Bash Shortcuts)

To significantly speed up testing and avoid typing long ROS 2 commands, add these functions to your `~/.bashrc` file:

```bash
# --- Pick and Place Shortcuts ---

# Call the vision service directly to test AI reasoning without moving the robot.
testscan() {
    ros2 service call /detect_bricks brick_interfaces/srv/DetectBricks "{user_prompt: '$*'}"
}

# Trigger the actual Pick-and-Place cycle
scan() {
    ros2 topic pub --once /pick_and_place/scan std_msgs/msg/String "{data: '$*'}"
}

# Emergency Stop / Soft Abort (Returns robot to ready pose)
alias stop='ros2 topic pub --once /pick_and_place/stop std_msgs/msg/Empty'
```

After saving and running `source ~/.bashrc`, you can instantly trigger tasks using natural language.

Examples:

```bash
# without arguments (uses the default prompt):
scan
# or
testscan

# with a custom prompt:
scan Pick the red and blue bricks and place them on the left side of the table
# or
testscan Pick the red and blue bricks and place them on the left side of the table
```

---

# V) Quick Start: Pick-and-Place with Gemini Vision

This section outlines how to start the primary Gemini-driven application. You will need to open multiple terminals:

## Step 1: Start the Robot and Camera (Real or Simulated)

### Option A: Simulation (Webots)
  
Start the Webots environment. This includes the UR5e robot and a simulated RealSense camera:

```bash
# Start the Webots simulation
ros2 launch workcell_simulation simulation.launch.py
```
> [!IMPORTANT]
> When using the Webots simulation, you MUST append `use_sim_time:=true` to **all subsequent launch commands** to synchronize the ROS 2 clock.

### Option B: Real Hardware (UR5e & RealSense)

> [!CAUTION]
> Follow all safety precautions when working with real robots.

> [!WARNING]
> **Hardware Specificity:** This project and its configurations are strictly designed and tested for the **UR5e**. Attempting to use this workspace with other Universal Robots models (e.g., UR3e, UR10e) will cause issues. You would need to heavily modify the URDF, MoveIt configurations, and launch files to match your specific robot model's kinematics and limits.

> [!IMPORTANT]
> Before launching, ensure the **[robot setup](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/robot_setup.html#robot-setup)** on the teach pendant is complete. 
> 
> Once the ROS 2 driver is running, you MUST start the program with the **external_control** node on the teach pendant so the robot can receive commands from ROS 2.

This will start the ROS 2 driver for the UR5e robot, allowing you to control the physical robot using ROS 2 interfaces:

```bash
# Start the UR5e driver (replace <ROBOT_IP_ADDRESS> with the actual IP, can also be set in the launch file)
ros2 launch workcell_control start_robot.launch.py robot_ip:=<ROBOT_IP_ADDRESS>
# Optional: Append launch_rviz:=true to automatically start RViz and visualize the robot
```

Next, open a new terminal and start the RealSense camera stream:

```bash
ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x6 rgb_camera.color_profile:=1280x720x6 camera_name:=d415 align_depth.enable:=true enable_sync:=true spatial_filter.enable:=true pointcloud.enable:=false
```


## Step 2: Start Gemini Vision & RViz

Open 2 new terminals and start the Gemini Vision node and RViz. 

```bash
# Start Gemini Vision (append 'use_sim_time:=true' if using Webots simulation)
ros2 launch gemini_vision gemini_vision.launch.py

# Open a new terminal and start RViz (append 'use_sim_time:=true' if using Webots)
ros2 launch workcell_application rviz.launch.py
```

> [!NOTE]
> If you want to test classic, non-AI computer vision, launch `color_detector.launch.py` from the `color_detection` package instead. Keep in mind that this requires the camera to be properly calibrated for HSV masking, and it won't understand natural language prompts.


## Step 3: Start Pick-and-Place Application

Launch the Pick-and-Place application. The robot will move to the `ready` pose and wait in STANDBY mode:

### Option A: Simulation (Webots)

```bash
ros2 launch workcell_application pick_and_place.launch.py use_sim_time:=true
```

### Option B: Real Hardware (UR5e & RealSense)

```bash
ros2 launch workcell_application pick_and_place.launch.py
```

## Step 4: Trigger Pick-and-Place Cycle

Once ready, open a new terminal and use the bash shortcuts configured in **[Section IV](#iv-workflow-tips-bash-shortcuts)** to trigger the cycle:

### Trigger Option A: Default Mode

  Picks all detected bricks and sorts them into their respective color-coded drop-off locations based on the YAML config.

  ```bash
  scan
  ```

### Trigger Option B: Custom Prompt Mode - Gemini ONLY

  Lets you specify a custom natural language instruction to guide the sorting logic. For example, you can ask it to only pick certain colors, or to calculate specific drop-off locations based on the prompt.

  Simply include your instructions in the command after `scan`. For example:

  ```bash
  scan Pick the red and blue bricks and place them on the left side of the table
  ```

---

# VI) Hardware Testing Tools

The `workcell_application` package includes several utility scripts for hardware commissioning and testing. See its [README](https://www.google.com/search?q=workcell_application/README.md) for details on:

  * **`move_to_coords.py`**: Instantly move the TCP to specific X/Y/Z coordinates or named poses.
  * **`verify_alignment.py`**: A step-by-step interactive script to verify physical workspace alignment against the digital twin.
  * **Legacy Port**: Instructions for running the old continuous-topic `brick_sorter_legacy.py`.

---

# VII) Documentation and References

**ROS 2 Ecosystem**
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/index.html)
- [ROS 2 Control](https://control.ros.org/jazzy/index.html)
- [MoveIt 2](https://moveit.picknik.ai/main/index.html#)

**Universal Robots**
- [Universal Robots ROS 2 Driver](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/index.html)
- [Universal Robots ROS 2 Driver - Setup a Robot](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/doc/ur_client_library/doc/setup.html#setup-a-robot)
- [Universal Robots ROS 2 Driver GitHub](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/jazzy)
- [Universal Robots ROS 2 Description GitHub](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/jazzy)
- [Setting up the tool communication on an e-Series robot](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/setup_tool_communication.md)


**Realsense**
- [ROS Wrapper for RealSense(TM) Cameras](https://ai.google.dev/gemini-api/docs/quickstart#make-first-request)

**Google Gemini**
- [Google Gemini API](https://ai.google.dev/gemini-api/docs/)
- [Google Gemini Robotics-ER 1.5](https://ai.google.dev/gemini-api/docs/robotics-overview)

---