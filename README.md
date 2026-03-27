
# AI-Driven Pick-and-Place with Google Gemini and Universal Robots UR5e in ROS 2 Jazzy  <!-- omit from toc -->

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

![Video of the AI-Driven (Gemini) Pick-and-Place Application](docs/videos/pick-and-place_user-prompt-mode_demo.mp4)

- [I) Prerequisites](#i-prerequisites)
- [II) Installation and Setup](#ii-installation-and-setup)
- [III) Workspace Overview](#iii-workspace-overview)
- [IV) Developer Workflow Tips (Bash Shortcuts)](#iv-developer-workflow-tips-bash-shortcuts)
- [V) Quick Start: AI Pick-and-Place Pipeline](#v-quick-start-ai-pick-and-place-pipeline)
    - [Step 1: Start Hardware or Simulation](#step-1-start-hardware-or-simulation)
    - [Step 2: Start Gemini Vision \& RViz](#step-2-start-gemini-vision--rviz)
    - [Step 3: Start Application \& Trigger](#step-3-start-application--trigger)
- [VI) Hardware Testing Tools](#vi-hardware-testing-tools)
- [VII) Documentation and References](#vii-documentation-and-references)


# I) Prerequisites

**Software:**
- [Ubuntu 24.04 LTS](https://releases.ubuntu.com/noble/)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/index.html)
  
**Hardware (Real or Simulated):**
- [Universal Robot UR5e](https://www.universal-robots.com/)
- [Robotiq EPick Vakuumpumpe](https://robotiq.com/products/vacuum-grippers#EPick)
- [Piab piSOFTGRIP](https://www.piab.com/suction-cups-and-soft-grippers/soft-grippers/pisoftgrip-vacuum-driven-soft-gripper-/sg.x)
- [Intel RealSense D415 camera](https://www.intel.com/content/www/us/en/products/sku/128256/intel-realsense-depth-camera-d415/specifications.html)


# II) Installation and Setup

To ensure all components function correctly, you must install the required ROS 2 packages and configure access to the Gemini API.

1. **ROS 2 & Dependencies:** Follow the [INSTALL.md](INSTALL.md) guide.
2. **AI Setup:** Follow the [GEMINI_API.md](GEMINI_API.md) guide to generate your Google Gemini API key and export it as an environment variable.


# III) Workspace Overview

This repository uses a modular architecture, strictly separating hardware drivers, perception systems, and high-level application logic.

```bash
ros2_ws/src/
├── gemini_vision             # Main AI perception (Gemini API, spatial reasoning)
├── color_detection           # Alternative classic perception (OpenCV HSV masking)
├── brick_interfaces          # Custom ROS 2 messages and services (e.g., DetectBricks.srv)
├── descriptions              # URDF/Xacro models for the robot, grippers, and environment
├── workcell_bringup          # Centralized configuration YAMLs (cameras, TF frames)
├── workcell_application      # Main state machine, MoveIt 2 orchestrator (pick_and_place.py)
├── workcell_control          # Launch files for real UR5e hardware drivers
├── workcell_moveit_config    # MoveIt 2 SRDFs, kinematics, and controllers
└── workcell_simulation       # Webots digital twin environments
````

*(For detailed configuration options, see the `README.md` files located inside each package folder.)*

# IV) Developer Workflow Tips (Bash Shortcuts)

To significantly speed up testing and avoid typing long ROS 2 commands with JSON payloads, add these functions to your `~/.bashrc` file.

Open your terminal and edit your bashrc (e.g., `nano ~/.bashrc`), then append this block:

```bash
# --- Pick and Place Shortcuts ---

# Call the vision service directly to test AI reasoning without moving the robot
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

After saving and running `source ~/.bashrc`, you can instantly trigger tasks using natural language, e.g.: `scan Pick the red bricks and put them on the left side`.

# V) Quick Start: AI Pick-and-Place Pipeline

This section outlines how to start the primary Gemini-driven application. Open **three separate terminals**.

> [\!IMPORTANT]
> When using the Webots simulation, you MUST append `use_sim_time:=true` to **all** launch commands to synchronize the ROS 2 clock.

### Step 1: Start Hardware or Simulation

**Simulation (Webots):**

```bash
ros2 launch workcell_simulation simulation.launch.py
```

**Real Hardware:**
Ensure the *external control* node is active on the UR teach pendant, then launch the driver:

```bash
ros2 launch workcell_control start_robot.launch.py robot_ip:=<ROBOT_IP_ADDRESS>
```

### Step 2: Start Gemini Vision & RViz

Start the AI perception node and the RViz visualization to see the robot state and live camera annotations:

```bash
ros2 launch gemini_vision gemini_vision.launch.py use_sim_time:=true
ros2 launch workcell_application rviz.launch.py use_sim_time:=true
```

*(Note: If you prefer classic, non-AI computer vision, launch `color_detector.launch.py` from the `color_detection` package instead).*

### Step 3: Start Application & Trigger

Launch the MoveIt orchestrator. The robot will move to the `ready` pose and wait in STANDBY mode:

```bash
ros2 launch workcell_application pick_and_place.launch.py use_sim_time:=true
```

Once ready, open a new terminal and use the bash shortcuts configured in Section IV to trigger the cycle:

  * `scan` (Picks and sorts all detected objects to default locations).
  * `scan Pick only the blue block and place it at x=0.2, y=0.3` (Custom AI prompt).

# VI) Hardware Testing Tools

The `workcell_application` package includes several utility scripts for hardware commissioning and testing. See its [README](https://www.google.com/search?q=workcell_application/README.md) for details on:

  * **`move_to_coords.py`**: Instantly move the TCP to specific X/Y/Z coordinates or named poses.
  * **`verify_alignment.py`**: A step-by-step interactive script to verify physical workspace alignment against the digital twin.
  * **Legacy Port**: Instructions for running the old continuous-topic `brick_sorter_legacy.py`.

# VII) Documentation and References

**ROS 2 Ecosystem**

  - [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/index.html)
  - [MoveIt 2](https://moveit.picknik.ai/main/index.html#)

**Universal Robots**

  - [Universal Robots ROS 2 Driver](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/index.html)
  - [UR ROS 2 Driver - Setup a Robot](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/doc/ur_client_library/doc/setup.html#setup-a-robot)

**Sensors & AI**

  - [ROS Wrapper for RealSense™ Cameras](https://github.com/IntelRealSense/realsense-ros)
  - [Google Gemini API Docs](https://ai.google.dev/gemini-api/docs/)
  - [Google Gemini Robotics-ER 1.5](https://ai.google.dev/gemini-api/docs/robotics-overview)

<!-- end list -->
