
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

This repository demonstrates how to integrate vision-language models (like Gemini 3.1 Flash-Lite and Gemini Robotics-ER 1.5) with a Universal Robots UR5e manipulator in a ROS 2 Jazzy environment. 

The project features a complete perception-to-action pipeline. It utilizes the Gemini API for advanced spatial reasoning and natural language processing, and MoveIt 2 for hybrid motion planning (OMPL + Pilz LIN/PTP).

https://github.com/user-attachments/assets/ba4a9620-bd2b-4814-b3d8-9d6d37d2f78e

---

- [I) Prerequisites](#i-prerequisites)
- [II) Installation and Setup](#ii-installation-and-setup)
  - [ROS 2 Packages and Tools](#ros-2-packages-and-tools)
  - [Google Gemini API and SDK](#google-gemini-api-and-sdk)
- [III) Workspace Overview](#iii-workspace-overview)
- [IV) Global Configuration (YAML)](#iv-global-configuration-yaml)
- [V) Workflow Tips (Bash Shortcuts)](#v-workflow-tips-bash-shortcuts)
- [VI) Quick Start: Automated Bringup (Recommended)](#vi-quick-start-automated-bringup-recommended)
  - [Option A: Simulation (Webots)](#option-a-simulation-webots)
  - [Option B: Real Hardware (UR5e \& RealSense)](#option-b-real-hardware-ur5e--realsense)
- [VII) Triggering the Pick-and-Place Cycle](#vii-triggering-the-pick-and-place-cycle)
  - [Test the vision system (no robot movement)](#test-the-vision-system-no-robot-movement)
  - [Trigger the full Pick-and-Place cycle](#trigger-the-full-pick-and-place-cycle)
- [VIII) Advanced Usage, Manual Launch \& Legacy Systems](#viii-advanced-usage-manual-launch--legacy-systems)
- [IX) Hardware Testing Tools](#ix-hardware-testing-tools)
- [X) Documentation and References](#x-documentation-and-references)

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

  * **[`gemini_vision`](/gemini_vision/README.md)**: The AI perception pipeline. Uses the Gemini API to analyze RGB-D streams, maps bounding boxes to 3D coordinates, and interprets natural language prompts.
  * **[`workcell_application`](/workcell/workcell_application/README.md)**: The core MoveIt 2 orchestrator. Contains the main `pick_and_place.py` state machine and hardware alignment tools.
  * **[`color_detection`](/color_detection/README.md)**: The alternative classic perception pipeline. Uses OpenCV HSV masking to detect Lego bricks.
  * **[`workcell_bringup`](/workcell/workcell_bringup/README.md)**: Contains centralized YAML parameters for cameras and frames to ensure consistency across the entire workcell.
  * **[`workcell_control`](/workcell/workcell_control/README.md)**: Contains launch files to initiate the connection with the real UR5e hardware via `ur_robot_driver`.
  * **[`workcell_simulation`](/workcell/workcell_simulation/README.md)**: Launch files and worlds for the Webots digital twin environment.
  * **[`workcell_moveit_config`](/workcell/workcell_moveit_config/README.md)**: The MoveIt 2 setup package, containing SRDFs, kinematics, and Pilz/OMPL controller configurations.
  * **[`descriptions`](/descriptions/README.md)**: URDF/Xacro models for the UR5e, the vacuum gripper, and the workcell environment. Used for both simulation and RViz visualization.
  * **[`brick_interfaces`](/brick_interfaces/README.md)**: Custom ROS 2 messages and services for communication between the vision system and the pick-and-place orchestrator.

---

# IV) Global Configuration (YAML)

This project uses YAML files to easily adapt to different setups without requiring any source code changes.

> [!NOTE]
> **Ready out-of-the-box for Simulation:** The default YAML parameters are fully optimized for the included Webots world and our specific UR5e lab environment.
> 
> **Adapting to your Hardware:** To run this project in your own physical environment, adjust the configuration files to match your specific camera topics, workspace boundaries, and drop-off coordinates.

* **[Grasping Heights & Drop-off Zones](/workcell/workcell_application/README.md#grasping-heights--drop-off-zones) (`workcell_application`):** Defines the safe hover/grasp heights, brick center offsets, and specific Cartesian drop-off coordinates for the pick-and-place orchestrator.
  * `workcell/workcell_application/config/pick_and_place_parameters.yaml`
* **[Workspace Boundaries, Camera & TF Frames](/workcell/workcell_bringup/README.md#ii-workspace-configuration-yaml)  (`workcell_bringup`):** Defines the physical table dimensions (hardware safety limits), camera topic names, and target `tf2` reference frames for both simulation and real hardware.
  * `workcell/workcell_bringup/config/sim_workspace_parameters.yaml`
  * `workcell/workcell_bringup/config/real_workspace_parameters.yaml`
* **[Color Thresholds](/color_detection/README.md#iii-configuration--camera-setup-yaml) (`color_detection`):** If you are using the classic HSV vision mode instead of the Gemini AI, this file stores the upper and lower bounds for color masking.
  * `color_detection/config/hsv_bounds.yaml`

---

# V) Workflow Tips (Bash Shortcuts)

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

# VI) Quick Start: Automated Bringup (Recommended)

The easiest way to start the complete perception-to-action pipeline is using the centralized master launch files from the [`workcell_bringup`](/workcell/workcell_bringup/README.md) package. These files use staggered timers to automatically start the hardware/simulation, the selected vision system, RViz, and the MoveIt application in the correct order.

> [!IMPORTANT]
> **API Key Required:** Because the Gemini Vision pipeline is the default mode, ensure you have exported your `GEMINI_API_KEY` in your terminal before running these launch files.

> [!NOTE]
> **Staggered Startup:** Please wait roughly 10-15 seconds after launching for all controllers, drivers, and nodes to fully boot up!

## Option A: Simulation (Webots)

Start the Webots environment, simulated camera, and all nodes:

```bash
# Default: Gemini Vision
ros2 launch workcell_bringup sim.launch.py
```

**Alternative Vision Modes:**
Append the `vision` argument to use classic computer vision methods instead of the Gemini AI:
* **OpenCV HSV Masking:** `ros2 launch workcell_bringup sim.launch.py vision:=hsv`
* **Legacy (Continuous ROS 1 Port):** `ros2 launch workcell_bringup sim.launch.py vision:=legacy`

## Option B: Real Hardware (UR5e & RealSense)

> [!CAUTION]
> Follow all safety precautions when working with real robots. Keep the emergency stop button within reach.

> [!WARNING]
> **Hardware Specificity:** This project and its configurations are strictly designed and tested for the **UR5e**. Attempting to use this workspace with other UR models will cause issues.

> [!IMPORTANT]
> Ensure the physical robot is powered on and the **external control** program is loaded on the teach pendant. The launch file will attempt to automatically "play" the program via the dashboard client.

Connect to the UR5e, start the RealSense stream, and launch all nodes:

```bash
# Default: Gemini Vision (replace <ROBOT_IP_ADDRESS> with the actual IP)
ros2 launch workcell_bringup real.launch.py robot_ip:=<ROBOT_IP_ADDRESS>
```

**Alternative Vision Mode:**
Append the `vision` argument to use classic OpenCV computer vision:
* **OpenCV HSV Masking:** `ros2 launch workcell_bringup real.launch.py robot_ip:=<ROBOT_IP_ADDRESS> vision:=hsv`

-----

# VII) Triggering the Pick-and-Place Cycle

Once the master launch file has finished its sequence, the robot will move to its `ready` pose and wait in STANDBY mode.

> [!NOTE]
> If you launched the system in **`legacy`** vision mode (`vision:=legacy`), the robot will not wait for a manual trigger. It will automatically start sorting as soon as bricks appear in the camera's view and match the configured HSV color bounds.

For the `gemini` and `hsv` modes, open a **new terminal** to interact with the system. You can trigger actions using standard ROS 2 CLI commands or the convenient bash shortcuts (if you configured them as shown in **[Section IV](#v-workflow-tips-bash-shortcuts)**).

## Test the vision system (no robot movement)

This calls the active vision service directly to test the object detection and target calculation without moving the physical or simulated arm.

**Using standard ROS 2 commands:**

```bash
# Default call
ros2 service call /detect_bricks brick_interfaces/srv/DetectBricks

# Custom natural language instruction (Gemini Vision only)
ros2 service call /detect_bricks brick_interfaces/srv/DetectBricks "{user_prompt: 'Pick the red and blue bricks'}"
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

# VIII) Advanced Usage, Manual Launch & Legacy Systems

If you want to debug specific nodes, bypass the automated `workcell_bringup`, or explore the older non-AI systems, please refer to the detailed instructions in the respective package documentation:

* **[Manual Component Launch](./workcell/workcell_application/README.md):** Step-by-step guide to starting the Webots simulation or UR5e driver, RealSense camera, Vision nodes, and the pick-and-place orchestrator individually across multiple terminals.
* **[Classic Computer Vision (HSV)](./color_detection/README.md):** Details on configuring, calibrating, and running the standard OpenCV HSV masking pipeline instead of the Gemini API.
* **[Legacy Brick Sorter (ROS 1 Port)](./workcell/workcell_application/README.md):** Instructions for running the older, continuous-loop sorting method (`brick_sorter_legacy`) that does not rely on trigger commands or the Gemini API.

---

# IX) Hardware Testing Tools

The `workcell_application` package includes several utility scripts for hardware commissioning and testing. See its [README](/workcell/workcell_application/README.md) for detailed instructions on using:

* **`move_to_coords.py`**: Instantly move the physical or simulated TCP to specific X/Y/Z coordinates or named MoveIt poses.
* **`verify_alignment.py`**: A step-by-step interactive script to verify the physical workspace and camera alignment against the digital twin in RViz.

---

# X) Documentation and References

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
- [Google Gemini 3](https://ai.google.dev/gemini-api/docs/gemini-3)
- [Google Gemini Robotics-ER 1.5](https://ai.google.dev/gemini-api/docs/robotics-overview)

---