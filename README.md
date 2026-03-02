# Using Google Gemini Robotics-ER 1.5 (Gemini API) with Universal Robots UR5e in ROS 2 Jazzy <!-- omit from toc -->

[![jazzy][jazzy-badge]][jazzy]
[![ubuntu24][ubuntu24-badge]][ubuntu24]

[jazzy-badge]: https://img.shields.io/badge/-ROS%202%20JAZZY-orange?style=flat-square&logo=ros
[jazzy]: https://docs.ros.org/en/jazzy/index.html
[ubuntu24-badge]: https://img.shields.io/badge/-UBUNTU%2024%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu24]: https://releases.ubuntu.com/noble/


This repository demonstrates how to integrate the Google Gemini Robotics-ER 1.5 model (via the Gemini API) with a Universal Robots UR5e manipulator in a ROS 2 Jazzy environment. The project includes a complete perception-to-action pipeline for a simple pick-and-place application, utilizing MoveIt 2 for motion planning and control.

- [I) Prerequisites](#i-prerequisites)
- [II) Installation and Setup](#ii-installation-and-setup)
  - [ROS 2 Packages and Google GenAI SDK](#ros-2-packages-and-google-genai-sdk)
  - [Setup Google Gemini API](#setup-google-gemini-api)
- [III) Workspace](#iii-workspace)
  - [Overview](#overview)
  - [Key Packages](#key-packages)
- [IV) Quickstart: Brick Sorter Application](#iv-quickstart-brick-sorter-application)
  - [Step 1: Start the Robot Driver (Real or Simulated)](#step-1-start-the-robot-driver-real-or-simulated)
  - [Step 2: Start the Perception Pipeline](#step-2-start-the-perception-pipeline)
  - [Step 3: Start the Application](#step-3-start-the-application)
- [X) Documentation and References](#x-documentation-and-references)


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

To make sure all components of this workspace function correctly, you need to install and set up the required ROS 2 packages, the Google GenAI SDK, and configure access to the Gemini API.

## ROS 2 Packages and Google GenAI SDK

**Instructions:** [INSTALL.md](INSTALL.md)

- [ros2_control](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)
- [Universal Robots ROS2 Driver](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/installation/installation.html)
- [Universal Robots ROS2 Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/jazzy)
- [MoveIt 2](https://moveit.ai/install-moveit2/binary/)
- [Realsense ROS Wrapper](https://github.com/realsenseai/realsense-ros)
- [Optional: Webots](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html)
- [Google Gen AI SDK](https://ai.google.dev/gemini-api/docs/quickstart)


## Setup Google Gemini API

**Instructions:** [GEMINI_API.md](GEMINI_API.md)


# III) Workspace 

This workspace is structured to provide a clear separation of concerns, with dedicated packages for perception, application logic, and hardware interfaces. Each package contains its own `README.md` with specific instructions and details.

## Overview

This repository is designed with a modular architecture, separating hardware drivers, perception, and high-level application logic.

```bash
ros2_ws/src/
├── color_detection
├── color_detection_msgs
├── descriptions
│   ├── environment_description
│   ├── epick_gripper_description
│   ├── pisoftgrip_description
│   └── workcell_description
├── gemini_robotics_er
└── workcell
    ├── workcell_application
    ├── workcell_control
    ├── workcell_moveit_config
    └── workcell_simulation
```

## Key Packages
For detailed instructions, please refer to the `README.md` files located inside each specific package folder.

* **[`workcell_application`](workcell/workcell_application/)**: The core package. Contains the main `brick_sorter.py` state machine, MoveIt 2 Python API logic, and hardware alignment tools.
* **[`color_detection`](color_detection/)**: The perception pipeline. Uses OpenCV to detect Lego bricks and calculates 3D transformations via `tf2`. Includes a GUI tool for HSV color tuning.
* **[`color_detection_msgs`](color_detection_msgs/)**: Contains the custom ROS 2 messages (e.g., `LegoBrick.msg`) used for communication between perception and application.
* **[`workcell_control`](workcell/workcell_control/)**: Contains launch files (`start_robot.launch.py`) to initiate the connection with the real UR5e hardware via `ur_robot_driver`.
* **[`workcell_simulation`](workcell/workcell_simulation/)**: Launch files and worlds for the Webots simulation environment (`simulation.launch.py`).
* **[`workcell_moveit_config`](workcell/workcell_moveit_config/)**: The MoveIt 2 setup package, containing SRDFs, kinematics, and controller configurations.
* **[`descriptions`](descriptions/)**: Modular URDF and Xacro files for the complete Workcell, the UR5e, piSOFTGRIP, Robotiq EPick, and the environment.



# IV) Quickstart: Brick Sorter Application

To startup the complete system for automated pick-and-place, you need to start 3 launch files in individual terminals. 


## Step 1: Start the Robot Driver (Real or Simulated)

This will start the ROS 2 node that interfaces with the UR5e, either in real hardware mode or in Webots simulation.

<details>
  <summary><b>Option A: Webots Simulation</b></summary>

This will launch the Webots simulation of the workcell. 

Make sure you have Webots and the `webots_ros2` package installed. 

### Launch Command <!-- omit from toc -->
  
```bash
ros2 launch workcell_simulation simulation.launch.py
```
</details>

<details>
  <summary><b>Option B: Real Hardware (UR5e)</b></summary>

This will start the ROS 2 driver for the UR5e robot, allowing you to control the physical robot using ROS 2 interfaces.

Make sure the **external control** node is **active** on the teach pendant.
For details see the [**workcell_control README**](workcell/workcell_control/README.md).

> [!CAUTION]
> Follow all safety precautions when working with real robots.

### Launch Command <!-- omit from toc -->

```bash
# You can set the robot_ip either via command line argument or directly in the start_robot.launch.py file
ros2 launch workcell_control start_robot.launch.py robot_ip:=<ROBOT_IP_ADDRESS>
```
</details>


## Step 2: Start the Perception Pipeline

This node processes the camera stream and publishes the 3D coordinates of detected bricks.

For details see the [**color_detection README**](color_detection/README.md).

### Launch Command <!-- omit from toc -->

```bash
ros2 launch color_detection color_detector.launch.py # add launch arguments as needed, see below
```

### Launch Arguments <!-- omit from toc -->

You can append the following arguments to the launch command to customize the behavior:

- `use_sim_time` (bool, default: false): Set `use_sim_time:=true` to run with simulation camera topics and parameters, and use the `/clock` topic published by Webots.
- `sort_method` (string, default: "closest"): Use `sort_method:=random` to shuffle the target order.

Example with arguments:

```bash
ros2 launch color_detection color_detector.launch.py use_sim_time:=true sort_method:=random
```

> [!IMPORTANT] 
> **Using Real Hardware:** You need to adjust the **camera topics** and **frames** in the `real_params.yaml` file before running the node.


## Step 3: Start the Application

This will start the high-level state machine utilizing MoveIt 2 to pick and sort the detected bricks.

For details see the [**workcell_application README**](workcell/workcell_application/README.md).

### Launch Command <!-- omit from toc -->

```bash
ros2 launch workcell_application brick_sorter.launch.py # add launch arguments as needed, see below
```

### Launch Arguments <!-- omit from toc -->

You can append the following argument to the launch command to customize the behavior:

- `use_sim_time` (bool, default: false): Set `use_sim_time:=true` to run with simulation camera topics and parameters, and the simulation clock (`/clock` topic).



# X) Documentation and References

ROS 2
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/index.html)
- [ROS 2 Control](https://control.ros.org/jazzy/index.html)
- [MoveIt 2](https://moveit.picknik.ai/main/index.html#)

Universal Robots
- [Universal Robots ROS 2 Driver](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/index.html)
- [Universal Robots ROS 2 Driver - Setup a Robot](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/doc/ur_client_library/doc/setup.html#setup-a-robot)
- [Universal Robots ROS 2 Driver GitHub](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/jazzy)
- [Universal Robots ROS 2 Description GitHub](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/jazzy)
- [Setting up the tool communication on an e-Series robot](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/setup_tool_communication.md)


Realsense
- [ROS Wrapper for RealSense(TM) Cameras](https://ai.google.dev/gemini-api/docs/quickstart#make-first-request)

Google Gemini
- [Google Gemini API](https://ai.google.dev/gemini-api/docs/)
- [Google Gemini Robotics-ER 1.5](https://ai.google.dev/gemini-api/docs/robotics-overview)

---
> Jump to: [Table of Contents](#table-of-contents)