# Using Google Gemini Robotics-ER 1.5 (Gemini API) with Universal Robots UR5e in ROS 2 Jazzy <!-- omit from toc -->

[![jazzy][jazzy-badge]][jazzy]
[![ubuntu24][ubuntu24-badge]][ubuntu24]

[jazzy-badge]: https://img.shields.io/badge/-ROS%202%20JAZZY-orange?style=flat-square&logo=ros
[jazzy]: https://docs.ros.org/en/jazzy/index.html
[ubuntu24-badge]: https://img.shields.io/badge/-UBUNTU%2024%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu24]: https://releases.ubuntu.com/noble/

- [I) Prerequisites](#i-prerequisites)
- [II) Installation and Setup](#ii-installation-and-setup)
  - [Install ROS 2 Drivers and Tools](#install-ros-2-drivers-and-tools)
  - [Setup Google Gemini API](#setup-google-gemini-api)
- [III) Workspace Overview](#iii-workspace-overview)
    - [Key Packages](#key-packages)
- [IV) Quickstart: Brick Sorter Application](#iv-quickstart-brick-sorter-application)
    - [Step 1: Start the Robot Driver (Real or Simulated)](#step-1-start-the-robot-driver-real-or-simulated)
    - [Step 2: Start the Perception Pipeline](#step-2-start-the-perception-pipeline)
    - [Step 3: Start the Application](#step-3-start-the-application)
- [X) Documentation and References](#x-documentation-and-references)


# I) Prerequisites

**Software:**
- [(Ubuntu 24.04 LTS)](https://releases.ubuntu.com/noble/)
- [(ROS 2 Jazzy)](https://docs.ros.org/en/jazzy/index.html)
  
**Hardware:**
- [(Universal Robot UR5e)](https://www.universal-robots.com/)
- [(Robotiq EPick Vakuumpumpe)](https://robotiq.com/products/vacuum-grippers#EPick)
- [(Piab piSOFTGRIP)](https://www.piab.com/suction-cups-and-soft-grippers/soft-grippers/pisoftgrip-vacuum-driven-soft-gripper-/sg.x)
- [(Intel RealSense D415 camera)](https://www.intel.com/content/www/us/en/products/sku/128256/intel-realsense-depth-camera-d415/specifications.html)



# II) Installation and Setup

## Install ROS 2 Drivers and Tools

Instructions: [INSTALL.md](INSTALL.md)

- [ros2_control](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)
- [Universal Robots ROS2 Driver](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/installation/installation.html)
- [MoveIt 2](https://moveit.ai/install-moveit2/binary/)
- [Realsense ROS Wrapper](https://github.com/realsenseai/realsense-ros)
- [Google Gen AI SDK](https://ai.google.dev/gemini-api/docs/quickstart)
- [Optional: Webots](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html)


## Setup Google Gemini API

Instructions: [GEMINI_API.md](GEMINI_API.md)


# III) Workspace Overview

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

### Key Packages
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

### Step 1: Start the Robot Driver (Real or Simulated)

- **Option A: Webots Simulation**
  ```bash
  ros2 launch workcell_simulation simulation.launch.py
  ```

- **Option B: Real Hardware (UR5e)**

  Make sure the external control node is active on the teach pendant.

  ```bash
  ros2 launch workcell_control start_robot.launch.py
  ```

### Step 2: Start the Perception Pipeline

This node processes the camera stream and publishes the 3D coordinates of detected bricks.

```bash
ros2 launch color_detection color_detector.launch.py
```

### Step 3: Start the Application

This will start the high-level state machine utilizing MoveIt 2 to pick and sort the detected bricks.

```bash
ros2 launch workcell_application brick_sorter.launch.py
```

> [!NOTE]
> For **more details** to the **Brick Sorter** and **workspace calibration and alignment testing** see the **`workcell_application`** [**README**](workcell/workcell_application/README.md).

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
