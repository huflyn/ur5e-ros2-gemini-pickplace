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
  - [Folder Structure](#folder-structure)
  - [Key Packages](#key-packages)
- [IV) Usage](#iv-usage)
  - [Step 1: Start the Robot Driver (Real or Simulated)](#step-1-start-the-robot-driver-real-or-simulated)
    - [Option A: Webots Simulation](#option-a-webots-simulation)
    - [Option B: Fake Hardware (no Webots, just RViz and MoveIt)](#option-b-fake-hardware-no-webots-just-rviz-and-moveit)
    - [Option C: Real Hardware (UR5e)](#option-c-real-hardware-ur5e)
  - [Step 2: Start MoveIt](#step-2-start-moveit)
  - [Step 3: Run Task (e.g. pick and place)](#step-3-run-task-eg-pick-and-place)
- [1. Descriptions](#1-descriptions)
- [X) Documentation and References](#x-documentation-and-references)


# I) Prerequisites

**Software:**
- [(Ubuntu 24.04 LTS)](https://releases.ubuntu.com/noble/)
- [(ROS 2 Jazzy)](https://docs.ros.org/en/jazzy/index.html)
  
**Hardware:**
- [(Universal Robot UR5e)](https://www.universal-robots.com/)
- [(Robotiq EPick Vakuumpumpe)](https://robotiq.com/products/vacuum-grippers#EPick)
- [(Piab piSOFTGRIP)](https://www.piab.com/suction-cups-and-soft-grippers/soft-grippers/pisoftgrip-vacuum-driven-soft-gripper-/sg.x)
- [(Intel RealSense 415D camera)](https://www.intel.com/content/www/us/en/products/sku/128256/intel-realsense-depth-camera-d415/specifications.html)



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

## Folder Structure

The Workspace folder structure is as follows:

```
lego_sorter_ws/
├── src/
│   ├── lego_sorter_interfaces/        # Custom msgs, srvs, actions
│   ├── description/       # URDF/Xacro, meshes, Webots PROTO files
│   ├── manipulation/      # Pick & place state machine
│   ├── moveit_config/     # MoveIt 2 configuration
│   ├── perception/        # Camera pipeline + Gemini API node
│   ├── webots/            # Webots world file, sim launch files
│   └── lego_sorter_bringup/           # Top-level launch files, configs, rviz
```

## Key Packages




# IV) Usage

To startup the complete system, you’ll have to start x launch files in individual terminals.

## Step 1: Start the Robot Driver (Real or Simulated)

### Option A: Webots Simulation

```bash
ros2 launch workcell_webots simulation.launch.py
```
This will start the Webots simulation of the workcell, which includes a simulated UR5e robot. The robot in the simulation will be controlled using the same ROS 2 interfaces as a real robot, allowing you to test your MoveIt 2 configuration without needing access to physical hardware.

### Option B: Fake Hardware (no Webots, just RViz and MoveIt)

```bash
# Make sure to set use_fake_hardware:=true in the command below if you want to use fake hardware
ros2 launch workcell_control start_robot.launch.py use_fake_hardware:=true
```
This will start a simulated robot using the ros2_control fake hardware interface. The robot will mirror the commands sent to it, allowing you to test your MoveIt 2 configuration without needing access to physical hardware or the Webots simulation.

### Option C: Real Hardware (UR5e)

If you want to start a real robot, make sure that the **[robot setup](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/robot_setup.html#robot-setup)** is done and **external_control** is active on the robot.
Make sure to set the correct **robot_ip** in the command below or in the ``start_robot.launch.py`` file in the ``workcell_control`` package!

```bash
# You can set the robot_ip either via command line argument or directly in the start_robot.launch.py file
ros2 launch workcell_control start_robot.launch.py robot_ip:=<ROBOT_IP_ADDRESS>
```
This will start the ROS 2 driver for the UR5e robot, allowing you to control the physical robot using ROS 2 interfaces. Make sure to follow all safety precautions when working with real robots.

## Step 2: Start MoveIt

Now that we have a robot running (either simulated or real), we can launch the MoveIt **[move_group node](https://moveit.picknik.ai/main/doc/concepts/move_group.html)**.

- With Webots Simulation, you need to set the argument ``use_sim_time:=true`` to make sure that MoveIt uses the simulation time provided by Webots.

```bash
# Set argument use_sim_time:=true if you are using the Webots simulation, default is false
ros2 launch my_robot_cell_moveit_config move_group.launch.py use_sim_time:=true
```

- With Fake Hardware or Real Hardware, you can simply launch the move_group node without the ``use_sim_time`` argument.
  
```bash
ros2 launch my_robot_cell_moveit_config move_group.launch.py
```

If everything went well you should see the output: “You can start planning now!”.

To interact with the MoveIt setup, you can start RViz with the correct setup file:

```bash
ros2 launch my_robot_cell_moveit_config moveit_rviz.launch.py
```

## Step 3: Run Task (e.g. pick and place)


---
---
---

# 1. Descriptions


All description files (URDF/XACRO) for the robot, grippers, sensors, and the workcell will be stored in the ``descriptions`` folder. This includes the URDF/XACRO files for the UR5e robot, the Robotiq EPick gripper, the Piab piSOFTGRIP, and the Intel RealSense camera.
To visualize the robot in RViz use the ``view_workcell.launch.py`` file from the workcell_description package.

```bash
ros2 launch workcell_description view_workcell.launch.py
```

---

# X) Documentation and References

- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/index.html)
- [ROS 2 Control](https://control.ros.org/jazzy/index.html)
- [MoveIt 2](https://moveit.picknik.ai/main/index.html#)

- [Universal Robots ROS 2 Driver](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/index.html)
- [Universal Robots ROS 2 Driver GitHub](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/jazzy)
- [Universal Robots Client Library GitHub](https://github.com/UniversalRobots/Universal_Robots_Client_Library) EVTL NICHT NOTWENDIG
- [Universal Robots ROS 2 Description GitHub](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/jazzy)

- [ROS Wrapper for RealSense(TM) Cameras](https://ai.google.dev/gemini-api/docs/quickstart#make-first-request)

- [ROS 2 driver for the Robotiq EPick gripper](https://github.com/PickNikRobotics/ros2_epick_gripper)

- [Google Gemini API](https://ai.google.dev/gemini-api/docs/)
- [Google Gemini Robotics-ER 1.5](https://ai.google.dev/gemini-api/docs/robotics-overview)