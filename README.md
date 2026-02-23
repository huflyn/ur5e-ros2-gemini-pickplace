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
- [IV) MoveIt 2 Usage](#iv-moveit-2-usage)
  - [Step 1: Start the Robot Driver (Real or Simulated)](#step-1-start-the-robot-driver-real-or-simulated)
  - [Step 2: Start MoveIt](#step-2-start-moveit)
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

## Key Packages




# IV) MoveIt 2 Usage

To startup the complete system, you’ll have to start 3 launch files in individual terminals.

## Step 1: Start the Robot Driver (Real or Simulated)

- ### Option A: Webots Simulation

  ```bash
  ros2 launch workcell_webots simulation.launch.py
  ```
  This will start the Webots simulation of the workcell, which includes a simulated UR5e robot. The robot in the simulation will be controlled using the same ROS 2 interfaces as a real robot, allowing you to test your MoveIt 2 configuration without needing access to physical hardware.

- ### Option B: Mock Hardware (no Webots, just RViz and MoveIt)

  ```bash
  # Make sure to set use_mock_hardware:=true in the command below if you want to use mock hardware
  ros2 launch workcell_control start_robot.launch.py use_mock_hardware:=true
  ```
  This will start a simulated robot using the ros2_control mock hardware interface. The robot will mirror the commands sent to it, allowing you to test your MoveIt 2 configuration without needing access to physical hardware or the Webots simulation.

- ### Option C: Real Hardware (UR5e)

  If you want to start a real robot, make sure that the **[robot setup](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/robot_setup.html#robot-setup)** is done and **external_control** is active on the robot.
  Make sure to set the correct **robot_ip** in the command below or in the ``start_robot.launch.py`` file in the ``workcell_control`` package!

  ```bash
  # You can set the robot_ip either via command line argument or directly in the start_robot.launch.py file
  ros2 launch workcell_control start_robot.launch.py robot_ip:=<ROBOT_IP_ADDRESS>
  ```
  This will start the ROS 2 driver for the UR5e robot, allowing you to control the physical robot using ROS 2 interfaces. Make sure to follow all safety precautions when working with real robots.

## Step 2: Start MoveIt

Now that we have a robot running (either simulated or real), we can launch the MoveIt **[move_group node](https://moveit.picknik.ai/main/doc/concepts/move_group.html)**.
 
```bash
ros2 launch workcell_moveit_config move_group.launch.py
```

If everything went well you should see the output: “You can start planning now!”.

To interact with the MoveIt setup, you can start RViz with the correct setup file:

```bash
ros2 launch workcell_moveit_config moveit_rviz.launch.py
```

---

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