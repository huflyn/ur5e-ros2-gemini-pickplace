# Workcell Control Package (`workcell_control`) <!-- omit from toc -->

[![jazzy][jazzy-badge]][jazzy]
[![ubuntu24][ubuntu24-badge]][ubuntu24]

[jazzy-badge]: https://img.shields.io/badge/-ROS%202%20JAZZY-orange?style=flat-square&logo=ros
[jazzy]: https://docs.ros.org/en/jazzy/index.html
[ubuntu24-badge]: https://img.shields.io/badge/-UBUNTU%2024%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu24]: https://releases.ubuntu.com/noble/


This package manages the connection to the physical UR5e hardware. It contains the launch files necessary to start the official `ur_robot_driver` with the correct calibration parameters, network settings, and ROS 2 controllers for this specific workcell.

![Screenshot of the UR5e in RViz.](/docs/images/workcell_control_rviz.png)

---

- [I) Package Structure](#i-package-structure)
- [II) Starting the Robot Driver](#ii-starting-the-robot-driver)
  - [Option A: Real Hardware (UR5e)](#option-a-real-hardware-ur5e)
  - [Option B: Mock Hardware (no Webots simulation, just RViz)](#option-b-mock-hardware-no-webots-simulation-just-rviz)

---

## I) Package Structure
* **`start_robot.launch.py`**: The main launch file to start the UR5e driver. It includes parameters for both real hardware and mock hardware, allowing you to switch between them easily.
* `config/` directory: Contains the robots calibration YAML file.

---

## II) Starting the Robot Driver

### Option A: Real Hardware (UR5e)

This will start the ROS 2 driver for the UR5e robot, allowing you to control the physical robot using ROS 2 interfaces.

> [!CAUTION]
> Follow all safety precautions when working with real robots.

> [!WARNING]
> **Hardware Specificity:** This project and its configurations are strictly designed and tested for the **UR5e**. Attempting to use this workspace with other Universal Robots models (e.g., UR3e, UR10e) will cause issues. You would need to heavily modify the URDF, MoveIt configurations, and launch files to match your specific robot model's kinematics and limits.

> [!IMPORTANT]
> Before launching, ensure the **[robot setup](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/robot_setup.html#robot-setup)** on the teach pendant is complete. 
> 
> Once the ROS 2 driver is running, you MUST start the program with the **external_control** node on the teach pendant so the robot can receive commands from ROS 2.

```bash
# Start the UR5e driver (replace <ROBOT_IP_ADDRESS> with the actual IP, can also be set in the launch file)
ros2 launch workcell_control start_robot.launch.py robot_ip:=<ROBOT_IP_ADDRESS>
# Optional: Append launch_rviz:=true to automatically start RViz and visualize the robot
```

### Option B: Mock Hardware (no Webots simulation, just RViz)

```bash
ros2 launch workcell_control start_robot.launch.py use_mock_hardware:=true
# Optional: Append launch_rviz:=true to automatically start RViz and visualize the robot
```
This will start a virtual robot using the ros2_control mock hardware interface. The robot will mirror the commands sent to it, without needing access to physical hardware or the Webots simulation. Set ``launch_rviz:=true`` to visualize the robot in RViz, as shown in the screenshot at the top.

> [!NOTE]
> If you want to use the **Webots simulation** instead, please refer to the **[workcell_simulation package](../workcell_simulation/README.md)**, which includes its own launch file that starts the Webots simulation.

---
