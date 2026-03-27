# Workcell Control Package (`workcell_control`) <!-- omit from toc -->

[![jazzy][jazzy-badge]][jazzy]
[![ubuntu24][ubuntu24-badge]][ubuntu24]

[jazzy-badge]: https://img.shields.io/badge/-ROS%202%20JAZZY-orange?style=flat-square&logo=ros
[jazzy]: https://docs.ros.org/en/jazzy/index.html
[ubuntu24-badge]: https://img.shields.io/badge/-UBUNTU%2024%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu24]: https://releases.ubuntu.com/noble/


This package manages the connection to the physical UR5e hardware. It contains the launch files necessary to start the official `ur_robot_driver` with the correct calibration parameters, network settings, and ROS 2 controllers for this specific workcell.

![Screenshot of the UR5e in RViz.](/docs/images/workcell_control_rviz.png)

- [I) Package Structure](#i-package-structure)
- [II) Usage / Launch](#ii-usage--launch)
  - [Option A: Real Hardware (UR5e)](#option-a-real-hardware-ur5e)
  - [Option B: Mock Hardware (no Webots simulation, just RViz)](#option-b-mock-hardware-no-webots-simulation-just-rviz)

## I) Package Structure
* **`start_robot.launch.py`**: The main launch file to start the UR5e driver. It includes parameters for both real hardware and mock hardware, allowing you to switch between them easily.
* `config/` directory: Contains the robots calibration YAML file.

## II) Usage / Launch

### Option A: Real Hardware (UR5e)

> [!Note]
> If you want to start the real UR5e robot, make sure that the **[robot setup](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/robot_setup.html#robot-setup)** on the teach pendant is done and the program with the **external_control** node is created.

Make sure to set the correct **robot_ip** in the command below or in the ``start_robot.launch.py`` file in the ``workcell_control`` package!

```bash
# You can set the robot_ip either via command line argument or directly in the start_robot.launch.py file
ros2 launch workcell_control start_robot.launch.py robot_ip:=<ROBOT_IP_ADDRESS>
# launch_rviz:=true can be set to automatically start RViz to visualize the robot and its movements
```
This will start the ROS 2 driver for the UR5e robot, allowing you to control the physical robot using ROS 2 interfaces. Make sure to follow all safety precautions when working with real robots.

> [!IMPORTANT]
> Start the program with the **external control** node on the teach pendant, to ensure the robot can receive commands from ROS 2.

### Option B: Mock Hardware (no Webots simulation, just RViz)

```bash
ros2 launch workcell_control start_robot.launch.py use_mock_hardware:=true
# launch_rviz:=true can be set to automatically start RViz to visualize the robot and its movements
```
This will start a virtual robot using the ros2_control mock hardware interface. The robot will mirror the commands sent to it, without needing access to physical hardware or the Webots simulation. Set ``launch_rviz:=true`` to visualize the robot in RViz, as shown in the screenshot at the top.

> [!NOTE]
> If you want to use the **Webots simulation** instead, please refer to the **[workcell_simulation package](../workcell_simulation/README.md)**, which includes its own launch file that starts the Webots simulation.
