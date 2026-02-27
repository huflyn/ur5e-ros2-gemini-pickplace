# Workcell Control Package (`workcell_control`)

[![jazzy][jazzy-badge]][jazzy]
[![ubuntu24][ubuntu24-badge]][ubuntu24]

[jazzy-badge]: https://img.shields.io/badge/-ROS%202%20JAZZY-orange?style=flat-square&logo=ros
[jazzy]: https://docs.ros.org/en/jazzy/index.html
[ubuntu24-badge]: https://img.shields.io/badge/-UBUNTU%2024%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu24]: https://releases.ubuntu.com/noble/


This package manages the connection to the physical UR5e hardware. It contains the launch files necessary to start the official `ur_robot_driver` with the correct calibration parameters, network settings, and ROS 2 controllers for this specific workcell.

## Usage / Launch

- ### Option A: Mock Hardware (no Webots, just RViz and MoveIt)

  ```bash
  # Make sure to set use_mock_hardware:=true in the command below if you want to use mock hardware
  ros2 launch workcell_control start_robot.launch.py use_mock_hardware:=true
  ```
  This will start a simulated robot using the ros2_control mock hardware interface. The robot will mirror the commands sent to it, allowing you to test your MoveIt 2 configuration without needing access to physical hardware or the Webots simulation.

- ### Option B: Real Hardware (UR5e)

  If you want to start a real robot, make sure that the **[robot setup](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/robot_setup.html#robot-setup)** is done and **external_control** is active on the robot.
  Make sure to set the correct **robot_ip** in the command below or in the ``start_robot.launch.py`` file in the ``workcell_control`` package!

  ```bash
  # You can set the robot_ip either via command line argument or directly in the start_robot.launch.py file
  ros2 launch workcell_control start_robot.launch.py robot_ip:=<ROBOT_IP_ADDRESS>
  ```
  This will start the ROS 2 driver for the UR5e robot, allowing you to control the physical robot using ROS 2 interfaces. Make sure to follow all safety precautions when working with real robots.