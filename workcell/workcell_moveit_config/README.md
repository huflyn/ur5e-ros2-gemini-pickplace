# Workcell MoveIt Configuration Package (`workcell_moveit_config`) <!-- omit from toc -->

[![jazzy][jazzy-badge]][jazzy]
[![ubuntu24][ubuntu24-badge]][ubuntu24]

[jazzy-badge]: https://img.shields.io/badge/-ROS%202%20JAZZY-orange?style=flat-square&logo=ros
[jazzy]: https://docs.ros.org/en/jazzy/index.html
[ubuntu24-badge]: https://img.shields.io/badge/-UBUNTU%2024%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu24]: https://releases.ubuntu.com/noble/


This package contains the semantic representation (SRDF), kinematics, and motion planning configurations for the UR5e workcell. It was primarily generated using the MoveIt 2 Setup Assistant and adapted for the specific collision environment and tool setup (gripper).

![Screenshot of the RViz interface with MoveIt plugin and Webots simulation.](/docs/images/moveit_webots.png)

---

- [I) Package Structure](#i-package-structure)
- [II) Usage](#ii-usage)
  - [Step 1: Start the Robot (Real or Simulated)](#step-1-start-the-robot-real-or-simulated)
    - [Option A: Webots Simulation](#option-a-webots-simulation)
    - [Option B: Real Hardware (UR5e)](#option-b-real-hardware-ur5e)
  - [Step 2: Start MoveIt](#step-2-start-moveit)

---

# I) Package Structure

* **`config/`**: Contains the SRDF and YAML configuration files.
* **`launch/`**: Contains the launch files to start the MoveIt 2 move_group node and the RViz setup with the MoveIt plugin. Both launch files were modified to accept a `use_sim_time` argument for proper synchronization with the Webots simulation clock.

---

# II) Usage

Normally, MoveIt is launched automatically by the higher-level application launch files. For isolated testing, you can start 3 launch files in individual terminals and in this order:

## Step 1: Start the Robot (Real or Simulated)

### Option A: Webots Simulation

```bash
ros2 launch workcell_simulation simulation.launch.py
```

> [!IMPORTANT]
> **``use_sim_time``:** When using the Webots simulation, you MUST append `use_sim_time:=true` to **all subsequent launch commands**! This ensures proper time synchronization between the simulator, MoveIt, and all ROS nodes.

### Option B: Real Hardware (UR5e)

If you want to start a real robot, make sure that the **[robot setup](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/robot_setup.html#robot-setup)** is done and **external_control** is active on the robot.
Make sure to set the correct **robot_ip** in the command below or in the ``start_robot.launch.py`` file in the ``workcell_control`` package!

```bash
# You can set the robot_ip either via command line argument or directly in the start_robot.launch.py file
ros2 launch workcell_control start_robot.launch.py robot_ip:=<ROBOT_IP_ADDRESS>
```

## Step 2: Start MoveIt

1. ### ``move_group`` Node

    Now that we have a robot running (either simulated or real), we can launch the MoveIt **move_group** node. 

    ```bash
    ros2 launch workcell_moveit_config move_group.launch.py
    # use_sim_time:=true is required if using the Webots simulation, but can be left out for real hardware
    ```

    If everything went well you should see the output: “You can start planning now!”.

2. ### RViz with MoveIt Plugin

    To interact with the MoveIt setup, you can start RViz with the correct setup file:

    ```bash
    ros2 launch workcell_moveit_config moveit_rviz.launch.py
    # use_sim_time:=true is required if using the Webots simulation, but can be left out for real hardware
    ```
If everything is running correctly, you should see the UR5e robot in the RViz environment with the MoveIt plugin enabled, allowing you to visualize the robot state, plan motions, and execute trajectories in both the simulated and real environments. The image at the top of this README shows the RViz interface with the MoveIt plugin and the Webots simulation running in parallel.

---
