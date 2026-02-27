# Workcell MoveIt Configuration (`workcell_moveit_config`)

[![jazzy][jazzy-badge]][jazzy]
[![ubuntu24][ubuntu24-badge]][ubuntu24]

[jazzy-badge]: https://img.shields.io/badge/-ROS%202%20JAZZY-orange?style=flat-square&logo=ros
[jazzy]: https://docs.ros.org/en/jazzy/index.html
[ubuntu24-badge]: https://img.shields.io/badge/-UBUNTU%2024%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu24]: https://releases.ubuntu.com/noble/


This package contains the semantic representation (SRDF), kinematics, and motion planning configurations for the UR5e workcell. It was primarily generated using the MoveIt 2 Setup Assistant and adapted for the specific collision environment and tool setup (gripper).

**Important Modification:** The launch files `move_group.launch.py` and `moveit_rviz.launch.py` were manually extended to accept a `use_sim_time` argument. This ensures that MoveIt synchronizes correctly with the simulated clock when used alongside the Webots simulation.

## Usage

Normally, MoveIt is launched automatically by the higher-level application launch files. For isolated testing, it can be started like this:

> [!IMPORTANT]
> Make sure the robot, real or simulated, is running first before starting MoveIt, otherwise MoveIt will not receive the necessary joint states and will throw errors.

To startup the complete system, you’ll have to start 3 launch files in individual terminals.

### Step 1: Start the Robot Driver (Real or Simulated)

- #### Option A: Webots Simulation

  ```bash
  ros2 launch workcell_webots simulation.launch.py
  ```

- #### Option B: Mock Hardware (no Webots, just RViz and MoveIt)

  ```bash
  # Make sure to set use_mock_hardware:=true in the command below if you want to use mock hardware
  ros2 launch workcell_control start_robot.launch.py use_mock_hardware:=true
  ```

- #### Option C: Real Hardware (UR5e)

  If you want to start a real robot, make sure that the **[robot setup](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/robot_setup.html#robot-setup)** is done and **external_control** is active on the robot.
  Make sure to set the correct **robot_ip** in the command below or in the ``start_robot.launch.py`` file in the ``workcell_control`` package!

  ```bash
  # You can set the robot_ip either via command line argument or directly in the start_robot.launch.py file
  ros2 launch workcell_control start_robot.launch.py robot_ip:=<ROBOT_IP_ADDRESS>
  ```

### Step 2: Start MoveIt

1. #### ``move_group`` Node

Now that we have a robot running (either simulated or real), we can launch the MoveIt **move_group** node.
Use the `use_sim_time` argument if you are using the Webots simulation to ensure proper time synchronization.
 
```bash
ros2 launch workcell_moveit_config move_group.launch.py # use_sim_time:=true # if using simulation
```

If everything went well you should see the output: “You can start planning now!”.

2. #### RViz with MoveIt Plugin

To interact with the MoveIt setup, you can start RViz with the correct setup file:

```bash
ros2 launch workcell_moveit_config moveit_rviz.launch.py
```