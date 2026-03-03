# Workcell Application Package (`workcell_application`) <!-- omit from toc -->

[![jazzy][jazzy-badge]][jazzy]
[![ubuntu24][ubuntu24-badge]][ubuntu24]

[jazzy-badge]: https://img.shields.io/badge/-ROS%202%20JAZZY-orange?style=flat-square&logo=ros
[jazzy]: https://docs.ros.org/en/jazzy/index.html
[ubuntu24-badge]: https://img.shields.io/badge/-UBUNTU%2024%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu24]: https://releases.ubuntu.com/noble/


This package manages high-level robot control for the Brick Sorter application using the **MoveIt 2 Python API**. It coordinates motion planning, gripper actuation, and sorting logic based on perception data.

- [Package Structure](#package-structure)
- [Configuration (YAML)](#configuration-yaml)
- [Usage of `brick_sorter.py`](#usage-of-brick_sorterpy)
  - [Workflow](#workflow)
  - [Starting the Application](#starting-the-application)
- [Usage of `verify_alignment.py`](#usage-of-verify_alignmentpy)
  - [Workflow](#workflow-1)
  - [Starting the Script](#starting-the-script)
- [Usage of `test_moveit_api.py`](#usage-of-test_moveit_apipy)
  - [Starting the Script](#starting-the-script-1)
  - [Features Tested](#features-tested)
- [Known Limitations \& Future Work](#known-limitations--future-work)


# Package Structure

* **`brick_sorter.py`**: The main application node. It handles the pick-and-place state machine, listens to `/lego_brick_info`, and executes trajectories.
* **`verify_alignment.py`**: A verification and calibration script to manually test workspace coordinates, TCP accuracy, and robot alignment using an interactive ROS topic trigger.
* **`test_moveit_api.py`**: A verification script used to test basic MoveIt 2 planning and connectivity.

# Configuration (YAML)

Sorting locations and safe heights are managed via **`config/sorter_params.yaml`**. This allows for layout adjustments without modifying the source code.

```yaml
brick_sorter_node:
  ros__parameters:
    # Z-Heights
    hover_height: 0.20
    dropoff_height: 0.10
    grasp_z_offset: 0.0 # Optional: To ensure a secure grasp by going slightly below the detected Z
    
    # Drop-off coordinates: [X, Y]
    dropoff_yellow: [-0.27, 0.450]
    dropoff_red: [0.27, 0.450]
    dropoff_green: [-0.27, 0.350]
    dropoff_blue: [0.27, 0.350]

```

---

# Usage of `brick_sorter.py`

This is an **advanced replication of the ROS 1 ur5e_moveit_script_erweitert.py**. It utilizes a hybrid motion planning architecture, seamlessly switching between **OMPL** for joint-space travel and the **Pilz Industrial Motion Planner** (LIN) for strict Cartesian vertical movements. It also features **dynamic fallback logic** to prevent execution failures during singularities.

 It continuously sorts detected Lego bricks by executing the following specific workflow:

## Workflow

1. **Clear View:** Moves to a predefined `ready` pose so the camera has an unobstructed view of the workspace.
2. **Lock Perception:** Once a brick is detected, it locks the incoming ROS message queue to prevent processing stale data ("ghost bricks") during movement.
3. **Pick:** Moves above the brick's X/Y coordinates, descends to the defined Z-height, and activates the vacuum gripper.
4. **Place:** Transports the brick to the color-coded drop-off zone (defined in the YAML) and releases the gripper.
5. **Flush & Reset:** Returns to the `ready` pose, clears all old camera messages from the queue, and unlocks perception for the next brick.
*(Note: If a trajectory fails due to controller timeouts or collisions, the cycle safely aborts and resets to step 1).*

## Starting the Application

You need to open 3 terminals to run the full application:

### Step 1: Start the Robot Driver (Real or Simulated) <!-- omit from toc -->

This will start the ROS 2 node that interfaces with the UR5e, either in real hardware mode or in Webots simulation.

<details>
  <summary><b>Option A: Webots Simulation</b></summary>

This will launch the Webots simulation of the workcell. 

Make sure you have Webots and the `webots_ros2` package installed. 

#### Launch Command <!-- omit from toc -->
  
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

#### Launch Command <!-- omit from toc -->

```bash
# You can set the robot_ip either via command line argument or directly in the start_robot.launch.py file
ros2 launch workcell_control start_robot.launch.py robot_ip:=<ROBOT_IP_ADDRESS>
```
</details>


### Step 2: Start the Perception Pipeline <!-- omit from toc -->

This node processes the camera stream and publishes the 3D coordinates of detected bricks.

For details see the [**color_detection README**](color_detection/README.md).

#### Launch Command <!-- omit from toc -->

```bash
ros2 launch color_detection color_detector.launch.py # add launch arguments as needed, see below
```

#### Launch Arguments <!-- omit from toc -->

You can append the following arguments to the launch command to customize the behavior:

- `use_sim_time` (bool, default: false): Set to true to use simulation topics and parameters, and the simulation clock (`/clock` topic).
- `sort_method` (string, default: "closest"): Method to select the target brick. By default, it selects the brick closest to the camera lens based on the depth map. Options: "closest" and "random".
- `verbose` (bool, default: false): Set to true to print detailed logs of detected bricks and their coordinates.

Example with arguments:

```bash
ros2 launch color_detection color_detector.launch.py use_sim_time:=true sort_method:=random
```

> [!IMPORTANT] 
> **Using Real Hardware:** You need to adjust the **camera topics** and **frames** in the `real_params.yaml` file before running the node.


### Step 3: Start the Application <!-- omit from toc -->

This will start the brick_sorter node, which listens to the perception data and executes the pick-and-place logic.

#### Launch Command <!-- omit from toc -->

```bash
ros2 launch workcell_application brick_sorter.launch.py # add launch arguments as needed, see below
```

#### Launch Arguments <!-- omit from toc -->

You can append the following argument to the launch command to customize the behavior:

- `use_sim_time` (bool, default: false): Set `use_sim_time:=true` to run with simulation camera topics and parameters, and the simulation clock (`/clock` topic).

---



# Usage of `verify_alignment.py`

This script is a manual tool used for hardware commissioning. It allows you to move the robot step-by-step through predefined test poses to verify workspace coordinates and TCP alignment safely.

## Workflow

1. Start the script via the launch command below.
2. The robot will wait for a manual trigger before moving to the next test pose.
3. Open a separate terminal to send the trigger command:
    ```bash
    ros2 topic pub --once /next_step std_msgs/msg/Empty
    ```

    It is recommended to create an alias for quick triggering:
    ```bash
    alias next="ros2 topic pub --once /next_step std_msgs/msg/Empty"
    ```
    You can then simply execute `next` in the separate terminal to move to the next pose without typing the full command each time.


## Starting the Script

Launch Arguments:

- `use_sim_time` (bool, default: false): Use `use_sim_time:=true` to run with simulation camera topics and parameters, and the simulation clock (`/clock` topic).

```bash
ros2 launch workcell_application verify_alignment.launch.py
```

---

# Usage of `test_moveit_api.py`

This script is designed to verify the connectivity and demonstrate the basic functionality of the MoveIt 2 Python API.

## Starting the Script

```bash
ros2 launch workcell_application test_moveit_api.launch.py
```

## Features Tested

* Initializing the MoveIt 2 interface.
* Planning a simple trajectory to a predefined pose.
  * set goal state with predefined string (e.g., "ready" pose, defined in the MoveIt Config SRDF)
  * set goal state with `PoseStamped` message (e.g., target brick position coordinates)
  * Single-Pipeline Planning (`PlanRequestParameters`)
  * Multi-Pipeline Planning (`MultiPipelinePlanRequestParameters`)
* Executing the planned trajectory.

---

# Known Limitations & Future Work

* **Grasping Accuracy:** The Intel RealSense camera is currently mounted horizontally. Depth estimation noise on the camera's Z-axis translates directly into grasping inaccuracies on the robot's X/Y table plane. A top-down (bird's-eye) camera perspective is planned for future iterations to improve pick precision.
