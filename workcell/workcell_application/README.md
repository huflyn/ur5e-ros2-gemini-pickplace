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
    - [Terminal 1: Start the Robot](#terminal-1-start-the-robot)
    - [Terminal 2: Start the Color Detector](#terminal-2-start-the-color-detector)
    - [Terminal 3: Start the Brick Sorter Application](#terminal-3-start-the-brick-sorter-application)
- [Usage of `verify_alignment.py`](#usage-of-verify_alignmentpy)
  - [Workflow](#workflow-1)
  - [Starting the Script](#starting-the-script)
- [Usage of `test_moveit_api.py`](#usage-of-test_moveit_apipy)
  - [Starting the Script](#starting-the-script-1)
  - [Features Tested](#features-tested)


## Package Structure

* **`brick_sorter.py`**: The main application node. It handles the pick-and-place state machine, listens to `/lego_brick_info`, and executes trajectories. It is a replication of the ROS 1 `ur5e_moveit_script_erweitert.py` with enhanced robustness.
* **`verify_alignment.py`**: A verification and calibration script to manually test workspace coordinates, TCP accuracy, and robot alignment using an interactive ROS topic trigger.
* **`test_moveit_api.py`**: A verification script used to test basic MoveIt 2 planning and connectivity.

## Configuration (YAML)

Sorting locations and safe heights are managed via **`config/sorter_params.yaml`**. This allows for layout adjustments without modifying the source code.

```yaml
brick_sorter_node:
  ros__parameters:
    # Z-Heights
    hover_height: 0.20
    dropoff_height: 0.12
    grasp_z_offset: 0.0 # Optional: To ensure a secure grasp by going slightly below the detected Z
    
    # Drop-off coordinates: [X, Y]
    dropoff_yellow: [0.27, 0.558]
    dropoff_red: [0.27, 0.438]
    dropoff_green: [0.27, 0.318]
    dropoff_blue: [0.27, 0.198]

```

---

## Usage of `brick_sorter.py`

This is the main automated sorting application. It continuously sorts detected Lego bricks by executing the following specific workflow:

### Workflow

1. **Clear View:** Moves to a predefined `ready` pose so the camera has an unobstructed view of the workspace.
2. **Lock Perception:** Once a brick is detected, it locks the incoming ROS message queue to prevent processing stale data ("ghost bricks") during movement.
3. **Pick:** Moves above the brick's X/Y coordinates, descends to the defined Z-height, and activates the vacuum gripper.
4. **Place:** Transports the brick to the color-coded drop-off zone (defined in the YAML) and releases the gripper.
5. **Flush & Reset:** Returns to the `ready` pose, clears all old camera messages from the queue, and unlocks perception for the next brick.
*(Note: If a trajectory fails due to controller timeouts or collisions, the cycle safely aborts and resets to step 1).*

### Starting the Application

You need to open three terminals to run the full application:

#### Terminal 1: Start the Robot

- **Option 1:** Start the **Simulation**  

  ```bash
  ros2 launch workcell_simulation simulation.launch.py
  ```

- **Option 2:** Start the **Real Robot** 

  ```bash
  # You can set the robot_ip either via command line argument or directly in the start_robot.launch.py file
  ros2 launch workcell_control start_robot.launch.py robot_ip:=<ROBOT_IP_ADDRESS>
  ```

#### Terminal 2: Start the Color Detector

Launch Arguments:

- `use_sim` (bool, default: false): Set to true to use simulation topics and parameters.
- `sort_method` (string, default: "closest", on y-axis): Method to sort detected bricks. Options: "closest" and "random".

```bash
ros2 launch color_detection color_detector.launch.py
```


#### Terminal 3: Start the Brick Sorter Application
   
```bash
ros2 launch workcell_application brick_sorter.launch.py
```

---

## Usage of `verify_alignment.py`

This script is a manual tool used for hardware commissioning. It allows you to move the robot step-by-step through predefined test poses to verify workspace coordinates and TCP alignment safely.

### Workflow

1. Start the script via the launch command below.
2. The robot will wait for a manual trigger before moving to the next test pose.
3. Open a separate terminal to send the trigger command:

```bash
ros2 topic pub --once /next_step std_msgs/msg/Empty
```

```bash
# Recommended: Create an alias for quick triggering
alias next="ros2 topic pub --once /next_step std_msgs/msg/Empty"

# Execute the alias to move to the next pose
next
```

### Starting the Script

```bash
ros2 launch workcell_application verify_alignment.launch.py
```

---

## Usage of `test_moveit_api.py`

This script is designed to verify the connectivity and demonstrate the basic functionality of the MoveIt 2 Python API.

### Starting the Script

```bash
ros2 launch workcell_application test_moveit_api.launch.py
```

### Features Tested

* Initializing the MoveIt 2 interface.
* Planning a simple trajectory to a predefined pose.
  * set goal state with predefined string (e.g., "ready" pose, defined in the MoveIt Config SRDF)
  * set goal state with `PoseStamped` message (e.g., target brick position coordinates)
  * Single-Pipeline Planning (`PlanRequestParameters`)
  * Multi-Pipeline Planning (`MultiPipelinePlanRequestParameters`)
* Executing the planned trajectory.
