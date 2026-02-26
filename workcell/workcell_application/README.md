# Workcell Application Package (ROS 2 Jazzy) <!-- omit from toc -->

[![jazzy][jazzy-badge]][jazzy]
[![ubuntu24][ubuntu24-badge]][ubuntu24]

[jazzy-badge]: https://img.shields.io/badge/-ROS%202%20JAZZY-orange?style=flat-square&logo=ros
[jazzy]: https://docs.ros.org/en/jazzy/index.html
[ubuntu24-badge]: https://img.shields.io/badge/-UBUNTU%2024%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu24]: https://releases.ubuntu.com/noble/


This package manages high-level robot control for the Brick Sorter application using the **MoveIt 2 Python API**. It coordinates motion planning, gripper actuation, and sorting logic based on perception data.

- [Package Structure](#package-structure)
- [Configuration (YAML)](#configuration-yaml)
- [Launch Files](#launch-files)
- [Workflow \& Robustness](#workflow--robustness)
- [Usage of `brick_sorter.py`](#usage-of-brick_sorterpy)
  - [Brick Sorter Workflow](#brick-sorter-workflow)
  - [Starting the Script](#starting-the-script)
- [Usage of `test_moveit_api.py`](#usage-of-test_moveit_apipy)
  - [MoveIt API Test Workflow](#moveit-api-test-workflow)
  - [Starting the Script](#starting-the-script-1)


## Package Structure

* **`brick_sorter.py`**: The main application node. It handles the pick-and-place state machine, listens to `/lego_brick_info`, and executes trajectories. It is a replication of the ROS 1 `ur5e_moveit_script_erweitert.py`.
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

## Launch Files
The launch file automatically loads the Robot Description (URDF/SRDF) and application parameters.

**Start Brick Sorter:**

```bash
ros2 launch workcell_application brick_sorter.launch.py

```

**Run MoveIt API Test:**

```bash
ros2 launch workcell_application test_moveit_api.launch.py

```

## Workflow & Robustness

The application follows a strict sequence to ensure reliable operation in both simulation and reality:

1. **Initialization**: Moves to a predefined `ready` pose to clear the camera view.
2. **Approach & Grasp**: Targets the brick's X/Y coordinates and descends to a fixed Z-height.
3. **Error Handling**: If a trajectory fails (e.g., controller timeout or joint deviation), the node aborts the current cycle, resets the brick data, and returns to the `ready` pose.
4. **Transport & Release**: Moves to the color-coded zone defined in the YAML file.
5. **Damping**: Includes short `time.sleep` intervals between moves to allow physics/controllers to settle, preventing "Start point deviates" errors.

---
## Usage of `brick_sorter.py`

### Brick Sorter Workflow

The application follows a strict sequence to ensure reliable operation in both simulation and reality:

1. **Initialization**: Moves to a predefined `ready` pose to clear the camera view.
2. **Approach & Grasp**: Targets the brick's X/Y coordinates and descends to a fixed Z-height.
3. **Error Handling**: If a trajectory fails (e.g., controller timeout or joint deviation), the node aborts the current cycle, resets the brick data, and returns to the `ready` pose.
4. **Transport & Release**: Moves to the color-coded zone defined in the YAML file.
5. **Damping**: Includes short `time.sleep` intervals between moves to allow physics/controllers to settle, preventing "Start point deviates" errors.

### Starting the Script

You need to open three terminals to run the full application:

1. **Terminal 1**: Start the Robot

   - **Option 1:** Start the Gazebo **Simulation**  

        ```bash
        ros2 launch workcell_simulation simulation.launch.py
        ```

   - **Option 2:** Start the **Real Robot** (after sourcing the appropriate workspace)  

        ```bash
        ros2 launch workcell_bringup bringup.launch.py
        ```

2. **Terminal 2**: Start the Color Detector
   
    ```bash
    ros2 launch color_detection color_detector.launch.py
    ```


3. **Terminal 2**: Start the Brick Sorter Application
   
    ```bash
    ros2 launch workcell_application brick_sorter.launch.py
    ```

---

## Usage of `test_moveit_api.py`

### MoveIt API Test Workflow

This script is designed to verify the connectivity and demonstrate the basic functionality of the MoveIt 2 Python API.
This includes:
- Initializing the MoveIt 2 interface.
- Planning a simple trajectory to a predefined pose.
  - set goal state with predefined string (e.g., "ready" pose, defined in the MoveIt Config SRDF)
  - set goal state with `PoseStamped` message (e.g., target brick position coordinates)
  - Single-Pipeline Planning (`PlanRequestParameters`)
  - Multi-Pipeline Planning (`MultiPipelinePlanRequestParameters`)
- Executing the planned trajectory.

### Starting the Script

You can run the MoveIt API test to verify connectivity and basic planning:

```bash
ros2 launch workcell_application test_moveit_api.launch.py
```