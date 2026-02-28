# Robot and Environment Descriptions (URDF / Xacro) <!-- omit from toc -->

[![jazzy][jazzy-badge]][jazzy]
[![ubuntu24][ubuntu24-badge]][ubuntu24]

[jazzy-badge]: https://img.shields.io/badge/-ROS%202%20JAZZY-orange?style=flat-square&logo=ros
[jazzy]: https://docs.ros.org/en/jazzy/index.html
[ubuntu24-badge]: https://img.shields.io/badge/-UBUNTU%2024%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu24]: https://releases.ubuntu.com/noble/

This folder contains the physical, visual, and collision models for the complete robotic workcell. The architecture is highly modular, allowing individual components (like grippers or the robot arm) to be swapped out easily.

- [Package Overview](#package-overview)
- [How It Works](#how-it-works)
- [Viewing the Models](#viewing-the-models)


## Package Overview

The models are divided into several independent packages, which are combined into the final assembly:

* **`epick_gripper_description`**: Contains the 3D meshes and Xacro files for the Robotiq EPick vacuum pump.
* **`pisoftgrip_description`**: Contains the 3D meshes and Xacro files for the Piab piSOFTGRIP suction cup.
* **`environment_description`**: Contains the static environment, such as the table, camera mounts, and collision objects.
* **`workcell_description`**: **The top-level assembly package.** This package imports all the macros from the packages above and connects them via TF joints (e.g., attaching the EPick to the UR5e's `tool0`, and the UR5e to the table).

## How It Works

The `workcell_description` acts as the master. Its main file (`workcell.urdf.xacro`) includes the components like this:

1. Instantiates the environment (Table).
2. Instantiates the UR5e arm from the official `ur_description` package and fixes its `base_link` to the table.
3. Instantiates the EPick pump and attaches it to the robot's `tool0`.
4. Instantiates the piSOFTGRIP and attaches it to the EPick.
5. Defines the `pisoftgrip_tcp` (Tool Center Point) used by MoveIt for planning.

## Viewing the Models

You can visualize the completely assembled workcell in RViz without starting any simulation or hardware drivers.

**View the complete assembled workcell:**

```bash
ros2 launch workcell_description view_workcell.launch.py
```

*(Note: The `view_` launch files start a `joint_state_publisher_gui`, allowing you to manually move the joints with sliders to test the kinematic chain and collision geometries).*






