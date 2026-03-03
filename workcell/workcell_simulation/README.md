# Workcell Simulation Package (`workcell_simulation`) <!-- omit from toc -->

[![jazzy][jazzy-badge]][jazzy]
[![ubuntu24][ubuntu24-badge]][ubuntu24]

[jazzy-badge]: https://img.shields.io/badge/-ROS%202%20JAZZY-orange?style=flat-square&logo=ros
[jazzy]: https://docs.ros.org/en/jazzy/index.html
[ubuntu24-badge]: https://img.shields.io/badge/-UBUNTU%2024%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu24]: https://releases.ubuntu.com/noble/


This package provides the Webots simulation environment for the Brick Sorter application. It contains the 3D world files (`.wbt`), the simulated sensors (such as the Intel RealSense camera), and the launch files required to spin up the virtual UR5e workcell.

![Screenshot of the Webots simulation environment, showing the UR5e robot, the table with bricks, and the Realsense camera.](/docs/images/webots_world_overlays.png)

# Package Structure

* **`worlds/`**: Contains the Webots world file.
* **`launch/`**: Contains the launch files to start the Webots simulation with the correct world and ROS 2 interfaces.
* **`config/`**: Contains the configuration files for the simulated robot and sensors.
* **`protos/`**: Contains the Webots PROTO files for custom objects like the bricks and the camera.
* **`meshes/`**: Contains the 3D mesh files used in the simulation, such as the bricks and the camera model.

# Usage

To start the complete simulation (robot, environment, and camera):

```bash
ros2 launch workcell_simulation simulation.launch.py
```


