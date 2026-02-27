# Workcell Simulation Package (`workcell_simulation`)

[![jazzy][jazzy-badge]][jazzy]
[![ubuntu24][ubuntu24-badge]][ubuntu24]

[jazzy-badge]: https://img.shields.io/badge/-ROS%202%20JAZZY-orange?style=flat-square&logo=ros
[jazzy]: https://docs.ros.org/en/jazzy/index.html
[ubuntu24-badge]: https://img.shields.io/badge/-UBUNTU%2024%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu24]: https://releases.ubuntu.com/noble/


This package provides the Webots simulation environment for the Brick Sorter application. It contains the 3D world files (`.wbt`), the simulated sensors (such as the Intel RealSense camera), and the launch files required to spin up the virtual UR5e workcell.

## Usage

To start the complete simulation (robot, environment, and camera):

```bash
ros2 launch workcell_simulation simulation.launch.py
```


