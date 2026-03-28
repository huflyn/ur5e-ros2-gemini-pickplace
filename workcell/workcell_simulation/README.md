# Workcell Simulation Package (`workcell_simulation`) <!-- omit from toc -->

[![jazzy][jazzy-badge]][jazzy]
[![ubuntu24][ubuntu24-badge]][ubuntu24]

[jazzy-badge]: https://img.shields.io/badge/-ROS%202%20JAZZY-orange?style=flat-square&logo=ros
[jazzy]: https://docs.ros.org/en/jazzy/index.html
[ubuntu24-badge]: https://img.shields.io/badge/-UBUNTU%2024%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu24]: https://releases.ubuntu.com/noble/


This package provides the Webots simulation environment for the Pick-and-Place application. It contains the Webots world file (`.wbt`), the simulated sensor (Intel RealSense camera), and the launch file required to spin up the virtual UR5e workcell.

![Screenshot of the Webots simulation environment, showing the UR5e robot, the table with bricks, and the Realsense camera.](/docs/images/webots_world_overlays.png)

---

- [I) Package Structure](#i-package-structure)
- [II) Usage](#ii-usage)

---

# I) Package Structure

* **`worlds/`**: Contains the Webots world file.
* **`launch/`**: Contains the launch file to start the Webots simulation with the correct world and ROS 2 interfaces.
* **`config/`**: Contains the configuration files for the simulated robot and sensors.
* **`protos/`**: Contains the Webots PROTO files for custom objects like the bricks and the camera.
* **`meshes/`**: Contains the 3D mesh files used in the simulation, such as the bricks and the camera model.

---

# II) Usage

To start the complete simulation (shown in the screenshot above), use the following command:

```bash
ros2 launch workcell_simulation simulation.launch.py 
# use_sim_time:=true is set by default in the launch file
```

> [!IMPORTANT]
> **``use_sim_time``:** When using the Webots simulation, you MUST append `use_sim_time:=true` to **all subsequent launch commands**! This ensures proper time synchronization between the simulation and all ROS nodes.

The simulation runs automatically and you should see the following output in the Webots console:

![Screenshot of the Webots simulation console output after starting.](/docs/images/webots_world_console.png)

Wait until you see that the `scaled_joint_trajectory_controller` has been successfully activated, which indicates that the robot is ready to receive commands. The relevant lines in the console output will look like this:
```bash
...

[webots_controller_ur5e-4] [INFO] [1774613576.573235034] [controller_manager]: Activating controllers: [ scaled_joint_trajectory_controller ]
[webots_controller_ur5e-4] [INFO] [1774613576.600943920] [controller_manager]: Successfully switched controllers!
[spawner-7] [INFO] [1774613576.616463985] [spawner_scaled_joint_trajectory_controller]: Configured and activated scaled_joint_trajectory_controller
[INFO] [spawner-7]: process has finished cleanly [pid 145270]
```

Now the simulation is ready for the Pick-and-Place application. It can also be used for testing different prompts with Gemini Vision and the Webots environment to see how well the vision system can identify and locate the bricks and drop-off locations and how accurate it follows the instructions.

---
