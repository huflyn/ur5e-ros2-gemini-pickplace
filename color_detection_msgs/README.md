# Color Detection Messages Package (`color_detection_msgs`)

[![jazzy][jazzy-badge]][jazzy]
[![ubuntu24][ubuntu24-badge]][ubuntu24]

[jazzy-badge]: https://img.shields.io/badge/-ROS%202%20JAZZY-orange?style=flat-square&logo=ros
[jazzy]: https://docs.ros.org/en/jazzy/index.html
[ubuntu24-badge]: https://img.shields.io/badge/-UBUNTU%2024%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu24]: https://releases.ubuntu.com/noble/

This package defines the custom ROS 2 interface messages used by the Lego Brick Sorting application. 

## Custom Messages

### `LegoBrick.msg`
Used to publish the consolidated data of a detected Lego brick, including its calculated 3D position in the robot's coordinate system, its color, and the raw depth distance from the camera.

**Definition:**
```text
geometry_msgs/PointStamped position  # Transformed 3D coordinates in the robot's base frame
std_msgs/String color                # The detected color name (e.g., "red", "blue")
float32 camera_distance_mm           # Raw depth distance from the camera lens to the brick
```

## Build Instructions

When modifying the `.msg` files, you must rebuild this package before building any dependent Python packages:

```bash
cd ros2_ws # Navigate to your ROS 2 workspace
colcon build --packages-select color_detection_msgs
source install/setup.bash
```