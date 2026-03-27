# Brick Interfaces Package (`brick_interfaces`) <!-- omit from toc -->

[![jazzy][jazzy-badge]][jazzy]
[![ubuntu24][ubuntu24-badge]][ubuntu24]

[jazzy-badge]: https://img.shields.io/badge/-ROS%202%20JAZZY-orange?style=flat-square&logo=ros
[jazzy]: https://docs.ros.org/en/jazzy/index.html
[ubuntu24-badge]: https://img.shields.io/badge/-UBUNTU%2024%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu24]: https://releases.ubuntu.com/noble/

This package defines the custom ROS 2 messages (`.msg`) and services (`.srv`) used for communication between the vision systems (``color_detector.py`` and ``gemini_vision.py``) and the Pick-and-Place Application (``pick_and_place.py``).

- [Custom Message: `LegoBrick.msg`](#custom-message-legobrickmsg)
- [Custom Service: `DetectBricks.srv`](#custom-service-detectbrickssrv)

# Custom Message: `LegoBrick.msg`

Used to encapsulate all consolidated data of a detected object, including its 3D position, orientation, color, bounding box, and optional AI-generated drop-off coordinates.

**Definition:**
```text
geometry_msgs/PointStamped position         # Transformed 3D center coordinates in the robot's base frame
std_msgs/String color                       # The detected color or object name (e.g., "red", "brown bread")
float64 camera_distance_mm                  # Raw depth distance from the camera lens to the object
float64 yaw_degrees                         # Calculated from orientation of the object (e.g., 0.0 or 30.0 degrees)
int32[4] bounding_box_px                    # 2D bounding box in the camera image [ymin, xmin, ymax, xmax]
bool has_user_dropoff                       # True if a specific custom drop-off location was defined for this object
geometry_msgs/Point user_dropoff_position   # Transformed 3D coordinates of the custom drop-off location
```

# Custom Service: `DetectBricks.srv`

Used to trigger an on-demand image scan and object detection process. It allows passing optional text instructions to the ``gemini_vision`` node to guide the detection and placement logic.

**Definition:**
```text
# Request
string user_prompt  # Optional: Instructions for the vision model (e.g., "Put all bricks on the yellow circle")
---
# Response
bool success        # True if the detection process completed without errors
string message      # Status or error message from the vision node
LegoBrick[] bricks  # Array of all successfully detected and processed objects
```
---