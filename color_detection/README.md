# Color Detection Package (`color_detection`) <!-- omit from toc -->

[![jazzy][jazzy-badge]][jazzy]
[![ubuntu24][ubuntu24-badge]][ubuntu24]

[jazzy-badge]: https://img.shields.io/badge/-ROS%202%20JAZZY-orange?style=flat-square&logo=ros
[jazzy]: https://docs.ros.org/en/jazzy/index.html
[ubuntu24-badge]: https://img.shields.io/badge/-UBUNTU%2024%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu24]: https://releases.ubuntu.com/noble/


This package detects colored Lego bricks using a camera stream (RGB + Depth), calculates their 3D coordinates relative to the robot's base frame, and publishes them to the ROS 2 network.

- [Package Structure](#package-structure)
- [Published Topics \& Custom Messages](#published-topics--custom-messages)
  - [Message Format (`color_detection_msgs/LegoBrick.msg`)](#message-format-color_detection_msgslegobrickmsg)
- [Configuration \& Camera Setup (YAML)](#configuration--camera-setup-yaml)
- [Launch color\_detection](#launch-color_detection)
  - [Simulation](#simulation)
  - [Real Camera](#real-camera)
  - [Sorting Method (Endless Loop Prevention)](#sorting-method-endless-loop-prevention)
  - [Edge Margin (Safe Zone)](#edge-margin-safe-zone)
- [Using the HSV Tuner](#using-the-hsv-tuner)
- [How to Add a New Color (e.g., 'orange')](#how-to-add-a-new-color-eg-orange)


## Package Structure

* **`color_detector.py`**: The main ROS 2 node. It subscribes to the camera topics, matches the pixel coordinates with the depth image, and uses `tf2` to transform the coordinates into the robot's frame (`ur5e_base_link`).
* **`color_functions.py`**: A pure Python helper module containing the OpenCV logic for HSV masking and contour detection. It is completely independent of ROS.
* **`hsv_tuner.py`**: A standalone ROS 2 GUI tool. It opens an OpenCV window with trackbars, allowing you to fine-tune HSV values in real-time. It prints the tuned values directly to the terminal in a YAML-friendly format.


## Published Topics & Custom Messages

The `color_detector.py` node processes the images silently in the background to keep the terminal logs clean. It publishes the consolidated data of all detected bricks to the `/lego_brick_info` topic.

This package requires the **`color_detection_msgs`** package, which defines the custom message structure used for communication.

### Message Format (`color_detection_msgs/LegoBrick.msg`)
```text
geometry_msgs/PointStamped position  # Transformed 3D coordinates in the robot's base frame
std_msgs/String color                # The detected color name (e.g., "red", "blue")
float32 camera_distance_mm           # Raw depth distance from the camera lens to the brick
```

## Configuration & Camera Setup (YAML)

We use parameter files in the `config/` directory to seamlessly switch between Webots simulation and real-world hardware, and to manage color thresholds:

* **`sim_params.yaml` / `real_params.yaml`**: Store the camera topic names and the target `tf2` frames.
* **`hsv_bounds.yaml`**: Centrally stores the HSV color thresholds for all detected colors.

> [!IMPORTANT]
> Before running the node on a new setup, you must adjust the camera topics and the `tf2` frames in the respective parameters YAML file. Example (`sim_params.yaml`):

```yaml
color_detector_node:
  ros__parameters:
    # --- Camera Topics ---
    camera_info_topic: '/webots_realsense/depth/image_rect_raw/camera_info'
    depth_image_topic: '/webots_realsense/depth/image_rect_raw/image'
    color_image_topic: '/webots_realsense/color/image_raw/image_color'

    # Frame of the camera for TF transformations
    camera_frame: 'd415_sim_optical_frame'

    # Target frame for the 3D coordinates
    robot_base_frame: 'ur5e_base_link'
```

## Launch color_detection

**Launch Arguments**

- `use_sim` (bool, default: false): Set to true to use simulation topics and parameters.
- `sort_method` (string, default: "closest", on y-axis): Method to sort detected bricks. Options: "closest" and "random".

### Simulation
```bash
ros2 launch color_detection color_detector.launch.py use_sim:=true
```

### Real Camera
```bash
ros2 launch color_detection color_detector.launch.py
```

### Sorting Method (Endless Loop Prevention)

By **default**, the detector **sorts bricks deterministically by their Y-coordinate**. If the robot repeatedly fails to grasp a specific brick (e.g., due to camera distortion at the edges), it can get stuck in an endless loop.

To prevent this, you can **randomize the sorting order** by using the launch argument **`sort_method:=random`**. The node will print a summary of the active configuration to the terminal upon launch.

**Launch with randomized sorting (Simulation):**
```bash
ros2 launch color_detection color_detector.launch.py use_sim:=true sort_method:=random
```

**Launch with randomized sorting (Real Camera):**
```bash
ros2 launch color_detection color_detector.launch.py sort_method:=random
```

### Edge Margin (Safe Zone)

To ensure reliable grasping and accurate center-point calculations, the detector implements a **25-pixel safe zone** around the image borders. Bricks that touch or cross this margin (e.g., partially visible bricks at the edge of the camera frame) are deliberately ignored. 

This prevents the robot from calculating faulty TCP coordinates based on incomplete contours or distortion.


## Using the HSV Tuner

To find the perfect HSV color thresholds for your environment, use the built-in tuning tool. It opens a live video feed with trackbars and automatically loads the correct camera topics based on your configuration.

**Run the tuner (Simulation):**
```bash
ros2 launch color_detection hsv_tuner.launch.py use_sim:=true
```

**Tuning Workflow:**

1. Adjust the trackbars until the `Pure Mask` window clearly shows your target object in solid white and everything else in black.
2. The node automatically prints the YAML-formatted values to your terminal every 2 seconds.
3. Press `Ctrl+C` to stop the tuner, copy the printed array values from the terminal, and paste them into your **`hsv_bounds.yaml`** configuration file.

## How to Add a New Color (e.g., 'orange')

Adding a new color requires exactly three steps, without touching the core image processing logic:

**Step 1: Add the HSV bounds to your `hsv_bounds.yaml` file**

```yaml
#   hsv_color_lower: [H, S, V]
#   hsv_color_upper: [H, S, V]

    hsv_orange_lower: [5, 150, 150]
    hsv_orange_upper: [15, 255, 255]
```

**Step 2: Declare and map the parameters in `color_detector.py`**

In the `__init__` function, declare the parameters and map them into the dictionary:

```python
        self.declare_parameter('hsv_orange_lower', [0, 0, 0])
        self.declare_parameter('hsv_orange_upper', [255, 255, 255])
        
        # Add to self.color_bounds dictionary:
        self.color_bounds = {
            # ... existing colors ...
            'orange': {
                'lower': np.array(self.get_parameter('hsv_orange_lower').value),
                'upper': np.array(self.get_parameter('hsv_orange_upper').value)
            }
        }
```

**Step 3: Add the color to the processing list**

At the bottom of the `image_callback` function in `color_detector.py`, simply add the string to the list:

```python
        colors = ['green', 'yellow', 'red', 'blue', 'orange']
```

---
