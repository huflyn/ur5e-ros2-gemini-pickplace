# Color Detection Package (ROS 2 Jazzy)

[![jazzy][jazzy-badge]][jazzy]
[![ubuntu24][ubuntu24-badge]][ubuntu24]

[jazzy-badge]: https://img.shields.io/badge/-ROS%202%20JAZZY-orange?style=flat-square&logo=ros
[jazzy]: https://docs.ros.org/en/jazzy/index.html
[ubuntu24-badge]: https://img.shields.io/badge/-UBUNTU%2024%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu24]: https://releases.ubuntu.com/noble/


This package detects colored Lego bricks using a camera stream (RGB + Depth), calculates their 3D coordinates relative to the robot's base frame, and publishes them to the ROS 2 network.



## Package Structure

* **`color_detector.py`**: The main ROS 2 node. It subscribes to the camera topics, matches the pixel coordinates with the depth image, and uses `tf2` to transform the coordinates into the robot's frame (`ur5e_base_link`).
* **`color_functions.py`**: A pure Python helper module containing the OpenCV logic for HSV masking and contour detection. It is completely independent of ROS.
* **`hsv_tuner.py`**: A standalone ROS 2 GUI tool. It opens an OpenCV window with trackbars, allowing you to fine-tune HSV values in real-time. It prints the tuned values directly to the terminal in a YAML-friendly format.



## Configuration & Camera Setup (YAML)

We use parameter files (**`config/sim_topics.yaml`** and **`config/real_topics.yaml`**) to seamlessly switch between Webots simulation and real-world hardware. These files store topic names, camera frames, and the HSV color thresholds.

> [!IMPORTANT]
> Before running the node on a new setup, you must adjust the camera topics and the `tf2` frame in the YAML file:

```yaml
color_detector_node:
  ros__parameters:
    # 1. Update these to match your camera's output topics:
    camera_info_topic: '/webots_realsense/depth/image_rect_raw/camera_info'
    depth_image_topic: '/webots_realsense/depth/image_rect_raw/image'
    color_image_topic: '/webots_realsense/color/image_raw/image_color'
    
    # 2. Update this to the optical frame of your camera (Z-axis pointing forward):
    camera_frame_id: 'd415_sim_optical_frame'
```

## Launch color_detection

**Simulation:**
```bash
ros2 launch color_detection color_detector.launch.py use_sim:=true
```

**Real Camera:**
```bash
ros2 launch color_detection color_detector.launch.py
```



## Using the HSV Tuner

To find the perfect HSV color thresholds for your environment, use the built-in tuning tool. It opens a live video feed with trackbars. 

You can pass the correct camera topic directly via the command line:

**Run the tuner (Simulation):**
```bash
ros2 run color_detection hsv_tuner --ros-args -p color_image_topic:=/webots_realsense/color/image_raw/image_color
```

**Run the tuner (Real Camera):**

```bash
ros2 run color_detection hsv_tuner --ros-args -p color_image_topic:=/camera/color/image_raw  # adjust topic name
```

**Tuning Workflow:**

1. Adjust the trackbars until the `Pure Mask` window clearly shows your target object in solid white and everything else in black.
2. The node automatically prints the YAML-formatted values to your terminal every 2 seconds.
3. Press `Ctrl+C` to stop the tuner, copy the printed array values from the terminal, and paste them into your active YAML configuration file (e.g., `sim_topics.yaml`).



## How to Add a New Color (e.g., 'orange')

Adding a new color requires exactly three steps, without touching the core image processing logic:

**Step 1: Add the HSV bounds to your YAML file**

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
