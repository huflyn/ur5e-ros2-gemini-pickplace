# Color Detection Package (`color_detection`) <!-- omit from toc -->

[![jazzy][jazzy-badge]][jazzy]
[![ubuntu24][ubuntu24-badge]][ubuntu24]

[jazzy-badge]: https://img.shields.io/badge/-ROS%202%20JAZZY-orange?style=flat-square&logo=ros
[jazzy]: https://docs.ros.org/en/jazzy/index.html
[ubuntu24-badge]: https://img.shields.io/badge/-UBUNTU%2024%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu24]: https://releases.ubuntu.com/noble/

This package detects colored Lego bricks using a camera stream (RGB + Depth). It generates precise bounding boxes by applying configurable HSV color masks and contour detection to the RGB image. By mapping these 2D bounding boxes to the aligned depth map, it calculates the 3D coordinates of each brick relative to the robot's base frame and provides this data to the ROS 2 network.

![Screenshot of the annotated image stream by the color detection node in RQT.](../docs/images/color_detector_rqt.png)

---

- [I) Package Structure](#i-package-structure)
- [II) Services, Topics \& Custom Messages](#ii-services-topics--custom-messages)
- [III) Configuration (YAML)](#iii-configuration-yaml)
- [IV) Launch color\_detector (Service Node)](#iv-launch-color_detector-service-node)
- [V) Launch color\_detector\_legacy (Topic-based Node)](#v-launch-color_detector_legacy-topic-based-node)
- [VI) Using the HSV Tuner](#vi-using-the-hsv-tuner)
- [VII) How to Add a New Color (e.g., 'orange')](#vii-how-to-add-a-new-color-eg-orange)

---

# I) Package Structure

* **`color_detector.py`**: The main ROS 2 node. It acts as a Multi-threaded Service Server that provides on-demand brick detection (`/detect_bricks`). It uses `tf2` to transform coordinates and dynamically broadcasts TF frames for RViz. It also publishes a continuous live annotated image stream.
* **`color_detector_legacy.py`**: The old version adapted from ROS 1. It continuously processes images and publishes the best target brick to the `/lego_brick_info` topic using a timer.
* **`color_functions.py`**: A pure Python helper module containing the OpenCV logic for HSV masking and contour detection. It is completely independent of ROS.
* **`hsv_tuner.py`**: A standalone ROS 2 GUI tool. It opens an OpenCV window with trackbars, allowing you to fine-tune HSV values in real-time.

---

# II) Services, Topics & Custom Messages

This package relies on the **`brick_interfaces`** package for custom message and service definitions.

**Service Server (`color_detector.py`):**
* `/detect_bricks` (`brick_interfaces/srv/DetectBricks`): Evaluates the current camera frame on demand and returns an array of all valid detected `LegoBrick` objects. 

**Published Topics (`color_detector.py`):**
* `/annotated_image` (`sensor_msgs/Image`): Publishes a live, non-blocking visualization of detected bounding boxes, 3D coordinates, and safe zones at 6 Hz for RQT/RViz.
* `/tf`: Broadcasts individual `TransformStamped` frames for every detected brick (e.g., `brick_red_0`) relative to the robot's base frame.

**Subscribed Topics:**
* Camera Info, Color Image, and Depth Image (configurable via parameters).

**Message Format (`brick_interfaces/msg/LegoBrick`):**
The service response returns an array of these messages.
```text
geometry_msgs/PointStamped position  # Transformed 3D coordinates in the robot's base frame
std_msgs/String color                # The detected color name (e.g., "red", "blue")
float32 camera_distance_mm           # Raw depth distance from the camera lens to the brick
float32 yaw_degrees                  # Calculated brick orientation (0.0 or 30.0 degrees) based on aspect ratio
int32[] bounding_box_px              # Bounding box array [xmin, ymin, xmax, ymax]
```

---

# III) Configuration (YAML)

The configuration for this node relies on parameter files distributed across two locations to cleanly separate global camera settings from local color thresholds:

  * **[Global Workspace Parameters](/workcell/workcell_bringup/README.md#ii-workspace-configuration-yaml) (`workcell_bringup`):** Centralized files (`sim_workspace_parameters.yaml` and `real_workspace_parameters.yaml`) that store the camera topic names, the target `tf2` frames, and the workspace boundaries.
  * **Local Color Parameters (`color_detection`):** The local `hsv_bounds.yaml` file that stores the specific HSV color limits for all detected Lego bricks.

> [!IMPORTANT]
> Before running the node in a new lighting environment, you must calibrate the **HSV bounds** in the local `hsv_bounds.yaml` using the provided tuning tool.

Example `hsv_bounds.yaml` (located in `color_detection/config/`):

```yaml
color_detector_node:
  ros__parameters:
    # --- HSV Color Thresholds ---
    hsv_red_lower: [0, 145, 100]
    hsv_red_upper: [10, 255, 255]

    hsv_yellow_lower: [16, 128, 117]
    hsv_yellow_upper: [53, 255, 255]

    hsv_green_lower: [28, 143, 0]
    hsv_green_upper: [73, 255, 255]

    hsv_blue_lower: [86, 168, 175]
    hsv_blue_upper: [147, 255, 255]
```

---

# IV) Launch color_detector (Service Node)

![Screenshot of the color detection node running in Webots simulation, showing detected bricks highlighted with bounding boxes and their coordinates printed in the terminal.](../docs/images/color_detector_terminal.png)

> [!TIP]
> **Automated Bringup (Recommended):** Instead of opening multiple separate terminals and manually launching each component, you can simply use the master launch files provided in the **[workcell_bringup](/workcell/workcell_bringup/README.md)** package and launch everything with a single command!
> To improve debugging and understanding of the individual components, you can of course launch every component individually.

## Launch Command <!-- omit from toc -->

> [!IMPORTANT]
> **Camera Prerequisite:** This node requires an active camera stream to function. Ensure that either the Webots simulation is running or the real RealSense camera stream is active.

This launches the service server:

```bash
# Launch the color_detector node (append 'use_sim_time:=true' if using Webots simulation)
ros2 launch color_detection color_detector.launch.py 
```
## Interaction <!-- omit from toc -->
To **test the functionality** manually via the terminal, use:

```bash
ros2 service call /detect_bricks brick_interfaces/srv/DetectBricks
```

Launch RViz to **view the live annotated image stream** and **visualize the TF frames** of the detected bricks, as shown in the image above:

```bash
# Launch RViz (append 'use_sim_time:=true' if using Webots simulation)
ros2 launch workcell_application rviz.launch.py
```

Alternatively, you can use RQT to view the `/annotated_image` topic, but it may be less stable than RViz (sometimes no image):

```bash
rqt
```

In RQT go to Plugins → Visualization → Image View, then select the `/annotated_image` topic.

## Edge Margins (Safe Zone) <!-- omit from toc -->

To ensure reliable grasping and accurate depth calculations, the detector implements edge margins on the camera frame. Bricks that touch or cross this margin are deliberately ignored. This applies to the legacy node as well.

  * Default `edge_margin_x`: 100 pixels
  * Default `edge_margin_y`: 1 pixel

These margins are drawn as a red safe-zone rectangle in the `/annotated_image` stream.

---

# V) Launch color_detector_legacy (Topic-based Node)

![Screenshot of the color detection node running in Webots simulation, showing detected bricks highlighted with bounding boxes.](../docs/images/color_detector_legacy_terminal.png)

## Launch Command <!-- omit from toc -->

> [!IMPORTANT]
> The `color_detector_legacy` node only works with the `brick_sorter_legacy` node and vice versa!
> It does not work with the real hardware (gripper control) yet, only simulated in Webots.

> [!IMPORTANT]
> **Camera Prerequisite:** This node requires an active camera stream to function. Ensure that either the Webots simulation is running or the real RealSense camera stream is active.

If you want to try the old version adapted from ROS 1, which continuously processes images and publishes the best target brick to a topic, use this launch command:

```bash
ros2 launch color_detection color_detector_legacy.launch.py use_sim_time:=true # must be true for Webots simulation
```

This node evaluates all targets and publishes only the best target brick to `/lego_brick_info` using a 1.0-second timer.

## Sorting Method (Endless Loop Prevention) <!-- omit from toc -->

By default, the legacy node selects the brick with the shortest camera depth (closest to the lens). If the robot repeatedly fails to grasp a specific target, it can get stuck in an endless loop. To prevent this, you can randomize the target selection by using the launch argument `sort_method:=random`.

---

# VI) Using the HSV Tuner

![Screenshot of the HSV tuner GUI running in Webots simulation, showing the trackbars and live video feed.](../docs/images/hsv_tuner_terminal.png)

> [!IMPORTANT]
> **Camera Prerequisite:** This node requires an active camera stream to function. Ensure that either the Webots simulation is running or the real RealSense camera stream is active.

To find the perfect HSV color thresholds for your environment, use the built-in tuning tool. It opens a live video feed with trackbars and automatically loads the correct camera topics based on your configuration.

```bash
# Launch the HSV tuner (append 'use_sim_time:=true' if using Webots simulation)
ros2 launch color_detection hsv_tuner.launch.py use_sim_time:=true
```

1.  Adjust the trackbars until the `Pure Mask` window clearly shows your target object in solid white and everything else in black.
2.  The node automatically prints the YAML-formatted values to your terminal every 2 seconds.
3.  Copy the printed array values from the terminal, and paste them into your **`hsv_bounds.yaml`** configuration file.

---

# VII) How to Add a New Color (e.g., 'orange')

Adding a new color requires exactly three steps, without touching the core image processing logic:

## Step 1: Add the HSV bounds to your `hsv_bounds.yaml` file <!-- omit from toc -->

```yaml
#   hsv_color_lower: [H, S, V]
#   hsv_color_upper: [H, S, V]

    hsv_orange_lower: [5, 150, 150]
    hsv_orange_upper: [15, 255, 255]
```

## Step 2: Declare and map the parameters <!-- omit from toc -->

In the `__init__` function of your detector node (`color_detector.py`), declare the parameters and map them into the dictionary:

```python
...

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

## Step 3: Add the color to the processing list <!-- omit from toc -->

Inside the `detect_callback` function in `color_detector` (or `image_callback` in `color_detector_legacy`), simply add the string to the list:

```python
colors = ['green', 'yellow', 'red', 'blue', 'orange']
```

---