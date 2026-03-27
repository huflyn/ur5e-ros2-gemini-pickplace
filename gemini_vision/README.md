# Gemini Vision Package (`gemini_vision`) <!-- omit from toc -->

[![jazzy][jazzy-badge]][jazzy-link]
[![ubuntu24][ubuntu24-badge]][ubuntu24-link]
[![gemini][gemini-badge]][gemini-link]

[jazzy-badge]: https://img.shields.io/badge/-ROS%202%20JAZZY-orange?style=flat-square&logo=ros&logoColor=white
[jazzy-link]: https://docs.ros.org/en/jazzy/index.html
[ubuntu24-badge]: https://img.shields.io/badge/-UBUNTU%2024%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu24-link]: https://releases.ubuntu.com/noble/
[gemini-badge]: https://img.shields.io/badge/-GEMINI%20API-7C4DFF?style=flat-square&logo=googlegemini&logoColor=white
[gemini-link]: https://ai.google.dev/gemini-api/docs


This package provides a highly capable, **AI-driven vision system for the pick-and-place application**. It leverages the **Gemini API** (supporting models like Gemini 3 Flash and Gemini Robotics-ER 1.5) to analyze RGB-D camera streams. It can detect specified objects (like Lego bricks), map 2D bounding boxes to 3D spatial coordinates, and interpret complex natural language instructions to dynamically calculate custom drop-off locations on a table surface, if specified.

INSERT_IMAGE

- [I) Package Structure](#i-package-structure)
- [II) Prerequisites \& API Key](#ii-prerequisites--api-key)
- [III) Services, Topics \& Custom Messages](#iii-services-topics--custom-messages)
- [IV) Configuration (Camera and Gemini)](#iv-configuration-camera-and-gemini)
  - [Camera \& Hardware Configuration (YAML)](#camera--hardware-configuration-yaml)
  - [Gemini Configuration (Python)](#gemini-configuration-python)
- [V) Launch Gemini Vision](#v-launch-gemini-vision)
- [VI) Testing the Service via Terminal](#vi-testing-the-service-via-terminal)
- [VII) Models Overview\[^1\]](#vii-models-overview1)


## I) Package Structure

* **`gemini_vision.py`**: The core ROS 2 multi-threaded node. It acts as a Service Server, processes images into JPEG bytes, communicates with the Gemini API, calculates 3D transformations via `tf2`, and streams an annotated visualization.


## II) Prerequisites & API Key

Because this node relies on cloud-based AI processing, it requires a valid Google Gemini API key exported as an environment variable (`GEMINI_API_KEY`). 

> [!NOTE]
> Please refer to the **[Gemini API Setup Guide](../GEMINI_API.md)** for detailed instructions on how to generate your API key and permanently add it to your environment. This package assumes the API key setup is already completed.


## III) Services, Topics & Custom Messages

**Service Server:**
* `/detect_bricks` (`brick_interfaces/srv/DetectBricks`): Evaluates the current camera frame. You can pass an empty prompt for default detection, or a custom natural language instruction (e.g., "Sort the red bricks to the left"). Returns an array of valid `LegoBrick` objects.

**Published Topics:**
* `/annotated_image` (`sensor_msgs/Image`): Publishes a live, non-blocking visualization at 10 Hz. It draws the bounding boxes, labels, 3D coordinates, and calculated drop-off targets for RViz/RQT.
* `/tf`: Broadcasts individual `TransformStamped` frames for every detected brick relative to the robot's base frame.

**Subscribed Topics:**
* Camera Info, Color Image, and Depth Image (configurable via parameters).


## IV) Configuration (Camera and Gemini)

### Camera & Hardware Configuration (YAML)

To maintain consistency, the camera configuration files are centralized in the **`workcell_bringup`** package.

* **`workcell_bringup/config/`** **`sim_camera_parameters.yaml`** and **`real_camera_parameters.yaml`**: These files store the camera topic names and the target `tf2` frames. This allows the entire workcell to easily switch between Webots simulation and real-world hardware.

Example `sim_camera_parameters.yaml`:

```yaml
/**:
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

### Gemini Configuration (Python)

While you can override the AI model temporarily via launch arguments, the default behavior of the vision system is configured directly at the top of the **`gemini_vision.py`** script. 

You can permanently change the default model, adjust the "thinking" levels (which affect response time and reasoning depth), and modify the internal prompts by editing the constants block:

```python
# --- GEMINI CONFIGURATION ---

GEMINI_MODELS = [
    "gemini-3-flash-preview",
    "gemini-3.1-flash-lite-preview",
    "gemini-robotics-er-1.5-preview"
]
GEMINI_MODEL = GEMINI_MODELS[2] # Set your default model here

# Thinking levels determine the reasoning depth for Gemini 3 models.
GEMINI_THINKING_LEVEL_DEFAULT = GEMINI_THINKING_LEVELS[2]
GEMINI_THINKING_BUDGET = -1 # For Robotics-ER 1.5

# Prompts define what the AI looks for and how it formats the output
GEMINI_DEFAULT_PROMPT = textwrap.dedent("""...""")
GEMINI_SYSTEM_PROMPT = textwrap.dedent("""...""")
```


## V) Launch Gemini Vision

You can launch the node using the provided launch file. It connects to the central configuration automatically.

* **Option A: Simulation (Webots)**

    When running the simulation in Webots, you **must** append the `use_sim_time` argument so the node synchronizes with the simulated ROS clock:
    ```bash
    ros2 launch gemini_vision gemini_vision.launch.py use_sim_time:=true
    ```

* **Option B: Real Hardware (UR5e & RealSense)**
    ```bash
    ros2 launch gemini_vision gemini_vision.launch.py
    ```

**Override the Gemini Model:**

If you want to use a specific model for a complex task without altering the Python script defaults, pass the **`model`** argument:

```bash
ros2 launch gemini_vision gemini_vision.launch.py model:="gemini-3.1-flash-lite-preview"
```

> [!NOTE]
> Although several models have been tested and are supported, their performance in terms of spatial reasoning and latency varies significantly depending on the specific use case. Availability and rate limits are tied to your usage tier. 
> 
> Please refer to the **Models Overview** below or the official [Gemini API Models documentation](https://ai.google.dev/gemini-api/docs/models) for the most up-to-date information.


## VI) Testing the Service via Terminal

Once the node is running, you can trigger the vision pipeline directly from the terminal.

* **Option A: Default Mode** (Detects all bricks on the table):
    ```bash
    ros2 service call /detect_bricks brick_interfaces/srv/DetectBricks
    ```

* **Option B: User Prompt Mode** (Instructs the AI to perform specific spatial reasoning):
    ```bash
    ros2 service call /detect_bricks brick_interfaces/srv/DetectBricks "{user_prompt: 'Pick the red bricks and blue bricks'}"
    ```

## VII) Models Overview[^1]

The Gemini API provides access to a range of powerful language models, each designed for specific use cases, and with different request limits (you can see your current usage and limits in [Google AI Studio](https://aistudio.google.com/app/rate-limit)).

For this project, the following models are recommended for experimentation (descriptions adapted from the official Gemini API [Model Overview](https://ai.google.dev/gemini-api/docs/models)):

| Model | Description | RPD (Free tier) | Latest Update[^1] | Knowledge cutoff |
| --- | --- | :---: | :---: | :---: |
| [gemini-3-flash-preview](https://ai.google.dev/gemini-api/docs/models/gemini-3-flash-preview) | The best model in the world for multimodal understanding, and our most powerful agentic and vibe-coding model yet, delivering richer visuals and deeper interactivity, all built on a foundation of state-of-the-art reasoning. | 20 | December 2025 | January 2025 |
| [gemini-3.1-flash-lite-preview](https://ai.google.dev/gemini-api/docs/models/gemini-3.1-flash-lite-preview) | Our most cost-efficient multimodal model, offering the fastest performance for high-frequency, lightweight tasks. Gemini 3.1 Flash-Lite is best for high-volume agentic tasks, simple data extraction, and extremely low-latency applications where budget and speed are the primary constraints. | 500 | March 2026 | January 2025 |
| [gemini-robotics-er-1.5-preview](https://ai.google.dev/gemini-api/docs/models/gemini-robotics-er-1.5-preview) | Gemini Robotics-ER 1.5 is a vision-language model (VLM) that brings Gemini's agentic capabilities to robotics. It's designed for advanced reasoning in the physical world, allowing robots to interpret complex visual data, perform spatial reasoning, and plan actions from natural language commands. | 20 | September 2025 | January 2025 |

[^1]: Information from March 2026, please refer to the official [Gemini API Models documentation](https://ai.google.dev/gemini-api/docs/models) for the most up-to-date information on available models and their capabilities.
