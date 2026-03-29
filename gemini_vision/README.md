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

![Screenshot of the gemini_vision node running in Webots simulation, showing detected bricks highlighted with bounding boxes in RViz.](../docs/images/gemini_vision_user-prompt-mode_rviz.png)

---

- [I) Package Structure](#i-package-structure)
- [II) Prerequisites \& API Key](#ii-prerequisites--api-key)
- [III) Services, Topics \& Custom Messages](#iii-services-topics--custom-messages)
- [IV) Configuration (Camera and Gemini)](#iv-configuration-camera-and-gemini)
  - [Camera \& Hardware Configuration (YAML)](#camera--hardware-configuration-yaml)
  - [Gemini Configuration (Python)](#gemini-configuration-python)
- [V) Start Gemini Vision](#v-start-gemini-vision)
  - [Launch the Node](#launch-the-node)
  - [Override the Gemini Model](#override-the-gemini-model)
- [VI) Testing the Service via Terminal](#vi-testing-the-service-via-terminal)
- [VII) Reviewing the Gemini Output](#vii-reviewing-the-gemini-output)
  - [Terminal Logs](#terminal-logs)
  - [RViz2 Visualization (Recommended)](#rviz2-visualization-recommended)
  - [RQT Image View (Alternative)](#rqt-image-view-alternative)
- [VIII) Models Overview\[^1\]](#viii-models-overview1)

---

# I) Package Structure

* **`gemini_vision.py`**: The core ROS 2 multi-threaded node. It acts as a Service Server, processes images into JPEG bytes, communicates with the Gemini API, calculates 3D transformations via `tf2`, and streams an annotated visualization.

---

# II) Prerequisites & API Key

Because this node relies on cloud-based AI processing, it requires a valid Google Gemini API key exported as an environment variable (`GEMINI_API_KEY`). 

> [!NOTE]
> Please refer to the **[Gemini API Setup Guide](../GEMINI_API.md)** for detailed instructions on how to generate your API key and permanently add it to your environment. This package assumes the API key setup is already completed.

---

# III) Services, Topics & Custom Messages

**Service Server:**
* `/detect_bricks` (`brick_interfaces/srv/DetectBricks`): Evaluates the current camera frame. You can pass an empty prompt for default detection, or a custom natural language instruction (e.g., "Sort the red bricks to the left"). Returns an array of valid `LegoBrick` objects.

**Published Topics:**
* `/annotated_image` (`sensor_msgs/Image`): Publishes a live, non-blocking visualization at 10 Hz. It draws the bounding boxes, labels, 3D coordinates, and calculated drop-off targets for RViz/RQT.
* `/tf`: Broadcasts individual `TransformStamped` frames for every detected brick relative to the robot's base frame.

**Subscribed Topics:**
* Camera Info, Color Image, and Depth Image (configurable via parameters).

---

# IV) Configuration (Camera and Gemini)

## Camera & Hardware Configuration (YAML)

To maintain consistency, the camera configuration files are centralized in the **`workcell_bringup`** package. Configure the parameters according to your setup, ensuring that the camera topics, TF frames and workspace boundaries match your specific environment (simulation or real hardware).

* **[Global Workspace Parameters](/workcell/workcell_bringup/README.md#ii-workspace-configuration-yaml) (`workcell_bringup`):** Centralized files (`sim_workspace_parameters.yaml` and `real_workspace_parameters.yaml`) that store the camera topic names, the target `tf2` frames, and the workspace boundaries.

## Gemini Configuration (Python)

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

---

# V) Start Gemini Vision

> [!TIP]
> **Automated Bringup (Recommended):** Instead of opening multiple separate terminals and manually launching each component, you can simply use the master launch files provided in the **[workcell_bringup](/workcell/workcell_bringup/README.md)** package and launch everything with a single command!
> To improve debugging and understanding of the individual components, you can of course launch every component individually.

> [!IMPORTANT]
> **Camera Prerequisite:** This node requires an active camera stream to function. Ensure that either the Webots simulation is running or the real RealSense camera stream is active.

## Launch the Node

You can launch the node using the provided launch file:

```bash
# Launch the Gemini Vision node (append 'use_sim_time:=true' if using Webots simulation)
ros2 launch gemini_vision gemini_vision.launch.py
```

## Override the Gemini Model

If you want to use a specific model for a complex task without altering the Python script defaults, pass the **`model`** argument:

```bash
ros2 launch gemini_vision gemini_vision.launch.py model:="gemini-3.1-flash-lite-preview"
```

> [!NOTE]
> Although several models have been tested and are supported, their performance in terms of spatial reasoning and latency varies significantly depending on the specific use case. Availability and rate limits are tied to your usage tier. 
> 
> Please refer to the **Models Overview** below or the official [Gemini API Models documentation](https://ai.google.dev/gemini-api/docs/models) for the most up-to-date information.

---

# VI) Testing the Service via Terminal

Once the node is running, you can trigger the vision pipeline directly from the terminal.

* **Option A: Default Mode**

    Detects all bricks on the table (default prompt).

    ```bash
    ros2 service call /detect_bricks brick_interfaces/srv/DetectBricks
    ```

* **Option B: User Prompt Mode** 
  
    Lets you specify a custom natural language instruction to guide the detection and sorting logic. For example, you can ask it to only detect certain colors, or to calculate specific drop-off locations based on the prompt.

    ```bash
    ros2 service call /detect_bricks brick_interfaces/srv/DetectBricks "{user_prompt: 'Pick the red and blue bricks'}"
    ```

---

# VII) Reviewing the Gemini Output

To evaluate what the Gemini is "seeing" and deciding, you have three primary tools at your disposal:

## Terminal Logs

The most detailed information is printed directly in the terminal where the `gemini_vision` node is running. After every scan, the node outputs:
* Its "Thoughts" (the Gemini's internal reasoning process).
* The detected objects and their colors.
* The exact 3D pick coordinates [X, Y, Z].
* The calculated drop-off targets (if requested via a custom prompt).

## RViz2 Visualization (Recommended)

To view the live annotated image stream (bounding boxes, coordinates, target crosshairs) and the dynamically generated 3D `tf2` frames in real-time, launch the pre-configured RViz workspace:

* **For Simulation (Webots):**

    ```bash
    ros2 launch workcell_application rviz.launch.py use_sim_time:=true
    ```

* **For Real Hardware:**

    ```bash
    ros2 launch workcell_application rviz.launch.py
    ```

## RQT Image View (Alternative)
Alternatively, you can view just the raw 2D image overlay using RQT. 

```bash
rqt
```
*In RQT, navigate to **Plugins → Visualization → Image View**, then select the `/annotated_image` topic from the dropdown.*

> [!WARNING]
> RQT can sometimes drop frames or fail to load the image stream entirely. If the screen remains gray, use the RViz2 method above, which is generally much more stable.

---

# VIII) Models Overview[^1]

The Gemini API provides access to a range of powerful language models, each designed for specific use cases, and with different request limits (you can see your current usage and limits in [Google AI Studio](https://aistudio.google.com/app/rate-limit)).

For this project, the following models are recommended for experimentation (descriptions adapted from the official Gemini API [Model Overview](https://ai.google.dev/gemini-api/docs/models)):

| Model | Description | RPD (Free tier) | Latest Update[^1] | Knowledge cutoff |
| --- | --- | :---: | :---: | :---: |
| [gemini-3-flash-preview](https://ai.google.dev/gemini-api/docs/models/gemini-3-flash-preview) | The best model in the world for multimodal understanding, and our most powerful agentic and vibe-coding model yet, delivering richer visuals and deeper interactivity, all built on a foundation of state-of-the-art reasoning. | 20 | December 2025 | January 2025 |
| [gemini-3.1-flash-lite-preview](https://ai.google.dev/gemini-api/docs/models/gemini-3.1-flash-lite-preview) | Our most cost-efficient multimodal model, offering the fastest performance for high-frequency, lightweight tasks. Gemini 3.1 Flash-Lite is best for high-volume agentic tasks, simple data extraction, and extremely low-latency applications where budget and speed are the primary constraints. | 500 | March 2026 | January 2025 |
| [gemini-robotics-er-1.5-preview](https://ai.google.dev/gemini-api/docs/models/gemini-robotics-er-1.5-preview) | Gemini Robotics-ER 1.5 is a vision-language model (VLM) that brings Gemini's agentic capabilities to robotics. It's designed for advanced reasoning in the physical world, allowing robots to interpret complex visual data, perform spatial reasoning, and plan actions from natural language commands. | 20 | September 2025 | January 2025 |

[^1]: Information from March 2026, please refer to the official [Gemini API Models documentation](https://ai.google.dev/gemini-api/docs/models) for the most up-to-date information on available models and their capabilities.

