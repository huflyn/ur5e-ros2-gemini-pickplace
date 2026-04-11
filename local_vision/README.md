# local_vision (Experimental LM Studio Integration) <!-- omit from toc -->

[![jazzy](https://img.shields.io/badge/-ROS%202%20JAZZY-orange?style=flat-square&logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/index.html)
[![ubuntu24](https://img.shields.io/badge/-UBUNTU%2024%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/noble/)
[![lmstudio](https://img.shields.io/badge/-LM%20STUDIO-512BD4?style=flat-square)](https://lmstudio.ai/)

This package is an **experimental "drop-in replacement"** for the `gemini_vision` package. It enables AI-driven object detection completely locally, offline, and privacy-friendly using locally hosted Vision-Language Models (VLMs) via [LM Studio](https://lmstudio.ai/).

> [!CAUTION]
> This is an experimental feature branch intended primarily for scientific evaluation purposes (e.g., response comparisons between cloud AI and edge AI). It is not integrated into the main branch and may be revised or removed in the future.

---

- [Installation \& Prerequisites](#installation--prerequisites)
- [LM Studio Configuration (Host PC)](#lm-studio-configuration-host-pc)
- [Usage](#usage)
- [Manual Test Calls (Service Call)](#manual-test-calls-service-call)

---

# Installation & Prerequisites

This package simulates an OpenAI interface to connect to the local LM Studio server.

```bash
# Install the OpenAI SDK
pip install --user openai --break-system-packages
```

# LM Studio Configuration (Host PC)

To allow the ROS 2 laptop to access LM Studio, the server must be configured as follows:

1. **Load a Vision Model:** Download a multimodal VLM (e.g., from the `gemma` family).
2. **Developer Tab:** Open the Developer view on the left sidebar in LM Studio (or press CTRL + 2).
3. **Network Sharing:** In the **Server Settings**, enable the **`Serve on Local Network`** option.
4. **JIT Loading:** Optional: Also enable **"Just in time model loading"** to allow models to load dynamically.
5. **Start:** Start the server (default port is `1234`).

# Usage

Start the vision node using the provided launch file. It automatically binds to the `workcell_bringup` package and uses the same service definition (`/detect_objects`) as the main system.

**Standard Start:**
```bash
ros2 launch local_vision lm_studio_vision.launch.py
```

**Advanced Launch Arguments:**
You can enable simulation time and override the desired model via an index (defined in the script) or by using the exact model name:

```bash
# Example: Enable sim-time and load model index 0 (e.g., gemma-4-e4b)
ros2 launch local_vision lm_studio_vision.launch.py use_sim_time:=true model:=0

# Example: Load a specific model via its direct name
ros2 launch local_vision lm_studio_vision.launch.py model:="qwen/qwen3.5-9b"
```

# Manual Test Calls (Service Call)

Once the node is running, image processing can be triggered via the terminal. The `pick_and_place` orchestrator does not need to be modified.

```bash
# 🅰️ Default Mode (detects all objects on the table)
ros2 service call /detect_objects object_interfaces/srv/DetectObjects

# 🅱️ User Prompt Mode (with specific text instructions)
ros2 service call /detect_objects object_interfaces/srv/DetectObjects "{user_prompt: 'Pick the red bricks'}"
```
