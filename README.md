# Using Google Gemini Robotics-ER 1.5 (Gemini API) with Universal Robots UR5e in ROS2 Humble

![ROS2](https://img.shields.io/badge/ROS_2-Humble-349eeb.svg) ![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-e95420.svg)

- [I) Prerequisites](#i-prerequisites)
- [II) Environment Setup](#ii-environment-setup)
  - [Note: Shell Environment](#note-shell-environment)
  - [Important: Locale ``en_US.UTF-8``](#important-locale-en_usutf-8)
- [III) Install Software (Framework/Tool/Package/etc.)](#iii-install-software-frameworktoolpackageetc)
  - [1. ros2\_control (Docs)](#1-ros2_control-docs)
  - [2. Universal Robots ROS2 Driver (Docs, GitHub)](#2-universal-robots-ros2-driver-docs-github)
  - [3. Realsense ROS Wrapper (Docs, GitHub)](#3-realsense-ros-wrapper-docs-github)
  - [4. Google Gen AI SDK (Docs)](#4-google-gen-ai-sdk-docs)
- [IV) Gemini API](#iv-gemini-api)
  - [Create Gemini API Key (Docs)](#create-gemini-api-key-docs)
  - [Google GenAI SDK Installation (Docs)](#google-genai-sdk-installation-docs)
  - [Setting the API key as an environment variable (Docs)](#setting-the-api-key-as-an-environment-variable-docs)
  - [Keep your API key secure (Docs)](#keep-your-api-key-secure-docs)
  - [Make your first request (Docs)](#make-your-first-request-docs)
- [V) URDF/XACRO Files](#v-urdfxacro-files)
  - [Packages Overview](#packages-overview)
    - [`my_robot_cell_description`](#my_robot_cell_description)
    - [`epick_description`](#epick_description)
    - [`pisoftgrip_description`](#pisoftgrip_description)
  - [Folder Structure](#folder-structure)
- [X) Documentation and References](#x-documentation-and-references)

---


## I) Prerequisites

Ensure you have the following installed before proceeding:

- [(Ubuntu 22.04 LTS)](https://releases.ubuntu.com/jammy/)
- [(ROS 2 Humble)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [(Universal Robot UR5e)](https://www.universal-robots.com/)
- [(obotiq EPick Vakuumpumpe)](https://robotiq.com/products/vacuum-grippers#EPick)
- [(Piab piSOFTGRIP)](https://www.piab.com/suction-cups-and-soft-grippers/soft-grippers/pisoftgrip-vacuum-driven-soft-gripper-/sg.x)
- [(Intel RealSense 415D camera)](https://www.intel.com/content/www/us/en/products/sku/128256/intel-realsense-depth-camera-d415/specifications.html)

---


## II) Environment Setup


### <span style="color: dodgerblue;">Note:</span> Shell Environment

**Bash** is the standard shell on most Ubuntu systems and is used throughout this guide.

* If you are using **Bash**, use the ``~/.bashrc`` file.
* If you are using **Zsh**, please replace instances of ``.bashrc`` with ``.zshrc`` in all following commands.

Check your current shell by running:

```bash
echo $0
```


### <span style="color: purple;">Important:</span> Locale ``en_US.UTF-8``

Some packages and tools may require the ``en_US.UTF-8`` locale to function correctly (using a dot ``.`` as the decimal separator). If your system is set to a different locale (e.g. using a comma ``,``), you might encounter issues during installation or runtime. To force the locale to ``en_US.UTF-8``, follow these steps:

```bash
# 1. Check current settings (optional)
locale

# 2. Install locales
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8

# 3. Permanent configuration via .bashrc
# It forces English/UTF-8 for ALL locale variables
echo "export LC_ALL=en_US.UTF-8" >> ~/.bashrc

# 4. Load changes
source ~/.bashrc

# 5. Verify (Everything should now be set to en_US.UTF-8)
locale
```

```bash
LANG=en_US.UTF-8
LANGUAGE=
LC_CTYPE="en_US.UTF-8"
LC_NUMERIC="en_US.UTF-8"
LC_TIME="en_US.UTF-8"
LC_COLLATE="en_US.UTF-8"
LC_MONETARY="en_US.UTF-8"
LC_MESSAGES="en_US.UTF-8"
LC_PAPER="en_US.UTF-8"
LC_NAME="en_US.UTF-8"
LC_ADDRESS="en_US.UTF-8"
LC_TELEPHONE="en_US.UTF-8"
LC_MEASUREMENT="en_US.UTF-8"
LC_IDENTIFICATION="en_US.UTF-8"
LC_ALL=en_US.UTF-8
```

---


## III) Install Software (Framework/Tool/Package/etc.)


### 1. ros2_control ([Docs](https://control.ros.org/humble/index.html#))

The ros2_control is a framework for real-time control of robots using ROS 2.

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```


### 2. Universal Robots ROS2 Driver ([Docs](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/index.html), [GitHub](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble))

The ROS 2 driver packages for Universal Robots manipulators to control a Universal Robots arm from an external application using ROS 2 (ur_robot_driver).

```bash
sudo apt install ros-humble-ur
```


### 3. Realsense ROS Wrapper ([Docs](https://dev.realsenseai.com/docs/docs-get-started), [GitHub](https://github.com/realsenseai/realsense-ros))

ROS Wrapper for RealSense(TM) Cameras

**1. Install latest RealSense™ SDK 2.0**
Option 1: Install librealsense2 debian package from RealSense servers
- [Linux Debian Installation Guide](https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)
  - In this case treat yourself as a developer: make sure to follow the instructions to also install librealsense2-dev and librealsense2-dkms packages


Option 2: Install librealsense2 (without graphical tools and examples) debian package from ROS servers (Foxy EOL distro is not supported by this option):
  
1. Configure your Ubuntu repositories ([Docs](https://wiki.ros.org/Installation/Ubuntu/Sources))
   
   - Setup your sources.list to accept software from packages.ros.org.
   ```bash
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   ```
   - Setup your keys

   ```bash
   sudo apt install curl # if you haven't already installed curl
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   ```

2. Install all realsense ROS packages

    ```bash
    sudo apt install ros-humble-librealsense2*
    ```

**2. Install ROS Wrapper for RealSense™ cameras**

Option 1: Install debian package from ROS servers (Foxy EOL distro is not supported by this option):

1. Configure your Ubuntu repositories ([Docs](https://wiki.ros.org/Installation/Ubuntu/Sources))
   - Already done in previous step for librealsense2 packages (Option 2), so you can skip this step if you followed the previous instructions
 
2.  Install all realsense ROS packages
    ```bash
    sudo apt install ros-humble-librealsense2*
    ```

Option 2: Install from source
- See [instructions](https://github.com/realsenseai/realsense-ros?tab=readme-ov-file#option-2-install-from-source) in the GitHub repository.

### 4. Google Gen AI SDK ([Docs](https://ai.google.dev/gemini-api/docs/quickstart))

Google Gen AI Python SDK provides an interface for developers to integrate Google's generative models into their Python applications

Using Python 3.9+, install the google-genai package using the following pip command:

```bash
pip install -q -U google-genai
```

---


## IV) Gemini API

### Create Gemini API Key ([Docs](https://ai.google.dev/gemini-api/docs/api-key))

To use the Gemini API, you need an API key. Create and manage your keys in [*Google AI Studio*](https://aistudio.google.com/app/apikey).

### Google GenAI SDK Installation ([Docs](https://ai.google.dev/gemini-api/docs/quickstart))

Using Python 3.9+, install the google-genai package using the following pip command:

```bash
pip install -q -U google-genai
```

### Setting the API key as an environment variable ([Docs](https://ai.google.dev/gemini-api/docs/api-key#set-api-env-var))

Bash is a common Linux and macOS terminal configuration. You can check if you have a configuration file for it by running the following command:

```bash
~/.bashrc
```

If the response is "No such file or directory", you will need to create this file and open it by running the following commands, or use zsh:

```bash
touch ~/.bashrc
open ~/.bashrc
```

Next, you need to set your API key by adding the following export command at the end of .bashrc:

```bash
export GEMINI_API_KEY=<YOUR_API_KEY_HERE>
```

After saving the file, apply the changes by running:

```bash
source ~/.bashrc
```

Verify the environment variable is set by running:

```bash
echo $GEMINI_API_KEY
```

It should print your API key.

### Keep your API key secure ([Docs](https://ai.google.dev/gemini-api/docs/api-key#security))

Your API key is like a password that allows access to your Google Cloud resources. Keep it secure and do not share it publicly or commit it to version control systems. If you believe your API key has been compromised, revoke it immediately in the Google AI Studio and generate a new one.

### Make your first request ([Docs](https://ai.google.dev/gemini-api/docs/quickstart#make-first-request))

Here is a simple example of how to use the Google GenAI SDK to make a request to the Gemini API with Gemini Robotics-ER 1.5 model:

```python
import os
from google import genai
from google.genai import types

# Set Google API key from environment variable
GOOGLE_API_KEY  = os.environ.get('GEMINI_API_KEY')

# Initialize the GenAI client
client = genai.Client(api_key=GOOGLE_API_KEY)

# Define model and prompt
MODEL_ID = "gemini-robotics-er-1.5-preview"
PROMPT = "Are you there?"

# Generate content / API Call
response = client.models.generate_content(
    model=MODEL_ID, 
    contents=PROMPT
)

# Print the response
print(response.text)
```

Safe the code in a file named `gemini_api_test.py` and run it using the following command:

```bash
python3 gemini_api_test.py
```

You should see a response from the Gemini Robotics-ER model printed in the terminal, similar to the following:

```bash
Yes, I'm here! How can I assist you today?
```

---

## V) URDF/XACRO Files

The URDF/XACRO files for the Universal Robots UR5e with Robotiq EPick vacuum gripper and Piab piSOFTGRIP are located in the `descriptions` folder. They are organized into separate packages to ensure modularity and reusability.

### Packages Overview

- **`epick_description`**: Contains the URDF models and mesh files for the Robotiq EPick vacuum gripper.
- **`pisoftgrip_description`**: Contains the URDF models and mesh files for the Piab piSOFTGRIP.
- **`my_robot_cell_description`**: The main description package that combines the robot, grippers, and environment into a complete workcell.
- **Note on UR5e**: The base robot description is provided by the [Universal Robots ROS2 Driver](#2-universal-robots-ros2-driver-docs-github) ([ur_description package](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_description/doc/index.html)).

The project follows ROS "best practices" by separating the macro definitions (blueprints) from the instantiation files.

#### `my_robot_cell_description`

- **`urdf/my_robot_cell_macro.xacro`**: Defines the modular blueprint of the complete workcell, aggregating the environment, robot arm, and grippers into a single reusable component. It handles the internal connections and joints between all sub-components without creating a standalone robot instance.
- **`urdf/my_robot_cell.urdf.xacro`**: Serves as the top-level entry point that instantiates the workcell macro and anchors it to a `world` link. It allows for the injection of arguments (like `ur_type`) and is the file directly loaded by launch files and the `robot_state_publisher`.
- **`urdf/environment_macro.xacro`**: Defines the static environment (table, walls, robot mount) to keep the main robot macro clean and focused on the kinematic chain.

#### `epick_description`

- **`urdf/robotiq_epick_model_macro.xacro`**: The macro definition for the vacuum gripper. It includes the visual/collision meshes and defines the `epick_end_effector` link, which serves as the attachment point for suction cups or other tools (like the piSOFTGRIP).

#### `pisoftgrip_description`

- **`urdf/pisoftgrip_macro.xacro`**: The macro definition for the soft gripper fingers. It is designed to be attached to the `epick_end_effector` link.

### Folder Structure

```text
descriptions/
├── epick_description/
│   ├── meshes/
│   └── urdf/
│       └── robotiq_epick_model_macro.xacro
├── pisoftgrip_description/
│   ├── meshes/
│   └── urdf/
│       └── pisoftgrip_macro.xacro
└── my_robot_cell_description/
    ├── launch/
    │   └── view_robot.launch.py
    └── urdf/
        ├── environment_macro.xacro
        ├── my_robot_cell_macro.xacro
        └── my_robot_cell.urdf.xacro
```

---

## X) Documentation and References

- [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)
- [ROS 2 Control](https://control.ros.org/humble/index.html)
- [MoveIt 2](https://moveit.picknik.ai/main/index.html#)

- [Universal Robots ROS 2 Driver](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/index.html)
- [Universal Robots ROS 2 Driver GitHub](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble)
- [Universal Robots Client Library GitHub](https://github.com/UniversalRobots/Universal_Robots_Client_Library) EVTL NICHT NOTWENDIG
- [Universal Robots ROS 2 Description GitHub](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/humble)

- [ROS Wrapper for RealSense(TM) Cameras](https://ai.google.dev/gemini-api/docs/quickstart#make-first-request)

- [ROS 2 driver for the Robotiq EPick gripper](https://github.com/PickNikRobotics/ros2_epick_gripper)

- [Google Gemini API](https://ai.google.dev/gemini-api/docs/)
- [Google Gemini Robotics-ER 1.5](https://ai.google.dev/gemini-api/docs/robotics-overview)