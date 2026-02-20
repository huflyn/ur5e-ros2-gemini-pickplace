# Using Google Gemini Robotics-ER 1.5 (Gemini API) with Universal Robots UR5e in ROS2 Humble

![ROS2](https://img.shields.io/badge/ROS_2-Humble-349eeb.svg) ![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-e95420.svg)

- [I) Prerequisites](#i-prerequisites)
- [II) Environment \& Installation](#ii-environment--installation)
  - [Shell \& Locale Setup](#shell--locale-setup)
    - [Shell Environment](#shell-environment)
    - [Locale ``en_US.UTF-8``](#locale-en_usutf-8)
  - [Install ROS 2 Drivers and Tools](#install-ros-2-drivers-and-tools)
    - [1. ros2\_control (Docs)](#1-ros2_control-docs)
    - [2. Universal Robots ROS2 Driver (Docs, GitHub)](#2-universal-robots-ros2-driver-docs-github)
    - [3. MoveIt 2 (Docs)](#3-moveit-2-docs)
    - [4. Realsense ROS Wrapper (Docs, GitHub)](#4-realsense-ros-wrapper-docs-github)
    - [5. Google Gen AI SDK (Docs)](#5-google-gen-ai-sdk-docs)
    - [6. Optional: Webots (Docs)](#6-optional-webots-docs)
  - [Setup Google Gemini API](#setup-google-gemini-api)
    - [1. Create Gemini API Key (Docs)](#1-create-gemini-api-key-docs)
    - [2. Google GenAI SDK Installation (Docs)](#2-google-genai-sdk-installation-docs)
    - [3. Setting the API key as an environment variable (Docs)](#3-setting-the-api-key-as-an-environment-variable-docs)
    - [5. Make your first request (Docs)](#5-make-your-first-request-docs)
- [III) Workspace Overview](#iii-workspace-overview)
  - [Folder Structure](#folder-structure)
  - [Key Packages](#key-packages)
  - [IV) Usage](#iv-usage)
    - [Step 1: Start the Robot Driver (Real or Simulated)](#step-1-start-the-robot-driver-real-or-simulated)
      - [Option A: Webots Simulation](#option-a-webots-simulation)
      - [Option B: Fake Hardware (no Webots, just RViz and MoveIt)](#option-b-fake-hardware-no-webots-just-rviz-and-moveit)
      - [Option C: Real Hardware (UR5e)](#option-c-real-hardware-ur5e)
    - [Step 2: Start MoveIt](#step-2-start-moveit)
    - [Step 3: Run Task (e.g. pick and place)](#step-3-run-task-eg-pick-and-place)
  - [1. Descriptions](#1-descriptions)
- [X) Documentation and References](#x-documentation-and-references)

---


## I) Prerequisites

**Software:**
- [(Ubuntu 22.04 LTS)](https://releases.ubuntu.com/jammy/)
- [(ROS 2 Humble)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
  
**Hardware:**
- [(Universal Robot UR5e)](https://www.universal-robots.com/)
- [(Robotiq EPick Vakuumpumpe)](https://robotiq.com/products/vacuum-grippers#EPick)
- [(Piab piSOFTGRIP)](https://www.piab.com/suction-cups-and-soft-grippers/soft-grippers/pisoftgrip-vacuum-driven-soft-gripper-/sg.x)
- [(Intel RealSense 415D camera)](https://www.intel.com/content/www/us/en/products/sku/128256/intel-realsense-depth-camera-d415/specifications.html)

---


## II) Environment & Installation

### Shell & Locale Setup

#### Shell Environment
**Bash** is the standard shell on most Ubuntu systems and is used throughout this guide.
* If you are using **Bash**, use the ``~/.bashrc`` file.
* If you are using **Zsh**, please replace instances of ``.bashrc`` with ``.zshrc`` in all following commands.
Check your current shell by running:
```bash
echo $0
```

#### Locale ``en_US.UTF-8``
Some packages and tools may require the ``en_US.UTF-8`` locale to function correctly (using a dot ``.`` as the decimal separator).
If your system is set to a different locale (e.g. using a comma ``,``), you might encounter issues during installation or runtime.

To force the locale to ``en_US.UTF-8``, follow these steps:

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

# 5. Verify settings
locale
```

Everything should now be set to ``en_US.UTF-8``

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


### Install ROS 2 Drivers and Tools


#### 1. ros2_control ([Docs](https://control.ros.org/humble/index.html#))

The ros2_control is a framework for real-time control of robots using ROS 2.

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```


#### 2. Universal Robots ROS2 Driver ([Docs](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/index.html), [GitHub](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble))

The ROS 2 driver packages for Universal Robots manipulators to control a Universal Robots arm from an external application using ROS 2 (ur_robot_driver).

```bash
sudo apt install ros-humble-ur
```

#### 3. MoveIt 2 ([Docs](https://moveit.ai/install-moveit2/binary/))

[MoveIt 2](https://moveit.picknik.ai/humble/index.html#) is the robotic manipulation platform for ROS 2, and incorporates the latest advances in motion planning, manipulation, 3D perception, kinematics, control, and navigation.

```bash
 sudo apt install ros-humble-moveit
```
It is recommendet to use CycloneDDS as the default ROS 2 middleware when working with MoveIt.

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
# You may want to add this to ~/.bashrc to source it automatically
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

#### 4. Realsense ROS Wrapper ([Docs](https://dev.realsenseai.com/docs/docs-get-started), [GitHub](https://github.com/realsenseai/realsense-ros))

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

#### 5. Google Gen AI SDK ([Docs](https://ai.google.dev/gemini-api/docs/quickstart))

Google Gen AI Python SDK provides an interface for developers to integrate Google's generative models into their Python applications

Using Python 3.9+, install the google-genai package using the following pip command:

```bash
pip install -q -U google-genai
```

#### 6. Optional: Webots ([Docs](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html#installation-ubuntu))

This is optional, but if you want to use the Webots simulation of the workcell, you need to install Webots and the ``webots_ros2`` package.

Webots is an open source and multi-platform desktop application used to simulate robots. It provides a complete development environment to model, program and simulate robots. The ``webots_ros2`` package provides an interface between ROS 2 and Webots. It includes several sub-packages, including ``webots_ros2_driver``, which allows you to start Webots and communicate with it.

1. **Install Webots**
Follow the instructions in the Webots documentation to install Webots on your system: [Webots Installation Guide](https://cyberbotics.com/doc/guide/installation-procedure#installation-on-linux)

2. **Install the ``webots_ros2`` package**

```bash
sudo apt-get install ros-humble-webots-ros2
```

---


### Setup Google Gemini API

#### 1. Create Gemini API Key ([Docs](https://ai.google.dev/gemini-api/docs/api-key))

To use the Gemini API, you need an API key. Create and manage your keys in [*Google AI Studio*](https://aistudio.google.com/app/apikey).

#### 2. Google GenAI SDK Installation ([Docs](https://ai.google.dev/gemini-api/docs/quickstart))

Using Python 3.9+, install the google-genai package using the following pip command:

```bash
pip install -q -U google-genai
```

#### 3. Setting the API key as an environment variable ([Docs](https://ai.google.dev/gemini-api/docs/api-key#set-api-env-var))

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

**Keep your API key secure ([Docs](https://ai.google.dev/gemini-api/docs/api-key#security))**

Your API key is like a password that allows access to your Google Cloud resources. Keep it secure and do not share it publicly or commit it to version control systems. If you believe your API key has been compromised, revoke it immediately in the Google AI Studio and generate a new one.

#### 5. Make your first request ([Docs](https://ai.google.dev/gemini-api/docs/quickstart#make-first-request))

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

## III) Workspace Overview

### Folder Structure

The Workspace folder structure is as follows:

```
lego_sorter_ws/
├── src/
│   ├── lego_sorter_interfaces/        # Custom msgs, srvs, actions
│   ├── description/       # URDF/Xacro, meshes, Webots PROTO files
│   ├── manipulation/      # Pick & place state machine
│   ├── moveit_config/     # MoveIt 2 configuration
│   ├── perception/        # Camera pipeline + Gemini API node
│   ├── webots/            # Webots world file, sim launch files
│   └── lego_sorter_bringup/           # Top-level launch files, configs, rviz
```

### Key Packages


---

### IV) Usage

To startup the complete system, you’ll have to start x launch files in individual terminals.

#### Step 1: Start the Robot Driver (Real or Simulated)

##### Option A: Webots Simulation

```bash
ros2 launch workcell_webots simulation.launch.py
```
This will start the Webots simulation of the workcell, which includes a simulated UR5e robot. The robot in the simulation will be controlled using the same ROS 2 interfaces as a real robot, allowing you to test your MoveIt 2 configuration without needing access to physical hardware.

##### Option B: Fake Hardware (no Webots, just RViz and MoveIt)

```bash
# Make sure to set use_fake_hardware:=true in the command below if you want to use fake hardware
ros2 launch workcell_control start_robot.launch.py use_fake_hardware:=true
```
This will start a simulated robot using the ros2_control fake hardware interface. The robot will mirror the commands sent to it, allowing you to test your MoveIt 2 configuration without needing access to physical hardware or the Webots simulation.

##### Option C: Real Hardware (UR5e)

If you want to start a real robot, make sure that the **[robot setup](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/robot_setup.html#robot-setup)** is done and **external_control** is active on the robot.
Make sure to set the correct **robot_ip** in the command below or in the ``start_robot.launch.py`` file in the ``workcell_control`` package!

```bash
# You can set the robot_ip either via command line argument or directly in the start_robot.launch.py file
ros2 launch workcell_control start_robot.launch.py robot_ip:=<ROBOT_IP_ADDRESS>
```
This will start the ROS 2 driver for the UR5e robot, allowing you to control the physical robot using ROS 2 interfaces. Make sure to follow all safety precautions when working with real robots.

#### Step 2: Start MoveIt

Now that we have a robot running (either simulated or real), we can launch the MoveIt **[move_group node](https://moveit.picknik.ai/humble/doc/concepts/move_group.html)**.

- With Webots Simulation, you need to set the argument ``use_sim_time:=true`` to make sure that MoveIt uses the simulation time provided by Webots.

```bash
# Set argument use_sim_time:=true if you are using the Webots simulation, default is false
ros2 launch my_robot_cell_moveit_config move_group.launch.py use_sim_time:=true
```

- With Fake Hardware or Real Hardware, you can simply launch the move_group node without the ``use_sim_time`` argument.
  
```bash
ros2 launch my_robot_cell_moveit_config move_group.launch.py
```

If everything went well you should see the output: “You can start planning now!”.

To interact with the MoveIt setup, you can start RViz with the correct setup file:

```bash
ros2 launch my_robot_cell_moveit_config moveit_rviz.launch.py
```

#### Step 3: Run Task (e.g. pick and place)


---
---
---

### 1. Descriptions


All description files (URDF/XACRO) for the robot, grippers, sensors, and the workcell will be stored in the ``descriptions`` folder. This includes the URDF/XACRO files for the UR5e robot, the Robotiq EPick gripper, the Piab piSOFTGRIP, and the Intel RealSense camera.
To visualize the robot in RViz use the ``view_workcell.launch.py`` file from the workcell_description package.

```bash
ros2 launch workcell_description view_workcell.launch.py
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