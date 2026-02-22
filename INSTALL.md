# Install ROS 2 Drivers and Tools <!-- omit from toc -->

[![jazzy][jazzy-badge]][jazzy]
[![ubuntu24][ubuntu24-badge]][ubuntu24]

[jazzy-badge]: https://img.shields.io/badge/-JAZZY-orange?style=flat-square&logo=ros
[jazzy]: https://docs.ros.org/en/jazzy/index.html
[ubuntu24-badge]: https://img.shields.io/badge/-UBUNTU%2024%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu24]: https://releases.ubuntu.com/noble/

- [ros2\_control (Docs)](#ros2_control-docs)
- [Universal Robots ROS2 Driver (Docs, GitHub)](#universal-robots-ros2-driver-docs-github)
- [MoveIt 2 (Docs)](#moveit-2-docs)
- [ROS Wrapper for RealSense™ Cameras (GitHub)](#ros-wrapper-for-realsense-cameras-github)
- [Google Gen AI SDK (Docs)](#google-gen-ai-sdk-docs)
- [Optional: Webots (Docs)](#optional-webots-docs)



# ros2_control ([Docs](https://control.ros.org/jazzy/index.html))

The ros2_control is a framework for real-time control of robots using ROS 2.

```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
```


# Universal Robots ROS2 Driver ([Docs](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/index.html), [GitHub](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/jazzy))

The ROS 2 driver packages for Universal Robots manipulators to control a Universal Robots arm from an external application using ROS 2 (ur_robot_driver).

```bash
sudo apt install ros-jazzy-ur
```


# MoveIt 2 ([Docs](https://moveit.ai/install-moveit2/binary/))

[MoveIt 2](https://moveit.picknik.ai/jazzy/index.html#) is the robotic manipulation platform for ROS 2, and incorporates the latest advances in motion planning, manipulation, 3D perception, kinematics, control, and navigation.

```bash
sudo apt install ros-jazzy-moveit
```
It is recommendet to use CycloneDDS as the default ROS 2 middleware when working with MoveIt.

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
# You may want to add this to ~/.bashrc to source it automatically
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```


# ROS Wrapper for RealSense™ Cameras ([GitHub](https://github.com/realsenseai/realsense-ros))

<details>
    <summary>
        Step 1: Install latest RealSense™ SDK 2.0
    </summary>

Please choose only one option from the 3 options below (in order to prevent multiple versions installation and workspace conflicts)

- ### Option 1: Install librealsense2 debian package from RealSense servers
    - [Linux Debian Installation Guide](https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)
    - In this case treat yourself as a developer: make sure to follow the instructions to also install librealsense2-dev and librealsense2-dkms packages

- ### Option 2: Install librealsense2 (without graphical tools and examples) debian package from ROS servers (Foxy EOL distro is not supported by this option):

    - Configure your Ubuntu repositories ([Docs](https://wiki.ros.org/Installation/Ubuntu/Sources))

      - Setup your sources.list to accept software from packages.ros.org.
      ```bash
      sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
      ```
      - Setup your keys

      ```bash
      sudo apt install curl # if you haven't already installed curl
      curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
      ```

    - Install all realsense ROS packages

    ```bash
    sudo apt install ros-jazzy-librealsense2*
    ```

- ### Option 3: Install from source
  - See [instructions](https://github.com/realsenseai/realsense-ros?tab=readme-ov-file#installation-on-ubuntu) in the GitHub repository.

</details>


<details>
    <summary>
        Step 2: Install ROS Wrapper for RealSense™ Cameras
    </summary>

- ### Option 1: Install debian package from ROS servers (Foxy EOL distro is not supported by this option):

  - [Configure](https://wiki.ros.org/Installation/Ubuntu/Sources) your Ubuntu repositories
  - You can skip this step if you followed the instructions to install [librealsense2 package (Step 1, Option 2)](#option-2-install-librealsense2-without-graphical-tools-and-examples-debian-package-from-ros-servers-foxy-eol-distro-is-not-supported-by-this-option) 

  - Install all realsense ROS packages
    ```bash
    sudo apt install ros-jazzy-librealsense2*
    ```

- ### Option 2: Install from source
  - See [instructions](https://github.com/realsenseai/realsense-ros?tab=readme-ov-file#installation-on-ubuntu) in the GitHub repository.

</details>


# Google Gen AI SDK ([Docs](https://ai.google.dev/gemini-api/docs/quickstart))

Google Gen AI Python SDK provides an interface for developers to integrate Google's generative models into their Python applications

Using Python 3.9+, install the google-genai package using the following pip command:

```bash
pip install -q -U google-genai
```


# Optional: Webots ([Docs](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html#installation-ubuntu))

This is optional, but if you want to use the Webots simulation of the workcell, you need to install Webots and the ``webots_ros2`` package.

- ### Install Webots
    Follow the instructions in the Webots documentation to install Webots on your system: [Webots Installation Guide](https://cyberbotics.com/doc/guide/installation-procedure#installation-on-linux)

- ### Install the ``webots_ros2`` package

    ```bash
    sudo apt-get install ros-jazzy-webots-ros2
    ```
---

Back to [README.md](README.md#install-ros-2-drivers-and-tools)