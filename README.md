# steve_ros2_ws

**ROS2 Humble workspace for the Steve Butler robot project** — an omnidirectional mobile manipulator designed for research in human-robot interaction, navigation, and manipulation.

---

## Overview

This workspace provides a complete software stack for the **Steve Butler** robot, supporting both high-fidelity Gazebo simulation and real hardware deployment. The robot features:
- **Neobotix MMO-700** omnidirectional mobile base
- **UR5e robotic arm** with Robotiq gripper
- **L515 RealSense camera** on a custom pan-tilt unit
- **Dual SICK LiDAR sensors** for 360° obstacle detection
- **Autonomous Navigation & SLAM** using the Nav2 stack

---

## Installation

### 1. Clone the Repository
Clone the workspace ensuring all submodules (including `steve_essentials` dependencies) are initialized:

```bash
git clone --recurse-submodules git@gitlab.igg.uni-bonn.de:hrl_students/ws2526_butler/steve_ros2_ws.git -b dev
cd steve_ros2_ws
```

### 2. Install Dependencies
Install required ROS 2 packages:

```bash
sudo apt install \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-joy \
  ros-humble-teleop-twist-joy \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-realsense2-camera
```

### 3. Build the Workspace
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Quick Start (Simulation)

Launch the full simulation environment with localization and navigation enabled:

```bash
ros2 launch steve_navigation localization.launch.py use_sim_time:=true
```

This command will:
1. Start the Gazebo simulation (default: `small_house` world).
2. Launch RViz for visualization.
3. Start the Nav2 stack with AMCL localization.

**To create a map using SLAM:**
```bash
ros2 launch steve_navigation slam.launch.py use_sim_time:=true
```

---

## Package Descriptions

### `steve_essentials`
**Foundational Drivers & Utilities**  
The core package containing all hardware drivers (LiDAR, Base, Camera), custom message definitions, and kinematics solvers. It consolidates dependencies like `neo_kinematics_omnidrive2`, `realsense-ros`, and `joy`.  
[Read more](src/steve_essentials/README.md)

### `steve_navigation`
**Navigation Stack**  
Provides launch files and configurations for autonomous navigation, SLAM, and localization. Supports both simulation and real hardware.  
[Read more](src/steve_navigation/README.md)

### `steve_simulation`
**Gazebo Simulation**  
Contains robot description files (URDF/Xacro), world files, and simulation-specific launch configurations.  
[Read more](src/steve_simulation/README.md)

### `steve_perception`
**Vision Pipeline**  
Perception stack using the L515 RealSense camera for object detection, scene understanding, and visual processing.  
[Read more](src/steve_perception/README.md)

### `steve_command_grounding`
**HRI & Semantics**  
Natural language command understanding and grounding to robot actions, enabling semantic navigation and task execution.  
[Read more](src/steve_command_grounding/README.md)

### `steve_hardware_bringup`
**Robot Startup**  
Scripts and launch files for initializing the physical robot hardware. Managed automatically via `ROS_AUTOSTART.sh`.  
[Read more](src/steve_hardware_bringup/README.md)

### `steve_manipulation` (In Progress)
**Arm Control**  
Motion planning and control for the UR5e arm and Robotiq gripper.

---

## Contributors & Acknowledgments

This project is developed at the **Humanoid Robots Lab (HRL)**, University of Bonn.

### Lead Developers
- **Shrikar Nakhye** - [GitHub](https://github.com/Itsshriks) | [Email](mailto:nakhyeshrikar@icloud.com)
- **Pratik Adhikari** - [GitHub](https://github.com/pratik-adhikari) | [Email](mailto:pratikadhikari.de@gmail.com)
- **Riddhesh More** - [GitHub](https://github.com/RiddheshMore) | [Email](mailto:riddheshmore311@gmail.com)

### Acknowledgements
- **Rohit Menon** - For mentorship and technical guidance on Neobotix platforms.
- **Prof. Maren Bennewitz** - Head of the Humanoid Robots Lab, University of Bonn.

---

## Robot Communication (Physical Robot)

When connecting to the physical robot (Ubuntu 20/22):

- **Static IP**: `10.7.4.213`
- **Ethernet Port**: `192.168.60.90` (DHCP)

Configure your network settings to match the robot's subnet for proper communication.

---

## Docker Support

This workspace includes a `.devcontainer` configuration for VSCode, allowing you to develop in a consistent, containerized environment.
1. Open the workspace in VSCode.
2. When prompted, select **"Reopen in Container"**.
