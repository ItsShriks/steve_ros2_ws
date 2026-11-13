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

### 1. Clone the Workspace and Packages
Clone the workspace, then fetch all package repositories into `src/`:

```bash
git clone git@gitlab.igg.uni-bonn.de:hrl_software/steve-moma/steve_ros2_ws.git -b test   # or -b dev
cd steve_ros2_ws
./scripts/repos.sh
```

Every repository is also on GitHub. Cloning the workspace from there works the same way, and the packages are then fetched from GitHub too:

```bash
git clone git@github.com:ItsShriks/steve_ros2_ws.git -b test
```

All repositories live in the GitLab group [`hrl_software/steve-moma`](https://gitlab.igg.uni-bonn.de/hrl_software/steve-moma), with a copy of each on GitHub (see [Package Descriptions](#package-descriptions)). The packages are **not** submodules: each folder in `src/` is a normal git clone on the branch listed in the profile's repos file ([`steve_test.repos`](steve_test.repos) or [`steve_dev.repos`](steve_dev.repos)). See [Working with Package Repositories](#working-with-package-repositories).

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

## Working with Package Repositories

Each profile has its own list: [`steve_test.repos`](steve_test.repos) (real robot) and [`steve_dev.repos`](steve_dev.repos). It names every package repository (on GitLab) and the **branch** it tracks. The main repo stores only these lists, not commit pointers. You can commit and push inside any package without touching the main repo.

The profile follows the workspace branch: on `test` the script uses `steve_test.repos`, on `dev` `steve_dev.repos`. `./scripts/repos.sh sync dev` (shortcut `git go-dev`, likewise `git go-test`) switches the workspace and every package to that profile; packages only the other profile lists are removed from `src/`, `build/` and `install/`.

| Command | What it does |
|---|---|
| `./scripts/repos.sh` | Clone missing packages, switch each to its branch, fast-forward to the latest commit. Also sets every clone's `origin` to fetch from the fetch host and push to both |
| `./scripts/repos.sh status` | Show branch, ahead/behind and local changes for each package |
| `./scripts/repos.sh push` | Push every package that has unpushed commits (to GitLab and GitHub) |
| `./scripts/repos.sh mirror` | Copy all branches and tags of every repository (also `steve_essentials`' submodules) from the fetch host to the other one |

Typical workflow:

```bash
./scripts/repos.sh                      # get the latest of everything
cd src/steve_navigation
git add -A && git commit -m "..."       # edit and commit as usual
git push                                # or ./scripts/repos.sh push from the root
```

- **Switch a package to another branch:** change its `version:` in the profile's repos file, run `./scripts/repos.sh`, and commit that file. This is the only change that needs a main-repo commit.
- **Add a package:** add an entry to the profile's repos file and run `./scripts/repos.sh`.
- **GitLab and GitHub:** each repos entry has a GitLab `url` and a `github` URL. Every clone (workspace and packages) fetches from one host and pushes to both, so a plain `git push` updates GitLab and GitHub together. The fetch host is the one the workspace was cloned from. Change it with `REPOS_HOST=gitlab ./scripts/repos.sh` (or `github`). The `steve_essentials` submodules use relative URLs (`../neo_msgs2.git`), so they come from the same host as `steve_essentials`.
- If one host fell behind (a push made outside these clones), `./scripts/repos.sh mirror` brings it up to date. It never force-pushes: a diverged branch is reported, not overwritten.
- `sync` never overwrites your work. It refuses to switch branches when there are uncommitted changes and stops if a branch has diverged from origin.
- The repos files also work with [vcstool](https://github.com/dirk-thomas/vcstool): `vcs import src < steve_test.repos` and `vcs pull src`.

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

**To launch simulation only (with map and robot description):**
```bash
ros2 launch steve_simulation simulation.launch.py
```

**To create a map using SLAM:**
```bash
ros2 launch steve_navigation slam.launch.py use_sim_time:=true
```

---

## Package Descriptions

| Package | Repositories |
|---|---|
| `steve_essentials` | [GitLab](https://gitlab.igg.uni-bonn.de/hrl_software/steve-moma/steve_essentials) · [GitHub](https://github.com/ItsShriks/steve_essentials) |
| `steve_navigation` | [GitLab](https://gitlab.igg.uni-bonn.de/hrl_software/steve-moma/steve_navigation) · [GitHub](https://github.com/ItsShriks/steve_navigation) |
| `steve_simulation` | [GitLab](https://gitlab.igg.uni-bonn.de/hrl_software/steve-moma/steve_simulation) · [GitHub](https://github.com/ItsShriks/steve_simulation) |
| `steve_perception` | [GitLab](https://gitlab.igg.uni-bonn.de/hrl_software/steve-moma/steve_perception) · [GitHub](https://github.com/pratik-adhikari/steve_perception) |
| `steve_command_grounding` | [GitLab](https://gitlab.igg.uni-bonn.de/hrl_software/steve-moma/steve_command_grounding) · [GitHub](https://github.com/RiddheshMore/steve_command_grounding) |
| `steve_hardware_bringup` | [GitLab](https://gitlab.igg.uni-bonn.de/hrl_software/steve-moma/steve_hardware_bringup) · [GitHub](https://github.com/ItsShriks/steve_hardware_bringup) |
| `steve_manipulation` | [GitLab](https://gitlab.igg.uni-bonn.de/hrl_software/steve-moma/steve_manipulation) · [GitHub](https://github.com/ItsShriks/steve_manipulation) |
| `steve_wbc` | [GitLab](https://gitlab.igg.uni-bonn.de/hrl_software/steve-moma/steve_wbc) · [GitHub](https://github.com/ItsShriks/steve_wbc) |
| `steve_grasping` | [GitLab](https://gitlab.igg.uni-bonn.de/hrl_software/steve-moma/steve_grasping) · [GitHub](https://github.com/ItsShriks/steve_grasping) |
| `steve_pan_tilt_controller` | [GitLab](https://gitlab.igg.uni-bonn.de/hrl_software/steve-moma/steve_pan_tilt_controller) · [GitHub](https://github.com/RiddheshMore/steve_pan_tilt_controller) |

`steve_essentials` pulls in the Neobotix drivers, `joystick_drivers` and `realsense-ros` as submodules, which are mirrored in the same GitLab group.

### `steve_essentials`
**Foundational Drivers & Utilities**  
The core package containing all hardware drivers (LiDAR, Base, Camera), custom message definitions, and kinematics solvers. It consolidates dependencies like `neo_kinematics_omnidrive2`, `realsense-ros`, and `joy`.  
[GitLab](https://gitlab.igg.uni-bonn.de/hrl_software/steve-moma/steve_essentials) · [GitHub](https://github.com/ItsShriks/steve_essentials)

### `steve_navigation`
**Navigation Stack**  
Provides launch files and configurations for autonomous navigation, SLAM, and localization. Supports both simulation and real hardware.  
[GitLab](https://gitlab.igg.uni-bonn.de/hrl_software/steve-moma/steve_navigation) · [GitHub](https://github.com/ItsShriks/steve_navigation)

### `steve_simulation`
**Gazebo Simulation**  
Contains robot description files (URDF/Xacro), world files, and simulation-specific launch configurations.  
[GitLab](https://gitlab.igg.uni-bonn.de/hrl_software/steve-moma/steve_simulation) · [GitHub](https://github.com/ItsShriks/steve_simulation)

### `steve_perception`
**Vision Pipeline**  
Perception stack using the L515 RealSense camera for object detection, scene understanding, and visual processing.  
[GitLab](https://gitlab.igg.uni-bonn.de/hrl_software/steve-moma/steve_perception) · [GitHub](https://github.com/pratik-adhikari/steve_perception)

### `steve_command_grounding`
**HRI & Semantics**  
Natural language command understanding and grounding to robot actions, enabling semantic navigation and task execution.  
[GitLab](https://gitlab.igg.uni-bonn.de/hrl_software/steve-moma/steve_command_grounding) · [GitHub](https://github.com/RiddheshMore/steve_command_grounding)

### `steve_hardware_bringup`
**Robot Startup**  
Scripts and launch files for initializing the physical robot hardware. Managed automatically via `ROS_AUTOSTART.sh`.  
[GitLab](https://gitlab.igg.uni-bonn.de/hrl_software/steve-moma/steve_hardware_bringup) · [GitHub](https://github.com/ItsShriks/steve_hardware_bringup)

### `steve_manipulation`
**Arm Control**  
MoveIt 2 motion planning and control for the UR5e arm and Robotiq 2F-85 gripper (base not planned).  
[GitLab](https://gitlab.igg.uni-bonn.de/hrl_software/steve-moma/steve_manipulation) · [GitHub](https://github.com/ItsShriks/steve_manipulation)

### `steve_wbc`
**Whole-Body Control**  
Velocity-level whole-body controller over base + arm (9 DOF) as a strict task hierarchy, with the grasping pipeline and web GUI.  
[GitLab](https://gitlab.igg.uni-bonn.de/hrl_software/steve-moma/steve_wbc) · [GitHub](https://github.com/ItsShriks/steve_wbc)

### `steve_grasping`
**End Effectors & Grasps**  
Tool models (D415 on the flange, Robotiq 2F-85, PSYONIC Ability Hand) and how `steve_wbc` grasps with each.  
[GitLab](https://gitlab.igg.uni-bonn.de/hrl_software/steve-moma/steve_grasping) · [GitHub](https://github.com/ItsShriks/steve_grasping)

### `steve_pan_tilt_controller`
**Pan-Tilt Unit**  
Control of the Dynamixel XH430-W250 pan-tilt unit, real hardware and Gazebo, with a JointState publisher for TF.  
[GitLab](https://gitlab.igg.uni-bonn.de/hrl_software/steve-moma/steve_pan_tilt_controller) · [GitHub](https://github.com/RiddheshMore/steve_pan_tilt_controller)

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

Use the provided scripts to build and run the Docker container:

### Build
```bash
./scripts/build_docker.sh
```

### Run
```bash
./scripts/run_docker.sh
```



This workspace includes a `.devcontainer` configuration for VSCode, allowing you to develop in a consistent, containerized environment.
1. Open the workspace in VSCode.
2. When prompted, select **"Reopen in Container"**.

---