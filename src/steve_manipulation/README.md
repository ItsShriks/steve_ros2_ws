# steve_manipulation

Arm + gripper control for Steve — **UR5e + Robotiq 2F-85 on the Neobotix MMO-700** — with
MoveIt 2. Only the arm and gripper are commanded; the base is not planned (planning frame:
`base_link`). For coordinated base + arm motion see [`steve_wbc`](../steve_wbc).

## Contents

| Path | What |
|---|---|
| `launch/steve_manipulation.launch.py` | Gazebo or physical robot + `move_group` + RViz |
| `config/steve.srdf` | groups `ur_manipulator` (UR5e base → `gripper_tcp`) and `gripper`, named states, collision matrix |
| `config/kinematics.yaml`, `joint_limits.yaml`, `ompl_planning.yaml` | MoveIt planning |
| `config/moveit_controllers_{sim,real}.yaml` | MoveIt → `joint_trajectory_controller` (Gazebo) / `scaled_joint_trajectory_controller` (UR driver) + `gripper_controller` |
| `config/gripper_controllers.yaml`, `urdf/gripper_only.urdf.xacro` | real robot: stand-alone gripper controller manager |
| `steve_manipulation/__init__.py`, `steve_manipulation/manipulator.py` | Python package with the `SteveManipulator` API and `manipulator` CLI |
| `steve_manipulation/safety_stop.py` | safety stop shared with `steve_wbc`: EM-stop system (`/emergency_stop_state`) or LiDAR fallback |
| `steve_manipulation/bringup_probe.py` | detects an already running robot bringup (`ROS_AUTOSTART.sh`) so launch files never start a second UR driver |
| `../steve_wbc/` | Unified base + arm whole-body controller; see [`steve_wbc/README.md`](../steve_wbc/README.md) |
| `tools/generate_srdf.py` | regenerates the SRDF from the robot URDF |
| `tools/save_named_pose.py` | saves the arm's **live** joint positions (`/joint_states`) as a named state in `config/steve.srdf` |

The robot description is `steve_simulation/robots/mmo_700/mmo_700.urdf.xacro` with
`arm_tool:=robotiq_2f_85` (gripper xacro: `steve_simulation/components/gripper/robotiq_2f_85.urdf.xacro`,
mounted on `ur5etool0` via the UR-to-Robotiq adapter). The default `arm_tool:=none` keeps every
other launch file unchanged.

## Install

```bash
sudo apt install ros-humble-moveit ros-humble-robotiq-description ros-humble-gripper-controllers \
                 ros-humble-ur-description ros-humble-ur-robot-driver
colcon build --symlink-install --packages-select steve_simulation steve_hardware_bringup steve_manipulation
source install/setup.bash
```

## Launch — simulation vs. physical robot

```bash
# Gazebo (robot with gripper) + MoveIt + RViz
ros2 launch steve_manipulation steve_manipulation.launch.py use_sim_time:=true

# Physical robot: MoveIt on the running bringup (ROS_AUTOSTART.sh), or UR driver + TF if none runs
ros2 launch steve_manipulation steve_manipulation.launch.py use_sim_time:=false

# Physical robot without autostart: complete hardware bringup
ros2 launch steve_manipulation steve_manipulation.launch.py use_sim_time:=false real_bringup:=full

# Never start drivers
ros2 launch steve_manipulation steve_manipulation.launch.py use_sim_time:=false launch_bringup:=false
```

`launch_bringup:=auto` (default) looks for `/controller_manager` on the ROS graph (3 s): if the
robot's `ROS_AUTOSTART.sh` bringup runs, only MoveIt (+ the gripper controller) is started — two
UR drivers / controller managers on one arm conflict. It warns if the running robot
description has no gripper (run the bringup with `arm_tool:=robotiq_2f_85`). `robot_ip`
defaults to `steve_hardware_bringup/config/robot_config.yaml`.

On the UR teach pendant, start the *External Control* program after the driver is up.

### Gripper on the physical robot (`gripper_hw`)

| Value | Behaviour |
|---|---|
| `none` (default) | arm only — the gripper interface is not decided yet |
| `mock` | simulated gripper controller; MoveIt/API gripper commands succeed, nothing moves |
| `robotiq_driver` | Robotiq RS-485 through the UR tool port (`/tmp/ttyUR`, UR tool communication is enabled automatically). Needs `robotiq_driver` + `robotiq_controllers` from [ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper) |

The gripper action is `/gripper_controller/gripper_cmd` (`control_msgs/GripperCommand`) in
Gazebo and on the robot. Knuckle angle: `0.0` open … `0.7929` closed.

## Commanding the arm and gripper

```bash
ros2 run steve_manipulation manipulator list
ros2 run steve_manipulation manipulator named ready          # home, ready, table_reach, forward, parked
ros2 run steve_manipulation manipulator joints 0 -1.2 1.2 -1.6 -1.57 0
ros2 run steve_manipulation manipulator pose 0.7 0.0 1.0 --rpy 3.14159 0 0   # gripper_tcp in base_link
ros2 run steve_manipulation manipulator gripper close        # open | close | 0.0..0.7929
ros2 run steve_manipulation manipulator --vel 0.1 --plan_only named forward
```

```python
import rclpy
from steve_manipulation.manipulator import SteveManipulator

rclpy.init()
arm = SteveManipulator(velocity_scaling=0.2)
arm.move_to_named("ready")
arm.move_to_pose([0.7, 0.0, 1.0], rpy=[3.14159, 0.0, 0.0])
arm.open_gripper(); arm.close_gripper()
arm.destroy_node(); rclpy.shutdown()
```

Or use the MotionPlanning panel in RViz (planning group `ur_manipulator` / `gripper`).

**New named states from the real arm**: move the arm there (pendant freedrive or `manipulator`),
then save its live joint positions (UR driver running; on a remote PC with the robot's
`ROS_DOMAIN_ID` / DDS profile):

```bash
python3 tools/save_named_pose.py parked --dry_run   # read and show only
python3 tools/save_named_pose.py parked             # add / replace 'parked' in config/steve.srdf
```

`parked` (2026-10-14): shoulder_pan 0°, lift −90°, elbow 150°, wrist_1 −65°, wrist_2 0°, wrist_3 9°
— the arm folded. `table_reach` puts the flange only ~0.20 m above a 0.83 m table: with a tool or the
gripper camera mounted use `ready` (flange ~0.40 m above it) instead. With `--symlink-install` no rebuild is needed; the robot PC needs the changed
SRDF in its own workspace. `--direct` sends the state straight to the trajectory controller
without MoveIt's collision check.

**Safety stop**: no arm or gripper motion is started while the robot's EM-stop system reports a
stop (EM stop, S300 protective field red), and a running arm motion is **cancelled** as soon as
it does. Without an EM-stop system (Gazebo) the LiDAR fallback is used; on the real robot a
missing `/emergency_stop_state` blocks motion. `--safety auto|em_stop|lidar|off`.

## Collision matrix

`config/steve.srdf` was generated by `tools/generate_srdf.py` (rules in its docstring) rather
than the Setup Assistant: rigid/adjacent links, the static robot body and the wrist + camera +
gripper cluster are excluded; the arm and gripper are checked against base, cabinet, lidars and
pan-tilt tower. If MoveIt reports a false *start state in collision*, add the pair to the
SRDF or refine with `ros2 launch moveit_setup_assistant setup_assistant.launch.py`.

## Notes

* With `gripper_hw:=none` on the physical robot, MoveIt logs that the gripper joint state is
  missing; arm planning is unaffected.
* Velocity/acceleration scaling defaults to 0.2 (`config/joint_limits.yaml`, `--vel/--acc`).
