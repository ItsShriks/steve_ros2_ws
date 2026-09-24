# steve_wbc

Unified velocity-level (kinematic) **whole-body controller** for Steve
(Neobotix MMO-700 omnidirectional base + UR5e). Base and arm velocities come from
**one** constrained QP over all 9 DOF — `u = [vx_b, vy_b, wz_b, dq1..dq6]`:

```
min_u  || J_wb u - v_ee ||²_Wt                 EE pose task (primary)
     + w_b || u_base - u_base_des ||²          soft base-pose task (optional)
     + || u - N·K(q_nom - q) ||²_Wu             null-space posture + effort
s.t.   velocity / acceleration / joint-position limits
       collision avoidance (arm/base vs table + floor, arm vs own body)
```

`J_wb` is the exact whole-body Jacobian from the MuJoCo model (`models/mmo_700_wbc.xml`),
including the lever arm of base yaw. The same controller runs in MuJoCo, Gazebo and on
the real robot.

| Executable | Purpose |
|---|---|
| `steve_figure8_wbc` | Figure-8 with the EE — MuJoCo (`--mode sim`), kinematic check (`--mode kin`), Gazebo / real robot over ROS 2 (`--mode ros`) |
| `figure8_wbc_demo` | MuJoCo-only demo: figure-8 on the table while the base orbits it, then a vertical figure-8 |
| `active_perception_wbc` | Drive (LiDAR) → perceive the object from t = 0 → (VGN scan) → **MPC interception** of its predicted path → grasp → lift; every phase solved by the same 9-DOF joint-velocity WBC |

## Active perception + MPC interception (`active_perception_wbc`)

* **Perception from the first tick**: head (pan-tilt) camera while driving, wrist + head camera
  afterwards; detections (object centre = surface point + radius along the viewing ray) feed a
  gated least-squares constant-velocity tracker. Detections closer than 20 cm to the camera are
  ignored (gripper occlusion). If the object is already tracked when the arm settles, the wrist
  camera looks straight at it and the shoulder sweep is skipped.
* **`InterceptionMPC`**: receding-horizon (1.5 s, 0.1 s steps) double-integrator plan per axis,
  minimising distance to the *predicted* object path + velocity mismatch + acceleration,
  |a| ≤ a_max, solved exactly as bounded least squares. The WBC tracks its first step, so the
  gripper arrives above the object moving with it, descends and grasps while the base follows.
  If the predicted path leaves the table before interception the run aborts with that reason.
* **VGN** (MuJoCo): 4 TSDF views around the object, inference in the `vgn_env` conda env
  (override with `VGN_PYTHON`, `VGN_DIR`); falls back to the tracked position if unavailable.
* **ROS 2**: LiDAR from `scan_topic`, object from `target_topic` (`geometry_msgs/PointStamped`,
  odom or base_link), gripper via `steve_manipulation` (`/gripper_controller/gripper_cmd`).

```bash
ros2 launch steve_wbc active_perception_wbc.launch.py                       # MuJoCo
ros2 launch steve_wbc active_perception_wbc.launch.py object_type:=box vgn:=false
ros2 launch steve_wbc active_perception_wbc.launch.py simulator:=gazebo       # needs a perception node
ros2 launch steve_wbc active_perception_wbc.launch.py use_sim_time:=false dry_run:=true gripper_hw:=mock
```

Without ROS (e.g. macOS): all scripts are also in `steve_ros2_ws/scripts/`
(`mjpython active_perception_wbc.py`, `mjpython figure8_wbc.py`, `mjpython steve_figure8_wbc.py --mode sim`).

## Build

```bash
pip install mujoco          # not available via rosdep; numpy/scipy come from rosdep
colcon build --symlink-install --packages-select steve_wbc
source install/setup.bash
```

## Launch — simulation vs. physical robot

Same convention as `steve_navigation`: `use_sim_time:=true` → simulation, `false` → robot.

```bash
# 1. MuJoCo (no ROS robot needed)
ros2 launch steve_wbc steve_figure8_wbc.launch.py use_sim_time:=true simulator:=mujoco

# 2. Gazebo (starts steve_simulation, WBC talks to /cmd_vel, /odom, joint_trajectory_controller)
ros2 launch steve_wbc steve_figure8_wbc.launch.py use_sim_time:=true

# 3. Physical robot — dry run: reads /joint_states + /odom, publishes nothing
ros2 launch steve_wbc steve_figure8_wbc.launch.py use_sim_time:=false dry_run:=true

# 4. Physical robot — small and slow first
ros2 launch steve_wbc steve_figure8_wbc.launch.py use_sim_time:=false \
    amplitude:=0.08 frequency:=0.25 loops:=1 log:=/tmp/wbc_real.csv
```

`launch_bringup:=false` skips starting Gazebo / the hardware bringup if it is already running.
Other controller options go through `extra_args`, e.g.
`extra_args:="--max_base_v 0.1 --w_base_lin 2.0"` (see `ros2 run steve_wbc steve_figure8_wbc --help`).

MuJoCo demo (simulation only):

```bash
ros2 launch steve_wbc figure8_wbc_demo.launch.py                 # both phases
ros2 launch steve_wbc figure8_wbc_demo.launch.py phase:=1 loops:=2
```

On macOS the MuJoCo viewer needs `mjpython`; use `headless:=true` or run the module with
`mjpython -m steve_wbc.figure8_wbc`.

## Interfaces (`--mode ros`)

| | Topic | Type |
|---|---|---|
| in | `/joint_states` | `sensor_msgs/JointState` (the `ur5e` joint prefix is detected automatically) |
| in | `/odom` | `nav_msgs/Odometry` |
| out | `/cmd_vel` | `geometry_msgs/Twist` (base frame) |
| out | `/scaled_joint_trajectory_controller/joint_trajectory` (robot) or `/joint_trajectory_controller/joint_trajectory` (Gazebo) | `trajectory_msgs/JointTrajectory` |
| out | `/forward_velocity_controller/commands` with `arm_interface:=velocity` | `std_msgs/Float64MultiArray` |

For `arm_interface:=velocity`, switch the controller first:

```bash
ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller --activate forward_velocity_controller
```

## Safety

The controller aborts, sending zero `/cmd_vel` and holding the arm, if feedback is older than
0.25 s, if the EE tracking error exceeds 10 cm, if the EE drops below 0.3 m, or on Ctrl+C. The
MMO-700 driver also stops the base when `/cmd_vel` is silent for 0.1 s.

## Collision avoidance

Hard constraints inside the same QP (velocity dampers): for every pair closer than
`d_influence` (10 cm) the approach speed is limited, reaching zero at `d_safe` (2 cm), so
the controller gives up EE tracking before it gives up clearance.

| Pair | Always | Only with `--collision_env model` |
|---|---|---|
| arm + gripper vs floor | ✓ | |
| arm + gripper vs own body (base, cabinet, pan-tilt tower) | ✓ | |
| arm + gripper vs static environment of the MuJoCo model (table) | | ✓ |
| base footprint vs static environment | | ✓ |

The arm is covered by spheres, obstacles by oriented boxes, and distances are computed in
closed form (MuJoCo's `mj_geomDistance` proved unreliable for these mesh/primitive pairs).
`--collision_env` defaults to `model` in `sim`/`kin` mode and to `none` in `ros` mode: the
MuJoCo table is at a fixed position that does not exist in the real robot's odom frame.
Tune with `--d_safe`, `--d_influence`, `--xi`; disable with `--no_collision`
(launch: `extra_args:="--d_safe 0.04"`).
