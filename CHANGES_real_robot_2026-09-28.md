# Real-robot bring-up — changes of 2026-09-28

Goal: get Steve (MMO-700 + UR5e) running on the real robot and run the vertical figure-8
(Phase 2) with the whole-body controller (WBC).
Result: the figure-8 runs on the real robot, quietly, with 0.12 cm RMS tracking (small figure)
in the new arm velocity mode; RViz on a PC works over the lab Wi-Fi.

Workflow: all code was edited on the PC (`dev`), copied to the robot (`~/steve_ros2_ws`,
branch `test`) and built there. Nothing was committed on the robot — push to `test` manually.

---

## 1. `steve_essentials` (submodule) — `neo_kinematics_omnidrive2`

### `src/neo_omnidrive_socketcan.cpp` — drives recover after EM / scanner stop
* `emergency_stop_callback` treats `emergency_state != EMFREE` **or** `scanner_stop` as a stop and
  reactivates the motors once both have been clear for `m_reactivate_delay` (1 s).
  Before: after a scanner stop the base (and the joystick) stayed dead until the bringup was restarted.
* New members `m_last_stop_time`, `m_reactivate_delay`.

### `src/neo_omnidrive_node.cpp` — base odometry
`/joint_states` is shared by the wheels (100 Hz), the UR arm (500 Hz) and the pan-tilt.
* Messages without wheel joints are ignored (they caused "invalid joint state delta time" and
  skipped odometry steps).
* Subscription queue 1 → 50.
* Main loop `spin_some` → `SingleThreadedExecutor::spin_all(5 ms)`: `spin_some` handled one
  message per cycle, so `/odom` ran at ~13 Hz with gaps > 0.25 s (WBC aborted "odom stale").
  Now 100 Hz.

---

## 2. `steve_hardware_bringup`

| File | Change |
|---|---|
| `config/robot_config.yaml` | `ur_robot_ip: 192.168.1.11` (was .102); wrist camera → `wrist_camera_model: d415`, `wrist_camera_serial: "840412060445"` |
| `launch/ur5e_arm.launch.py` | exposes `update_rate` (required by ur_robot_driver 2.14 after the apt upgrade); spawns `forward_velocity_controller` (inactive) |
| `launch/hardware_bringup.launch.py` | new args `enable_joystick` (teleop publishes `/cmd_vel` continuously and overrode the WBC) and `ur_autostart`; `enable_pan_tilt` / `enable_camera` default **false**; starts `ur_autostart` and `arm_velocity_watchdog` |
| `launch/wrist_camera.launch.py` | model / serial / profile from `robot_config.yaml` (D415: 640x480x15, runs on USB 2); args prefixed `wrist_` (plain `device_type` leaked in from the pan-tilt launch → it looked for an L515); `wrist_initial_reset` default false (the reset hung on USB 2) |
| all launch files | `steve_config` import with built-in fallback defaults (a missing config never breaks the bringup) |
| `scripts/ur_autostart` (new) | presses PLAY on External Control when the arm is powered, brakes released, safety NORMAL, remote control; re-activates `scaled_joint_trajectory_controller` after each program start. Never powers on, releases brakes or moves the arm |
| `scripts/arm_velocity_watchdog` (new) | sole publisher to `forward_velocity_controller`: forwards `/arm_velocity_cmd` (clamped to 1 rad/s), sends **zero velocity** after 0.1 s without input; respawned |
| `scripts/steve_doctor` | wrist camera check uses the configured model |
| `config/fastdds_steve.xml` (new) | robot DDS profile: UDP datagrams ≤ 1400 B (the lab Wi-Fi drops IP fragments), shared memory inside the robot |
| `config/fastdds_remote_pc.xml` (new) | PC DDS profile: unicast discovery to 10.7.4.213 (multicast is blocked), same 1400 B limit |
| `CMakeLists.txt`, `package.xml` | install the new scripts; deps `rclpy`, `std_msgs`, `controller_manager_msgs` |
| `README.md` | new launch args, `ur_autostart`, "Remote PC (RViz over Wi-Fi)" section |
| `config/ur_controller_overrides.yaml` | **unused — do not commit** (`open_loop_control: true` crashed `ur_ros2_control_node`) |

---

## 3. `steve_wbc`

| File | Change |
|---|---|
| `steve_wbc/steve_figure8_wbc.py` | `--arm_cmd_tau`: optional low-pass on the streamed arm velocity (trajectory mode; the position setpoint stays unfiltered, so no lag in the tracking loop). Velocity mode: switches `scaled_joint_trajectory_controller` → `forward_velocity_controller` on the first command and back on exit (`--switch_controllers`, `--arm_vel_controller`, `--arm_traj_controller`, `--controller_manager`) |
| `launch/steve_figure8_wbc.launch.py` | `arm_interface` default: **velocity on the real robot** (via `/arm_velocity_cmd` + watchdog), trajectory in Gazebo |
| `package.xml` | dep `controller_manager_msgs` |
| `README.md` | why velocity mode: streaming a new trajectory every 20 ms made the joints buzz |

`steve_wbc` and `steve_manipulation` did not exist on the robot before; both were copied over whole.

### 2026-09-29: strict task hierarchy (default)
* `steve_wbc/steve_figure8_wbc.py`: `--hierarchy strict|weighted` (default **strict**), `--hqp_tol`.
  Strict = one constrained least-squares per level (EE pose > base pose > posture); each lower level
  keeps the higher levels' achieved task values (`|A_i u − A_i u_i| ≤ hqp_tol`), so limits / collision
  constraints can never let a lower task pull the EE off. Used by all WBC scripts (shared controller).
* Kinematic check, table-orbit demo (Phase 1): weighted 0.55 / 1.90 cm, 1.30° → strict 0.01 / 0.03 cm, 0.00°.
  Vertical figure-8: unchanged (nothing competes). QP time about the same (~10–13 ms mean on the PC).
* `README.md`: formulation and comparison table.

## 4. `steve_manipulation`
* `launch/steve_manipulation.launch.py`: no longer forces `enable_pan_tilt:=true`; config-loader fallback.

---

## 5. Outside the repository

**Robot**
* `~/.bashrc` (backups `~/.bashrc.bak-2026-09-28`, `~/.bashrc.bak-2026-09-28-dds`): aliases
  `start-ssh-agent`, `start-ssh-hrl`; the second `ROS_AUTOSTART.sh` block commented out;
  `export FASTRTPS_DEFAULT_PROFILES_FILE=…/fastdds_steve.xml`.
* `~/ROS_AUTOSTART.sh` (backup `~/ROS_AUTOSTART.sh.bak-2026-09-28-dds`): same export.
* `~/ros_logs/` created; MuJoCo installed (`python3 -m pip install --user mujoco "numpy==1.21.5"`).
* `neo_relayboard_v2-2`: the **original** `NeoRelayBoardNode.cpp` (EM = button OR scanner) is built.

**PC**
* `~/.ssh/id_steve` — key login to the robot.
* `~/.ros/fastdds_steve.xml` (= `fastdds_remote_pc.xml`), `~/.ros/steve_robot.urdf` (RViz fallback).
* Use with: `export ROS_DOMAIN_ID=74 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds_steve.xml && ros2 daemon stop && rviz2`

---

## 6. How to run the figure-8 on the real robot

```bash
# robot, terminal 1 (inside screen so an SSH drop does not kill it)
ros2 launch steve_hardware_bringup hardware_bringup.launch.py enable_joystick:=false
# ur_autostart presses PLAY and activates the controller once the arm is on + brakes released

# robot, terminal 2: arm to the forward pose (20 s)
ros2 action send_goal /scaled_joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [ur5eshoulder_pan_joint, ur5eshoulder_lift_joint, ur5eelbow_joint, ur5ewrist_1_joint, ur5ewrist_2_joint, ur5ewrist_3_joint], points: [{positions: [1.5708, -1.558, 2.075, -0.516, 1.571, 0.0], time_from_start: {sec: 20}}]}}"

# figure-8 (velocity mode is the default on the robot); 1 m free left/right, hand on the e-stop
ros2 launch steve_wbc steve_figure8_wbc.launch.py use_sim_time:=false amplitude:=0.22 frequency:=0.45 loops:=3 log:=$HOME/ros_logs/fig8.csv
```

Run only **one** bringup: `ROS_AUTOSTART.sh` already starts one at boot (`screen -r bringup`);
a second one fights over the UR connection and the pendant program stops.

## 7. Results

| Run | Arm mode | RMS / max error | Noise |
|---|---|---|---|
| small (A 0.10 m, ω 0.25 rad/s, 1 loop) | trajectory | 0.17 / 0.71 cm | joints buzz |
| small | **velocity** | **0.12 / 0.65 cm** | **quiet** |
| full (A 0.22 m, ω 0.45 rad/s, 2 loops) | trajectory | 1.00 / 2.23 cm | buzz |
| full, 3 loops | velocity | completed (`fig8_vel_full.csv`) | quiet |

## 8. Problems found and fixed along the way
* UR driver could not connect: wrong IP (found 192.168.1.11 by ping sweep).
* `update_rate` launch error after the UR driver upgrade; `ros2 control` CLI / controller_manager version mismatch (apt upgrade).
* Base ignored the WBC: orphaned `neo_teleop2` processes and teleop publishing `/cmd_vel` constantly.
* Odometry stale / wrong (see §1).
* Trajectory controller inactive after every bringup restart (driver's controller_stopper) → `ur_autostart`.
* Arm buzzing → velocity mode + watchdog.
* Wrist camera looked for an L515, then hung at "Resetting device…" → `wrist_` args, reset off.
* No topics / no robot model on the PC over Wi-Fi → DDS unicast + 1400 B datagrams.

## 9. Open issues
1. PC `dev` still has the **modified** relayboard (EM = button only, decf630) and docs describing it
   (`safety_stop.py`, READMEs); the robot runs the original — revert on the PC.
2. The pendant program showed STOPPED again at 17:15 — check that only one bringup runs.
3. An EM "button" stop tripped by itself once; afterwards the drives went silent on CAN until the
   bringup was restarted — suspect a loose contact or the wireless e-stop.
4. MoveIt `named home`: "Cannot find planning configuration for group 'ur_manipulator'".
5. D415 not mounted yet: URDF / MuJoCo still have the D405 mount and camera pose; it runs on USB 2.1
   (15 fps); depth only beyond ~0.3 m.
6. UR calibration mismatch warning → extract with `ur_calibration`.
7. `steve_simulation` changes from `dev` not merged into `test` (patch conflicts).
8. Next: real-camera VGN pipeline (depth + intrinsics + TF → TSDF → grasps on the calibrated target).
