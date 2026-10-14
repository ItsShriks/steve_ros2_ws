# Grasp perception on the real robot — changes of 2026-10-14

Goal: grasp perception for the red-ball scenario (VGN; DiNBV-Grasp evaluated) and, on the real robot,
the MuJoCo "static ball" run — drive to the table, find the ball, go to the grasp pose — with a
D415 on the flange instead of a gripper.
Result: hand-held grasp test with live RViz; VGN + exploration of other table sides in
`active_perception_wbc` (MuJoCo-tested); gripper camera calibrated on the robot (ball RMS 0.17 cm);
table re-mapped; robot URDF fixed (arm mount). First real go-to-grasp-pose run was jerky (arm
commanded with streamed trajectories) → velocity interface + camera tilt, to be re-tested.

Setup: **no gripper**; D415 (serial 840412060445) ~5 cm below the flange on a short post, looking
along the flange axis, USB to the PC. The PC (Wi-Fi, ROS_DOMAIN_ID 74) runs camera, detector, WBC and
VGN; the robot runs its bringup (base, UR driver, LiDARs; no pan-tilt camera).

---

## 1. `steve_perception` (submodule, `test` @ `ee34141`)

| File | Change |
|---|---|
| `steve_perception/grasp_test/handheld_grasp_test.py` (new) | hand-held RealSense as the wrist camera: snapshot → table plane + object anchor the robot frame (image-top = robot forward, object at `--object_xy`) → VGN (TSDF with the table 5 cm above the volume floor, as VGN was trained) or Contact-GraspNet → 9-DOF WBC rollout (pre-grasp, grasp, lift) → `grasp.json`, `trajectory.csv/png`; `--from_npz`, `--synthetic` |
| `.../grasp_test/rviz_live.py` (new) | `--live`: live camera pose + cloud in `odom`, Enter = snapshot, grasp markers, Steve model replaying the trajectory; URDF shoulder offset read from `/robot_description` |
| `.../grasp_test/cgn_client.py` (new) | Contact-GraspNet client speaking DiNBV-Grasp's ZMQ protocol (server not part of either repo) |
| `launch/grasp_test_rviz.launch.py`, `rviz/grasp_test.rviz` (new) | robot_state_publisher (Steve URDF) + RViz layout |
| `setup.py`, `README.md`, `grasp_test/README.md` | entry point `handheld_grasp_test`, rviz data files, docs |

DiNBV-Grasp: a view planner on top of Contact-GraspNet; its Stage I/II checkpoints are not
released and the stack needs ≥ 24 GB VRAM — only its grasp generator (CGN) is usable here.

## 2. `steve_wbc`

| File | Change |
|---|---|
| `steve_wbc/active_perception_wbc.py` | **VGN on the robot** (`WristDepthGrabber`: depth subscribed only while views are taken, pose from TF, each view levelled on the table plane; `VGNInference` shared by sim + robot). **`--vgn_explore fuse\|fresh\|off`** (default fuse, static objects): no VGN grasp → side marked tried, drive around the table, scan, rerun VGN on all views (drift-compensated); no side left → `no grasp candidate found`. **`--goto_grasp_only`**: `AT_GRASP` (stay with the object for `--hold_time`) → `RETREAT` → `reached grasp pose`, gripper never used. **`--camera_tilt DEG`**: drive + scan poses look forward-down (wrist 1). **Approach side: stays where it is if the grasp is reachable from there** (before: the object's quadrant could send it around the table although reachable). **Smooth drive around the table** (`TableOrbit` / `OrbitPath`): rounded ring at the any-heading clearance, smooth blend on and off it, facing the table, min-jerk speed, tracked by the WBC base task (before: corner-to-corner waypoints with stops, heading turned at the start). `--vgn_camera`, `--vgn_no_level`, `--vgn_max_level`, `--vgn_explore_max_sides`. **Startup crash fixed**: `--table` / `--grasp_dz` were defined twice (the shared parser has them; `grasp_dz` keeps −0.015) |
| `launch/active_perception_wbc.launch.py` | VGN no longer forced off on the robot; **arm velocity interface on the real robot** (via `/arm_velocity_cmd` + watchdog, `--switch_controllers`) like the figure-8 launch; args `detector_cameras` (e.g. `gripper_camera`; no pan-tilt → head not commanded), `goto_grasp_only`, `arm_interface` |
| `launch/gripper_camera.launch.py` (new) | RealSense as `gripper_camera` (848x480x30, both 4.54 and ≥ 4.55 profile parameter names) + static mount TF `ur5etool0 → gripper_camera_color_optical_frame` from `config/gripper_camera.yaml` |
| `steve_wbc/gripper_camera_calibrator.py` (new) | eye-in-hand calibration with the ball on the table (no marker): per pose ball centre (sphere fit), table plane, flange pose (arm still); solves mount + ball + table height, multi-start, reports residuals and uncertainty; TF on its own thread |
| `steve_wbc/vgn_grasp_real.py` | table optional (measured in the view → table model `seen_table`), camera auto (gripper first), TSDF placed as in VGN's training |
| `test/vgn_explore_runner.py` (new) | MuJoCo with VGN's verdict forced to "no grasp" (FAIL=1 / all) |
| `config/gripper_camera.yaml` (new) | calibrated mount: xyz (0.0366, −0.0049, 0.0479) m, rpy (0.6, 0.2, 179.3)° |
| `config/targets/my_ball.yaml` (+ captures) | red ball, 6.5 cm, recorded with the gripper camera |
| `config/tables/lab_table.yaml` (+ png) | re-measured (`--lidar_only --top_z 0.827`), in today's `odom` |
| `setup.py`, `README.md` | entry point `gripper_camera_calibrator`; "Real robot: gripper camera on the PC" section, go-to-grasp-pose, camera tilt, exploration |

## 3. `steve_manipulation`

| File | Change |
|---|---|
| `config/steve.srdf`, `tools/generate_srdf.py` | named state **`parked`** (folded: pan 0°, lift −90°, elbow 150°, wrist_1 −65°, wrist_2 0°, wrist_3 9°), read from the live arm |
| `tools/save_named_pose.py` (new) | saves the live `/joint_states` as a named SRDF state |
| `README.md` | saving named states; `table_reach` too low with a tool mounted |

## 4. `steve_simulation` (submodule pointer)

`04180fb` → **`98c96eb`** (`origin/test`, "Horizontal arc8"): UR5e mounted at **+90°** in
`ur_arm.urdf.xacro` (was −90°), + Robotiq xacro, UR controller config. The robot was running the
old URDF: its TF had the arm turned 180° at the shoulder (checked with the parked pose: wrist on
the robot's left = +90°, as the WBC `--pan_offset` and the SRDF assume).

## 5. `steve_hardware_bringup` (submodule)

| File | Change |
|---|---|
| `config/fastdds_remote_pc.xml` | discovery peers also on **127.0.0.1** (the PC's own nodes never found each other: multicast off, peers = robot only); 16 MB SHM segment for images |
| `README.md` | remote-PC section: localhost peers, Ethernet address |

## 6. Outside the repository

* PC `~/.ros/fastdds_steve.xml`: same two changes (backup `fastdds_steve.xml.bak_20261014_170812`).
* Robot: `src/steve_simulation` checked out at `98c96eb`, rebuilt, bringup restarted.
* Robot: `steve_manipulation` install had lost its Python metadata (`PackageNotFoundError`) after
  building in a different mode → clean rebuild of that package.

---

## 7. How to run (gripper camera on the PC, robot bringup running)

```bash
# every PC terminal
source /opt/ros/humble/setup.bash && source ~/steve_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=74 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds_steve.xml

ros2 launch steve_wbc gripper_camera.launch.py                               # camera + mount TF
ros2 run steve_wbc table_probe --name lab_table --lidar_only --top_z 0.827   # after each bringup restart
ros2 launch steve_wbc active_perception_wbc.launch.py use_sim_time:=false launch_bringup:=false \
    dry_run:=true table:=lab_table target:=my_ball detector_cameras:=gripper_camera vgn:=false \
    goto_grasp_only:=true extra_args:="--camera_tilt 25"                       # dry run, at the table
# real: arm `named home`, robot ~1 m back, drop dry_run, add "--max_base_v 0.10 --max_arm_vel 0.3 --grasp_dz 0.05"
```

Camera mount calibration (once per mounting): `gripper_camera_calibrator --target my_ball`, 8 poses
via `manipulator --direct joints ...` (steve_wbc README).

## 8. Results

* MuJoCo, VGN exploration (`test/vgn_explore_runner.py`): no grasp from the front (forced) → left
  side, 8 fused views, VGN score 1.00, grasped and **held**; no grasp anywhere → front, left, back,
  right → **`no grasp candidate found`**, nothing grasped.
* MuJoCo, side choice + orbit: far-half ball reachable from the front → stays, `reached grasp pose`
  (before: drove around to the left side); forced "no grasp" from the front → smooth orbit to the
  left side in 11 s (sideways up to 0.26 m/s while turning −1° → −90° through the corner, no
  stops), base ≥ 18.3 cm from the table, `reached grasp pose`. Scene tests: 3 passed, 1 skipped —
  `test_single_moving_ball` passes again (it failed on `test` this morning).
* MuJoCo, `--goto_grasp_only`: static ball, VGN off → `reached grasp pose`, EE 0.3 cm (mean) / 0.6 cm
  (max) from the grasp point for 3 s; rolling ball (1 cm/s), VGN on → reached, 0.9 cm.
* Gripper camera calibration: ball RMS **0.17 cm** (8 poses, 0.36–0.56 m), table tilt ≤ 0.3°,
  height ±0.1 cm, mount ±4 mm / ±0.6°; table top 0.827 m.
* Hand-held test (synthetic): camera anchor 0.3 cm, VGN score 0.99, 9-DOF rollout 0.3 cm max error;
  RViz robot within 0.1 cm of the computed grasp.
* Wi-Fi to the robot: joint states 544 Hz, odom 102 Hz, gaps < 40 ms; PC/robot clocks within 31 ms.

## 9. Problems found and fixed

| Problem | Fix |
|---|---|
| Robot URDF mounted the UR5e at −90° (TF 180° off at the shoulder) | `steve_simulation` → `98c96eb` on PC and robot |
| `active_perception_wbc` crashed at startup on `test` (duplicate `--table`, `--grasp_dz`) | duplicates removed |
| PC nodes (camera, RViz) did not see each other | localhost discovery peers in the PC DDS profile |
| D415 topics are `/gripper_camera/color/...` with the workspace's realsense2_camera 4.54; profile parameters `rgb_camera.profile` / `depth_module.profile` (apt ≥ 4.55: `..._profile`) | launch passes both; docs use the actual topics |
| Ball depth 17 % valid at 0.40 m (D415 at 1280x720 sees > 0.45 m) | 848x480 default (> 0.31 m); calibrator rejects views with < 50 % valid depth |
| Calibrator: no TF while waiting at its prompt | TF listener on its own node + thread; waits for the robot TF at start |
| `-p head_topic:=` (empty) breaks rcl argument parsing | `-p "head_topic:=''"` |
| Real run jerky: the active-perception launch streamed one-point trajectories (spline restarts every 20 ms) | velocity interface on the robot, as the figure-8 launch |
| Camera along the flange axis looked straight down in drive / scan poses: ball not visible | `--camera_tilt` (25° covers ~0.65–1.3 m ahead) |
| `table_reach` puts the flange 0.20 m above the table | not used with a tool; calibration poses keep ≥ 1.14 m |
| Robot drove around the table although the ball was reachable from where it stood (quadrant rule first) | current side first if reachable |
| Driving around the table was rectangular: stop at every ring corner, turn to the final heading at the start | smooth `TableOrbit` path tracked by the WBC base task |

## 10. Open issues

* Real go-to-grasp-pose run with the velocity interface + `--camera_tilt 25` and the smooth orbit:
  not yet re-run.
* VGN and exploration on the robot (`vgn:=true`): untested on hardware.
* MuJoCo, static ball, VGN on, `--goto_grasp_only`: once the open fingers pushed the ball off the
  table in the final descent (VGN off: fine) — to investigate.
* During the descent the camera gets closer than the D415's minimum range: the static ball's
  estimate is held (`--lost_timeout_static`), a moving one is predicted.
* No gripper: keep `--grasp_dz` ≥ 0.05 (at the true grasp pose the camera ends ~1 cm above the ball).
* `tools/generate_srdf.py`: `NAMED_ARM` (except `parked`) still has the old shoulder_pan convention
  (0 instead of −π/2) — regenerating would overwrite the SRDF's checked states.
* Planned: YOLO ball detector; Contact-GraspNet server for the DiNBV path; Ethernet for the control loop.
