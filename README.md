# Steve pick-and-place recorder (`Tru_RnD`)

`pick_place_record.py` replays the xArm "Jessie" pick-and-place joint sequence on Steve's UR5 in MuJoCo
(a mobile base with a UR5 arm and a Robotiq 2F-85 gripper). It writes the result as CSV files in the same
format as a recorded `/xarm/joint_states` bag. You can use it to compare simulated and real recordings, or
to produce training and test data without the real robot.

Runs on **macOS** (Apple Silicon) and **Ubuntu** (22.04 and 24.04). This branch only contains the script. It
needs the packages from the main **steve_ros2_ws** workspace (`steve_wbc`, `steve_grasping`,
`steve_manipulation` and the Steve MJCF models) to run. ROS 2 is not required.

---

## 1. What the script does

- **Motion:** it runs the xArm script's sequence: `home → gripper_opening → pick → gripper_closing → lift →
  place → gripper_opening → retract → return_home`. Each move is a synchronised joint-space trapezoid, like
  the xArm SDK's `set_servo_angle` (20 deg/s, 500 deg/s² by default), followed by the same dwell time.
- **Joint mapping:** `--joint_map xarm` (the default) converts the xArm angles to the UR5's zero pose, so the
  arm pose looks the same. `--joint_map direct` applies them 1:1.
- **Object:** a can stands on a stand at the pick pose. A second stand is placed under the place pose. At the
  end, the script prints whether the can was held firmly and whether it was placed.
- **Control:** the arm is driven like `MujocoSimBackend`: a joint-velocity command plus gravity and damping
  feed-forward. The base is held still.
- **Recording:** the CSVs are made to look like a real bag:
  - 2 s of pre-roll before the motion starts
  - sensor noise on positions, velocities and efforts, plus a constant effort offset per joint
  - jitter on the header stamps and transport latency on the bag times
  - a few dropped messages

  Use `--no_noise` for the ideal simulation data.

---

## 2. Using it with the real repo

### Prerequisites

| | macOS (Apple Silicon) | Ubuntu 22.04 / 24.04 |
|---|---|---|
| **Python environment** | [pixi](https://pixi.sh), using the workspace's environment | `venv` and pip (the workspace's `pixi.toml` only supports `osx-arm64`) |
| **Packages** | installed by `pixi install` | `sudo apt install git python3-venv` |
| **Viewer only** | `mjpython`, included in the pixi environment | a display, and `sudo apt install libgl1 libegl1 libglfw3 libgl1-mesa-dri` |

Both need Python 3.10 or newer and MuJoCo 3.16 or newer. The script fails with older MuJoCo versions (a
`TypeError` from `mj_fullM` on 3.3.7).

### Steps

1. **Clone the main workspace and its `test` packages** over HTTPS (skip this if you already have them):
   ```bash
   git clone -b test https://github.com/ItsShriks/steve_ros2_ws.git
   cd steve_ros2_ws
   ./scripts/repos.sh sync test
   ```
   You don't need a GitHub account or SSH key, because these repositories are public. Use the `test` profile:
   the `dev` package list doesn't include `steve_wbc`, `steve_grasping` and `steve_manipulation`.

2. **Get the script from this branch** and put it into the `steve_wbc` package:
   ```bash
   git fetch origin Tru_RnD
   git show origin/Tru_RnD:pick_place_record.py > src/steve_wbc/steve_wbc/pick_place_record.py
   ```
   It must sit next to `steve_figure8_wbc.py`. It imports `ARM_JOINTS`, `BASE_JOINTS`, `load_model` and
   `resolve_model` from there, and `SimGripper`, `TOOLS`, `DEFAULT_TOOL` and `load_tool` from
   `steve_grasping`.

3. **Set up Python** from the workspace root:

   macOS:
   ```bash
   pixi install
   ```

   Ubuntu:
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   pip install --upgrade pip
   pip install "mujoco>=3.16" numpy scipy pyyaml
   ```

4. **(Optional, Ubuntu with ROS 2 Humble) Build the ROS 2 packages** if you want to import it as
   `steve_wbc.pick_place_record`:
   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --packages-select steve_wbc steve_grasping
   source install/setup.bash
   ```
   Without a build, the script falls back to imports from the source tree. Use that on macOS, where
   `steve_grasping` isn't in the overlay.

5. **Run it** from the workspace root. All options work the same on both systems.

   macOS:
   ```bash
   # headless, write the CSVs (default tool, can and stands, realistic noise)
   pixi run python src/steve_wbc/steve_wbc/pick_place_record.py

   # another tool on the flange
   pixi run python src/steve_wbc/steve_wbc/pick_place_record.py --tool ability_hand

   # repeatable run, ideal sim data, own output folder
   pixi run python src/steve_wbc/steve_wbc/pick_place_record.py --seed 42 --no_noise --out sim_data/run_42/csv

   # watch it in real time
   pixi run mjpython src/steve_wbc/steve_wbc/pick_place_record.py --viewer
   ```

   Ubuntu (with the venv active, see step 3):
   ```bash
   # headless, write the CSVs (default tool, can and stands, realistic noise)
   python src/steve_wbc/steve_wbc/pick_place_record.py

   # another tool on the flange
   python src/steve_wbc/steve_wbc/pick_place_record.py --tool ability_hand

   # repeatable run, ideal sim data, own output folder
   python src/steve_wbc/steve_wbc/pick_place_record.py --seed 42 --no_noise --out sim_data/run_42/csv

   # watch it in real time
   python src/steve_wbc/steve_wbc/pick_place_record.py --viewer
   ```

6. **Check the console output.** Each phase prints its end time and maximum joint error. At the end you get,
   for example, `Can slip … held` and `Can … placed`.

7. **Use the CSVs.** By default they are written to `./sim_data/pick_place_<timestamp>/csv/`:

   | File | Content |
   |---|---|
   | `full_motion_joint_states.csv` | `bag_time_ns, bag_time_sec, header_stamp_sec`, `joint1..6_position [rad] / _velocity [rad/s] / _effort [Nm]`, `left_finger_* / right_finger_*` |
   | `<phase>_joint_states.csv` | the rows of one phase (same columns) |
   | `phase_timestamps.csv` | `phase, start_time_ns, end_time_ns` |

   These are the same columns as the xArm bag → CSV export, so you can use the same analysis scripts on
   real and simulated runs.

### Useful options

| Option | Default | Meaning |
|---|---|---|
| `--model` | `steve_wbc/models/mmo_700_wbc.xml` | MJCF of Steve |
| `--tool` | `DEFAULT_TOOL` | tool on the flange (see `steve_grasping.tools.TOOLS`) |
| `--joint_map` | `xarm` | `xarm` (pose-equivalent) or `direct` (1:1) |
| `--no_object` | off | arm only, no can or stands |
| `--can_diameter` | 56 mm | must be larger than `GRIPPER_CLOSE` (52 mm) so the gripper squeezes the can |
| `--no_noise` / `--noise_scale` / `--seed` | off / 1.0 / random | noise of the recording |
| `--speed` / `--acc` | 20 deg/s / 500 deg/s² | move profile (xArm SDK defaults) |
| `--grip_time` | 1.0 s | gripper open / close time |
| `--rate` | 100 Hz | joint_states rate |
| `--pre_roll` | 2.0 s | recording before the motion |
| `--out` | `./sim_data/pick_place_<ts>/csv` | output folder |
| `--viewer` | off | MuJoCo viewer (macOS: run with `pixi run mjpython`; Ubuntu: run with `python`) |

To change the waypoints or gripper openings, edit `HOME_POSITION … RETRACT_POSITION`, `GRIPPER_OPEN` /
`GRIPPER_CLOSE` and `SEQUENCE` at the top of the script. To change the noise model, edit `NOISE` and
`NOISE_FINGER`.

---

## Acknowledgement

This work is part of the R&D project of **Trushar Ghanekar**
([Trushar2411](https://github.com/Trushar2411)).
