#!/usr/bin/env python3
"""
pick_place_record.py — the xArm "Jessie" pick-and-place joint sequence replayed on Steve's UR5 in MuJoCo,
recorded like the xArm's /xarm/joint_states bag -> CSV.

    pixi run python src/steve_wbc/steve_wbc/pick_place_record.py                     # headless, CSVs
    pixi run python src/steve_wbc/steve_wbc/pick_place_record.py --tool ability_hand
    mjpython src/steve_wbc/steve_wbc/pick_place_record.py --viewer                   # watch it (macOS)

The waypoints are the xArm's joint angles [deg]. --joint_map xarm (default) converts them to the UR5's
zero pose (xarm_to_ur: the arm pose looks like the xArm's — home tool-down, pick / place a forward side
grasp); --joint_map direct applies them 1:1 (joint1..6 = shoulder_pan .. wrist_3). A can stands on a
stand at the gripper's pinch point in the pick pose, a second stand waits under the pinch point of the
place pose (--no_object: the arm alone); the end of the run says whether the can arrived there.
Moves are synchronised joint-space trapezoids like the xArm SDK's set_servo_angle (default 20 deg/s,
500 deg/s^2), followed by the same dwell; the arm is driven like MujocoSimBackend (joint-velocity
command + gravity / damping feed-forward), the base is held still.

Like a recorded bag (--no_noise: the ideal sim): the recording starts PRE_ROLL before the motion (the xArm
script sleeps 2 s after starting `ros2 bag record`), the joint states carry sensor noise (NOISE, and a
constant effort offset per joint), header stamps jitter around the 100 Hz sample times, bag times come a
transport latency later, and a few messages are dropped. --noise_scale scales all of it, --seed repeats a run.

Output (--out, default ./sim_data/pick_place_<timestamp>/csv):
  full_motion_joint_states.csv   bag_time_ns, bag_time_sec, header_stamp_sec,
                                 joint<i>_position [rad] / _velocity [rad/s] / _effort [Nm],
                                 left_finger_* / right_finger_* (FINGER_JOINTS: the tool's finger joints;
                                 Robotiq: the driver knuckle, 0 open .. 0.8 rad closed; position quantised
                                 to the real gripper's 0..255 steps)
  <phase>_joint_states.csv       the rows of one phase (same columns)
  phase_timestamps.csv           phase, start_time_ns, end_time_ns
"""

import argparse
import csv
import time
from datetime import datetime
from pathlib import Path

import mujoco
import numpy as np

try:
    from steve_wbc.steve_figure8_wbc import ARM_JOINTS, BASE_JOINTS, load_model, resolve_model
    from steve_grasping.gripper import SimGripper
    from steve_grasping.tools import DEFAULT_TOOL, TOOLS, load_tool
except ImportError:  # plain script from the source tree (e.g. macOS: steve_grasping not in the Mac overlay)
    import sys
    SCRIPT_DIR = Path(__file__).resolve().parent
    sys.path[:0] = [str(SCRIPT_DIR), str(SCRIPT_DIR.parents[1] / "steve_grasping")]
    from steve_figure8_wbc import ARM_JOINTS, BASE_JOINTS, load_model, resolve_model
    from steve_grasping.gripper import SimGripper
    from steve_grasping.tools import DEFAULT_TOOL, TOOLS, load_tool

# Joint positions [deg] (xArm script)
HOME_POSITION = [0, -53.6, 4.3, 0, 49.4, 0]
PICK_POSITION = [0, 39, -60, 0, -67, 0]
LIFT_POSITION = [-22, 25, -72, 0, -45, 0]
PLACE_POSITION = [-15, 46, -68, 0, -69, 0]
RETRACT_POSITION = [-21, 16, -67, 0, -42, 0]

# Gripper positions (xArm gripper units: opening in 0.1 mm, 850 = 85 mm open) -> the same opening on the sim
# tool (gripper_fraction)
GRIPPER_OPEN = 700
GRIPPER_CLOSE = 520
# Robotiq 2F-85 in the sim: pad gap [mm] at a closure fraction (ctrl / 255), measured free (no object)
ROBOTIQ_GAP = ([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 1.0],
               [85.1, 77.6, 69.6, 61.3, 52.6, 43.7, 34.5, 25.2, 15.7, 0.0])

# (phase, kind, target, dwell [s]) — the xArm script's pick_and_place()
SEQUENCE = [
    ("home", "move", HOME_POSITION, 1.0),
    ("gripper_opening", "grip", GRIPPER_OPEN, 1.0),
    ("pick", "move", PICK_POSITION, 3.0),
    ("gripper_closing", "grip", GRIPPER_CLOSE, 1.0),
    ("lift", "move", LIFT_POSITION, 1.0),
    ("place", "move", PLACE_POSITION, 1.0),
    ("gripper_opening", "grip", GRIPPER_OPEN, 1.0),
    ("retract", "move", RETRACT_POSITION, 1.0),
    ("return_home", "move", HOME_POSITION, 1.0),
]
PHASES = ["home", "gripper_opening", "pick", "gripper_closing", "lift", "place", "retract", "return_home"]

# The object: a can (cylinder) standing on a stand at the pick pinch point; the place stand top is
# PLACE_DROP below the can's bottom in the place pose, so the opened gripper sets it down
CAN_HALF_H, CAN_MASS = 0.05, 0.10
STAND_HALF_XY = 0.04
PLACE_DROP = 0.005

# Real-world look of the recording (1 sigma unless noted; x --noise_scale)
NOISE = {
    "position": 1.5e-4,         # [rad] encoder / filter noise
    "velocity": 3e-3,           # [rad/s] velocity estimate from the encoders
    "effort": 0.15,             # [Nm] current-based torque estimate
    "effort_offset": 0.3,       # [Nm] constant per joint (friction / model error), drawn once per run
    "stamp_jitter": 0.4e-3,     # [s] header stamp around the nominal sample time
    "latency": 0.6e-3,          # [s] header -> bag: fixed part ...
    "latency_tail": 0.8e-3,     # [s] ... + exponential tail (mean)
    "drop": 0.003,              # probability that a message is missing from the bag
}
NOISE_FINGER = {"velocity": 0.01, "effort": 0.05}   # [rad/s], [Nm]
PRE_ROLL = 2.0     # [s] recorded before the motion starts

# Left / right finger joints of the tool in the CSV (position, velocity, effort), and the step of the real
# gripper's position reading [rad] (Robotiq: 0..255 over the driver's 0..0.8 rad)
FINGER_JOINTS = {
    "robotiq_2f85": {"left_finger": "left_driver_joint", "right_finger": "right_driver_joint", "step": 0.8 / 255},
}

KP = 10.0          # [1/s] position correction on top of the trajectory velocity
SETTLE_TOL = 1e-3  # [rad] a move ends once the arm is this close to its target (xArm wait=True)


def xarm_to_ur(q_deg):
    """xArm6 joint angles [deg] -> UR5 joint angles [deg] for the same arm pose. Both pitch joints
    (xArm 2 / 3 / 5, UR shoulder_lift / elbow / wrist_1) turn forward-down for positive angles; the xArm's
    zero is upper arm up, forearm forward, tool down (UR: [0, -90, 90, -90, -90, 0]); the xArm's forearm
    roll (joint 4) becomes the UR's wrist_2. Its link offsets (the L-shaped upper arm) are ignored."""
    j1, j2, j3, j4, j5, j6 = q_deg
    return np.radians([j1, j2 - 90.0, j3 + 90.0, j5 - 90.0, j4 - 90.0, j6])


def trapezoid(dq, v_max, a_max):
    """Synchronised joint move by dq: s(t) in [0, 1] and ds/dt, scaled to the joint moving most."""
    d = float(np.max(np.abs(dq)))
    if d < 1e-9:
        return 0.0, lambda t: (1.0, 0.0)
    v, a = v_max / d, a_max / d                      # in units of s
    if v * v / a > 1.0:                              # triangular: never reaches v
        v = np.sqrt(a)
    t_acc = v / a
    T = 1.0 / v + t_acc

    def profile(t):
        if t <= 0.0:
            return 0.0, 0.0
        if t < t_acc:
            return 0.5 * a * t * t, a * t
        if t < T - t_acc:
            return 0.5 * a * t_acc ** 2 + v * (t - t_acc), v
        if t < T:
            r = T - t
            return 1.0 - 0.5 * a * r * r, a * r
        return 1.0, 0.0
    return T, profile


class PickPlaceSim:
    def __init__(self, a):
        self.a = a
        self.tool_name = a.tool
        self.to_ur = xarm_to_ur if a.joint_map == "xarm" else np.radians
        tool = load_tool(a.tool)
        path = resolve_model(a.model)
        self.can_goal = None
        if a.no_object:
            self.m = load_model(path, tool=tool)
        else:
            pick, place = self.pinch_points(load_model(path, tool=tool), [PICK_POSITION, PLACE_POSITION])
            self.can_goal = place - [0.0, 0.0, PLACE_DROP]
            self.m = load_model(path, tool=tool,
                                spec_edit=lambda spec: add_objects(spec, pick, place, a.can_diameter / 2000.0))
        self.d = mujoco.MjData(self.m)
        m, d = self.m, self.d
        jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        aid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
        joints = [jid(n) for n in BASE_JOINTS + ARM_JOINTS]
        self.qpos_ids = np.array([m.jnt_qposadr[j] for j in joints])
        self.dof_ids = np.array([m.jnt_dofadr[j] for j in joints])
        names = ["base_x", "base_y", "base_yaw", "shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2",
                 "wrist_3"]
        self.vel_act = [aid(f"{n}_vel") for n in names]
        self.motor_act = [aid(f"{n}_motor") for n in names]
        self.grip = SimGripper(m, tool)
        self.grip_frac = 0.0
        self.M = np.zeros((m.nv, m.nv))
        self.can_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "can")
        self.pinch_site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "pinch")
        self.can_off0, self.can_slip = None, 0.0
        fingers = FINGER_JOINTS.get(a.tool, {})
        self.finger_step = fingers.get("step", 0.0)
        self.fingers = []  # (column prefix, qpos address, dof address)
        for name in ("left_finger", "right_finger"):
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, fingers[name]) if name in fingers else -1
            if j >= 0:
                self.fingers.append((name, m.jnt_qposadr[j], m.jnt_dofadr[j]))
        if not self.fingers:
            print(f"tool {a.tool}: no left / right finger joints configured (FINGER_JOINTS), no finger columns")

        floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        if floor >= 0:  # as MujocoSimBackend: the planar base would only fight floor friction
            m.geom_contype[floor] = m.geom_conaffinity[floor] = 0
        if m.nkey > 0:
            mujoco.mj_resetDataKeyframe(m, d, 0)
        d.qpos[self.qpos_ids[:3]] = 0.0
        d.qpos[self.qpos_ids[3:]] = self.to_ur(HOME_POSITION)
        d.qvel[:] = 0.0
        self.grip.command(d, fraction=self.grip_frac)
        mujoco.mj_forward(m, d)

        self.n_rec = max(1, int(round(1.0 / (a.rate * m.opt.timestep))))
        self.t0_ns = time.time_ns()
        self.rows, self.phase_log = [], []
        self.n_step, self.phase = 0, "home"
        self.rng = np.random.default_rng(a.seed)
        self.noise = 0.0 if a.no_noise else a.noise_scale
        self.effort_offset = self.rng.normal(0.0, NOISE["effort_offset"] * self.noise, 6)
        self.viewer = None
        if a.viewer:
            from mujoco import viewer
            self.viewer = viewer.launch_passive(m, d)
            self.viewer.cam.lookat[:] = [0.8, 0.0, 1.2]
            self.viewer.cam.distance, self.viewer.cam.elevation, self.viewer.cam.azimuth = 2.2, -15, 120

    def pinch_points(self, m, poses):
        """Where the tool's pinch site is in each of the arm `poses` (base at the origin)."""
        d = mujoco.MjData(m)
        arm = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in ARM_JOINTS]
        site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "pinch")
        out = []
        for q in poses:
            d.qpos[arm] = self.to_ur(q)
            mujoco.mj_kinematics(m, d)
            out.append(d.site_xpos[site].copy())
        return out

    # ── simulation ──────────────────────────────────────────────────────────
    def stamp_ns(self):
        return self.t0_ns + int(round(self.d.time * 1e9))

    def arm_q(self):
        return self.d.qpos[self.qpos_ids[3:]].copy()

    def step(self, q_ref, qd_ref):
        """One physics step tracking (q_ref, qd_ref) like MujocoSimBackend.tick (base held at zero)."""
        m, d = self.m, self.d
        cmd = np.zeros(len(self.dof_ids))
        cmd[3:] = qd_ref + KP * (q_ref - self.arm_q())
        mujoco.mj_fullM(m, d, self.M)  # MuJoCo 3.16 (pixi) has no data.qM any more
        for i, dof in enumerate(self.dof_ids):
            d.ctrl[self.vel_act[i]] = cmd[i]
            k_v = self.M[dof, dof] / 0.02
            d.ctrl[self.motor_act[i]] = d.qfrc_bias[dof] + m.dof_damping[dof] * cmd[i] + k_v * (cmd[i] - d.qvel[dof])
        self.grip.command(d, fraction=self.grip_frac)
        mujoco.mj_step(m, d)
        if self.can_goal is not None and self.phase in ("lift", "place"):  # slip of the can in the gripper
            off = d.xpos[self.can_body] - d.site_xpos[self.pinch_site]
            if self.can_off0 is None:
                self.can_off0 = off
            self.can_slip = max(self.can_slip, float(np.linalg.norm(off - self.can_off0)))
        if self.n_step % self.n_rec == 0:
            self.record()
        self.n_step += 1
        if self.viewer and self.n_step % 8 == 0:
            self.viewer.sync()
            time.sleep(max(0.0, self.d.time - (time.perf_counter() - self.wall0)))

    def record(self):
        d, k, rng = self.d, self.noise, self.rng
        if k > 0.0 and rng.random() < NOISE["drop"] * k:
            return
        header = self.stamp_ns() + int(rng.normal(0.0, NOISE["stamp_jitter"] * k) * 1e9)
        latency = (NOISE["latency"] + rng.exponential(NOISE["latency_tail"])) * k if k > 0.0 else 0.0
        bag = header + int(latency * 1e9)
        row = {"bag_time_ns": bag, "bag_time_sec": bag * 1e-9, "header_stamp_sec": header * 1e-9}
        q = d.qpos[self.qpos_ids[3:]] + rng.normal(0.0, NOISE["position"] * k, 6)
        qd = d.qvel[self.dof_ids[3:]] + rng.normal(0.0, NOISE["velocity"] * k, 6)
        tau = d.qfrc_actuator[self.dof_ids[3:]] + self.effort_offset + rng.normal(0.0, NOISE["effort"] * k, 6)
        for i in range(6):
            row[f"joint{i + 1}_position"] = q[i]
            row[f"joint{i + 1}_velocity"] = qd[i]
            row[f"joint{i + 1}_effort"] = tau[i]
        for name, qa, da in self.fingers:
            fq = d.qpos[qa]
            if k > 0.0 and self.finger_step:  # the real gripper reports whole steps
                fq = np.round(fq / self.finger_step) * self.finger_step
            row[f"{name}_position"] = fq
            row[f"{name}_velocity"] = d.qvel[da] + rng.normal(0.0, NOISE_FINGER["velocity"] * k)
            row[f"{name}_effort"] = d.qfrc_actuator[da] + rng.normal(0.0, NOISE_FINGER["effort"] * k)
        row["phase"] = self.phase
        self.rows.append(row)

    def hold(self, q, seconds):
        for _ in range(int(round(seconds / self.m.opt.timestep))):
            self.step(q, np.zeros(6))

    # ── xArm-like commands ──────────────────────────────────────────────────
    def move_to(self, target_deg):
        q0, q1 = self.arm_q(), self.to_ur(target_deg)
        T, prof = trapezoid(q1 - q0, np.radians(self.a.speed), np.radians(self.a.acc))
        t, dt = 0.0, self.m.opt.timestep
        while t < T:
            s, sd = prof(t)
            self.step(q0 + s * (q1 - q0), sd * (q1 - q0))
            t += dt
        for _ in range(int(2.0 / dt)):  # wait=True: until the arm has arrived (at most 2 s)
            if np.max(np.abs(self.arm_q() - q1)) < SETTLE_TOL:
                break
            self.step(q1, np.zeros(6))
        return q1

    def set_gripper(self, position, q_hold):
        target = self.gripper_fraction(position)
        f0, n = self.grip_frac, int(round(self.a.grip_time / self.m.opt.timestep))
        for k in range(1, n + 1):
            self.grip_frac = f0 + (target - f0) * k / n
            self.step(q_hold, np.zeros(6))

    def gripper_fraction(self, position):
        """xArm gripper position (0.1 mm of opening) -> closure fraction of the sim tool with that opening
        (Robotiq: measured pad gap; other tools: linear over the xArm's 85 mm)."""
        width = float(np.clip(position, 0.0, 850.0)) / 10.0
        if self.tool_name == "robotiq_2f85":
            frac, gap = ROBOTIQ_GAP
            return float(np.interp(width, gap[::-1], frac[::-1]))
        return 1.0 - width / 85.0

    def run(self):
        self.wall0 = time.perf_counter()
        q_hold = self.arm_q()
        self.hold(q_hold, self.a.pre_roll)  # bag running, robot not moving yet
        for name, kind, target, dwell in SEQUENCE:
            self.phase = name
            start = self.stamp_ns()
            print(f"Started phase: {name}")
            if kind == "move":
                q_hold = self.move_to(target)
            else:
                self.set_gripper(target, q_hold)
            self.hold(q_hold, dwell)
            self.phase_log.append({"phase": name, "start_time_ns": start, "end_time_ns": self.stamp_ns()})
            err = np.degrees(np.max(np.abs(self.arm_q() - q_hold)))
            print(f"Ended phase: {name}  (t = {self.d.time:.2f} s, max joint error {err:.2f} deg)")
        if self.can_goal is not None:
            err = self.d.xpos[self.can_body] - self.can_goal
            print(f"Can slip in the gripper during lift / place: {self.can_slip * 100:.1f} cm "
                  f"({'held' if self.can_slip < 0.01 else 'NOT held firmly'})")
            print(f"Can {np.linalg.norm(err) * 100:.1f} cm from the place point (xyz {np.round(err * 100, 1)} cm): "
                  f"{'placed' if np.linalg.norm(err) < 0.03 else 'NOT placed'}")
        if self.viewer:
            self.viewer.close()

    # ── output ──────────────────────────────────────────────────────────────
    def save(self, out):
        out.mkdir(parents=True, exist_ok=True)
        self.rows.sort(key=lambda r: r["bag_time_ns"])  # bag order: by receive time
        cols = [c for c in self.rows[0] if c != "phase"]
        _write_csv(out / "full_motion_joint_states.csv", self.rows, cols)
        for phase in PHASES:
            _write_csv(out / f"{phase}_joint_states.csv", [r for r in self.rows if r["phase"] == phase], cols)
        _write_csv(out / "phase_timestamps.csv", self.phase_log, ["phase", "start_time_ns", "end_time_ns"])
        print(f"Saved {len(self.rows)} rows ({self.d.time:.1f} s at {self.a.rate:g} Hz) to: {out}")


def add_objects(spec, pick, place, can_radius):
    """The can (free body) standing on a stand at the pick pinch point, and the place stand."""
    for name, top, rgba in (("pick_stand", pick[2] - CAN_HALF_H, [0.55, 0.55, 0.6, 1.0]),
                            ("place_stand", place[2] - CAN_HALF_H - PLACE_DROP, [0.3, 0.7, 0.35, 1.0])):
        xy = pick if name == "pick_stand" else place
        body = spec.worldbody.add_body(name=name, pos=[xy[0], xy[1], top / 2])
        body.add_geom(name=f"{name}_geom", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[STAND_HALF_XY, STAND_HALF_XY, top / 2], rgba=rgba)
    can = spec.worldbody.add_body(name="can", pos=list(pick))
    can.add_freejoint(name="can_free")
    can.add_geom(name="can_geom", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[can_radius, CAN_HALF_H, 0],
                 mass=CAN_MASS, friction=[1.5, 0.01, 0.001], condim=4, rgba=[0.85, 0.15, 0.15, 1.0])


def _write_csv(path, rows, fields):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--model", default=None, help="MJCF of Steve (default: steve_wbc/models/mmo_700_wbc.xml)")
    p.add_argument("--tool", default=DEFAULT_TOOL, help=f"tool on the flange: {' | '.join(TOOLS)}")
    p.add_argument("--joint_map", choices=["xarm", "direct"], default="xarm",
                   help="xarm: the waypoints converted to the UR5's zero pose; direct: applied 1:1")
    p.add_argument("--no_object", action="store_true", help="no can / stands: the arm motion alone")
    p.add_argument("--can_diameter", type=float, default=56.0,
                   help="can diameter [mm]; GRIPPER_CLOSE (52 mm) has to be smaller to squeeze it")
    p.add_argument("--no_noise", action="store_true", help="ideal sim data: no noise, jitter or drops")
    p.add_argument("--noise_scale", type=float, default=1.0, help="scale of all NOISE values")
    p.add_argument("--seed", type=int, default=None, help="random seed (default: a new one each run)")
    p.add_argument("--pre_roll", type=float, default=PRE_ROLL, help="recording before the motion [s]")
    p.add_argument("--speed", type=float, default=20.0, help="max joint speed [deg/s] (xArm SDK default)")
    p.add_argument("--acc", type=float, default=500.0, help="max joint acceleration [deg/s^2] (xArm SDK default)")
    p.add_argument("--grip_time", type=float, default=1.0, help="time the gripper takes to open / close [s]")
    p.add_argument("--rate", type=float, default=100.0, help="joint_states rate [Hz]")
    p.add_argument("--out", type=Path, default=None, help="CSV folder (default ./sim_data/pick_place_<ts>/csv)")
    p.add_argument("--viewer", action="store_true", help="MuJoCo viewer in real time (macOS: run with mjpython)")
    a = p.parse_args()
    out = a.out or Path("sim_data") / f"pick_place_{datetime.now().strftime('%Y%m%d_%H%M%S')}" / "csv"
    sim = PickPlaceSim(a)
    sim.run()
    sim.save(out)


if __name__ == "__main__":
    main()
