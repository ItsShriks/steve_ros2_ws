#!/usr/bin/env python3
"""
active_perception_wbc.py
========================
Active perception + VGN grasping for Steve (MMO-700 + UR5e + Robotiq 2F-85) with a
UNIFIED joint-velocity whole-body controller.

Every phase is one constrained QP over all 9 joints (base vx, vy, wz + 6 arm joints,
see steve_figure8_wbc.WholeBodyController) — only the active tasks change:

  phase          EE task                     base task              posture (null space)
  ─────────────  ──────────────────────────  ─────────────────────  ─────────────────────
  DRIVE          —                           LiDAR-reactive forward drive pose
  BACKUP         —                           reverse                drive pose
  SETTLE_ARM     —                           hold                   scan seed pose
  COARSE_DETECT  —                           hold                   scan pose, pan sweeps
                 (only if the object was not already tracked)
  VGN_SCAN       —                           hold                   scan pose, pan -> view i
  PRE_GRASP      MPC: meet the predicted     — (QP decides)         table-reach pose
                 path 10 cm above it
  DESCEND        MPC: onto the object        —                      table-reach pose
  GRASP          MPC: move with the object   —                      table-reach pose
  LIFT           MPC: +lift height           —                      table-reach pose

Perception runs from the very first tick — head (pan-tilt) camera while driving,
then wrist + head camera — into a gated least-squares constant-velocity tracker, so
the object's path is known long before the grasp. A receding-horizon MPC
(InterceptionMPC) plans the gripper trajectory to intercept that predicted path with
matched velocity; the WBC tracks the MPC's first step.

Hard constraints in every phase: joint position / velocity / acceleration limits and
collision avoidance (arm/base vs table, floor and own body; relaxed margin only for the
final descent). Ground truth is used only to report estimation errors.

Compared with active_perception_dynamic_wbc_vgn.py (porta_ros2_ws): there the torque
OSC was never called, reaching/grasping/lifting used arm-only IK, the base only drove
and the "tracking" read the object's ground-truth position.

Run
---
  # MuJoCo (on macOS use `mjpython` for the viewer)
  python3 active_perception_wbc.py [--headless] [--no_vgn] [--object_type box] [--log run.csv]
  ros2 launch steve_wbc active_perception_wbc.launch.py use_sim_time:=true

  # Physical robot / Gazebo: LiDAR from --scan_topic, object from --target_topic
  # (geometry_msgs/PointStamped in odom or base_link), gripper via steve_manipulation
  ros2 launch steve_wbc active_perception_wbc.launch.py use_sim_time:=false dry_run:=true
"""

import sys
import time

import mujoco
import numpy as np
from scipy.optimize import lsq_linear

try:
    from steve_wbc.steve_figure8_wbc import (
        NU, BaseReference, DryRunBackend, MujocoSimBackend, Reference, RosBackend, RobotState,
        WholeBodyController, WholeBodyKinematics, build_arg_parser, exp_so3, log_so3,
        parse_args, resolve_model, rot_z, summarize,
    )
    from steve_wbc.perception_core import (IMG_H, IMG_W, PerceptionConfig, detect_red_object,
                                           render_rgbd, unproject_pixel_to_world)
    from steve_wbc.vgn_mujoco_bridge import VGNBridge, read_lidar
except ImportError:  # plain script next to the other modules (e.g. macOS without ROS)
    from steve_figure8_wbc import (
        NU, BaseReference, DryRunBackend, MujocoSimBackend, Reference, RosBackend, RobotState,
        WholeBodyController, WholeBodyKinematics, build_arg_parser, exp_so3, log_so3,
        parse_args, resolve_model, rot_z, summarize,
    )
    from perception_core import (IMG_H, IMG_W, PerceptionConfig, detect_red_object,
                                 render_rgbd, unproject_pixel_to_world)
    from vgn_mujoco_bridge import VGNBridge, read_lidar

# ── Arm poses ────────────────────────────────────────────────────────────────
DRIVE_POSE = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]   # retracted for driving
SCAN_SEED_Q = [0.0, -1.8, 1.4, -1.2, -1.57, 0.0]             # forward-looking wide-FOV scan
ARM_HOME_Q = [0.0, -1.0, 1.5, -2.05, -1.57, 0.0]             # table reach, gripper down
R_TOPDOWN = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float)

# ── Driving (SICK S300, cf. neo_sick_s300-2) ─────────────────────────────────
LIDAR_SLOW_DIST = 0.60        # m  -> slow speed
LIDAR_STOP_DIST = 0.10        # m  -> stop
LIDAR_EMERGENCY_DIST = 0.04   # m  -> back up
SLOW_SPEED, BACKUP_SPEED = 0.10, 0.08
SIM_ARM_STOP_X = 1.20         # sim: base x giving optimal UR5e reach to the table centre

# ── Grasping ─────────────────────────────────────────────────────────────────
PREGRASP_HEIGHT = 0.10        # m above the object
LIFT_HEIGHT = 0.15            # m
EE_APPROACH_SPEED = 0.15      # m/s  max EE speed (relative to the object) on approach
EE_DESCEND_SPEED = 0.05       # m/s
EE_ROT_SPEED = 0.6            # rad/s
GRASP_TIME = 1.0              # s gripper closing while tracking


# ══════════════════════════════════════════════════════════════════════════════
# World interfaces: perception, LiDAR, gripper  (MuJoCo  |  ROS 2)
# ══════════════════════════════════════════════════════════════════════════════
class SimWorld:
    """Wrist-camera perception, rangefinder LiDAR and gripper actuator in MuJoCo."""

    def __init__(self, backend: MujocoSimBackend, args):
        self.b, self.a = backend, args
        m, d = backend.model, backend.data
        self.cfg = PerceptionConfig(cam_name="wrist_cam")
        self.renderer = mujoco.Renderer(m, IMG_H, IMG_W)
        self.obj_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "target_box")
        self.grip_act = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "fingers_actuator")
        self.half_h = 0.025  # sphere radius / cube half-size (geom size below)
        self.vgn = VGNBridge() if not args.no_vgn else None
        self._place_object()
        # The pan-tilt joints have no actuators in the MJCF: hold them with a stiff spring
        # (the real Dynamixels hold their position) so the head camera does not sag.
        for jn in ("pan_tilt_tilt_motor_joint", "pan_tilt_pan_motor_joint"):
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
            if j >= 0:
                m.jnt_stiffness[j] = 50.0
                m.qpos_spring[m.jnt_qposadr[j]] = d.qpos[m.jnt_qposadr[j]]
                m.dof_damping[m.jnt_dofadr[j]] = 5.0

    def _place_object(self):
        m, d, a = self.b.model, self.b.data, self.a
        g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "box_geom")
        if g >= 0:
            m.geom_type[g] = (mujoco.mjtGeom.mjGEOM_BOX if a.object_type == "box"
                              else mujoco.mjtGeom.mjGEOM_SPHERE)
            m.geom_size[g] = [0.025, 0.025, 0.025]
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "box_joint")
        if j >= 0:
            qa, va = m.jnt_qposadr[j], m.jnt_dofadr[j]
            vy = a.object_speed
            # resting on the table top (z = 0.80); the old script placed the 5 cm cube 1.25 cm into it
            d.qpos[qa:qa + 7] = [2.05, -0.25, 0.80 + self.half_h, 1, 0, 0, 0]
            d.qvel[va:va + 6] = [0.0, vy, 0.0, -vy / 0.025 if a.object_type == "sphere" else 0.0, 0, 0]
            mujoco.mj_forward(m, d)

    def front_distance(self):
        return read_lidar(self.b.model, self.b.data, "front_lidar")

    CAMERAS = ("wrist_cam", "pan_tilt_cam")

    def detect(self, cam="wrist_cam"):
        """Object centre in world frame from one RGB-D frame of `cam`, or None."""
        m, d = self.b.model, self.b.data
        mujoco.mj_forward(m, d)
        rgb, depth = render_rgbd(self.renderer, m, d, cam)
        found, cx, cy, _ = detect_red_object(rgb, self.cfg)
        if not found:
            return None
        pt, raw_d = unproject_pixel_to_world(m, d, cam, cx, cy, depth)
        if pt is None or raw_d < self.a.min_detect_range:
            return None  # at close range the gripper occludes part of the object -> biased
        # The blob centre pixel sees the surface point facing the camera; the object
        # centre lies one radius further along the viewing ray (exact for a sphere).
        # Shifting straight down instead biases the estimate with the viewpoint, which a
        # moving wrist camera turns into a phantom object velocity.
        cam = d.cam_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, cam)]
        ray = pt - cam
        return pt + self.half_h * ray / np.linalg.norm(ray)

    def ground_truth(self):
        return self.b.data.xpos[self.obj_body].copy()

    def support_exit_time(self, p, v):
        """Seconds until the predicted object path leaves the table top (inf if never)."""
        m, d = self.b.model, self.b.data
        g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "table_top")
        if g < 0:
            return np.inf
        lo = d.geom_xpos[g][:2] - m.geom_size[g][:2]
        hi = d.geom_xpos[g][:2] + m.geom_size[g][:2]
        t_exit = np.inf
        for ax in range(2):
            if v[ax] > 1e-4:
                t_exit = min(t_exit, (hi[ax] - p[ax]) / v[ax])
            elif v[ax] < -1e-4:
                t_exit = min(t_exit, (lo[ax] - p[ax]) / v[ax])
        return max(t_exit, 0.0)

    def gripper(self, close):
        self.b.data.ctrl[self.grip_act] = 255.0 if close else 0.0

    # VGN (simulation only)
    def vgn_available(self):
        return self.vgn is not None and self.vgn.available

    def vgn_init(self, center):
        self.vgn.init_tsdf(center)

    def vgn_view(self, center):
        mujoco.mj_forward(self.b.model, self.b.data)
        return self.vgn.collect_view(self.b.model, self.b.data, self.renderer, self.cfg.cam_name, center)

    def vgn_best(self):
        return self.vgn.best_grasp_world() if self.vgn.run_inference() else None


class RosWorld:
    """LiDAR from a LaserScan, object from a PointStamped, gripper via GripperCommand."""

    def __init__(self, node, args, get_base, dry_run):
        from geometry_msgs.msg import PointStamped
        from sensor_msgs.msg import LaserScan
        self.node, self.a, self.get_base, self.dry_run = node, args, get_base, dry_run
        self.log = node.get_logger()
        self._scan, self._target, self._target_t = np.inf, None, -np.inf
        node.create_subscription(LaserScan, args.scan_topic, self._scan_cb, 10)
        node.create_subscription(PointStamped, args.target_topic, self._target_cb, 10)
        self._grip = None
        if not dry_run:
            from control_msgs.action import GripperCommand
            from rclpy.action import ActionClient
            self._GripperCommand = GripperCommand
            self._grip = ActionClient(node, GripperCommand, args.gripper_action)

    def _scan_cb(self, msg):
        half = np.deg2rad(self.a.lidar_sector_deg) / 2
        ang = msg.angle_min + msg.angle_increment * np.arange(len(msg.ranges))
        r = np.asarray(msg.ranges, float)
        ok = (np.abs(ang) <= half) & np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        self._scan = float(r[ok].min()) if ok.any() else np.inf

    def _target_cb(self, msg):
        p = np.array([msg.point.x, msg.point.y, msg.point.z])
        frame = msg.header.frame_id.lstrip("/")
        if frame.endswith(("base_link", "base_footprint")):
            b = self.get_base()
            if b is None:
                return
            p = np.array([b[0], b[1], 0.0]) + rot_z(b[2]) @ p
        elif frame not in ("odom", "map", "world", ""):
            self.log.warn(f"target in unsupported frame '{frame}' (use odom or base_link)")
            return
        self._target, self._target_t = p, time.monotonic()

    def front_distance(self):
        return self._scan

    CAMERAS = ("target_topic",)

    def detect(self, cam=None):
        fresh = time.monotonic() - self._target_t < self.a.target_timeout
        return self._target.copy() if fresh and self._target is not None else None

    def ground_truth(self):
        return None

    def support_exit_time(self, p, v):
        return np.inf  # support surface unknown on the real robot

    def gripper(self, close):
        pos = 0.7929 if close else 0.0
        if self._grip is None:
            self.log.info(f"[dry-run] gripper -> {pos:.3f}")
            return
        if not self._grip.server_is_ready():
            self.log.warn(f"gripper action {self.a.gripper_action} not available")
            return
        goal = self._GripperCommand.Goal()
        goal.command.position, goal.command.max_effort = pos, self.a.grip_effort
        self._grip.send_goal_async(goal)

    def vgn_available(self):
        return False


# ══════════════════════════════════════════════════════════════════════════════
# Task-level state machine on top of the WBC
# ══════════════════════════════════════════════════════════════════════════════
class ObjectTracker:
    """Constant-velocity estimate: least-squares line fit over a sliding window of
    detections, with outlier gating. Objects stay on the table (no z velocity)."""

    GATE = 0.05      # m   reject detections this far from the prediction
    WINDOW = 4.0     # s   fit window
    V_MAX = 0.15     # m/s

    def __init__(self):
        self.p, self.v, self.t = None, np.zeros(3), None
        self.hist = []
        self.rejected = 0

    def update(self, p, t):
        if self.p is not None and np.linalg.norm(p - self.predict(t)) > self.GATE:
            self.rejected += 1
            return False
        self.hist = [(tt, pp) for tt, pp in self.hist if t - tt <= self.WINDOW] + [(t, p.copy())]
        T = np.array([tt for tt, _ in self.hist])
        P = np.array([pp for _, pp in self.hist])
        if len(T) >= 5 and T[-1] - T[0] > 0.5:
            A = np.column_stack([np.ones_like(T), T - t])
            coef, *_ = np.linalg.lstsq(A, P, rcond=None)   # P ≈ p(t) + v (T - t)
            self.p, self.v = coef[0], coef[1]
            self.v[2] = 0.0
            n = np.linalg.norm(self.v)
            if n > self.V_MAX:
                self.v *= self.V_MAX / n
        else:
            self.p = p.copy()
        self.t = t
        return True

    def predict(self, t):
        return self.p + self.v * (t - self.t)


class InterceptionMPC:
    """
    Receding-horizon interception planner for the EE reference.

    Each axis is a double integrator driven by acceleration a_k (step h):
        p_{k+1} = p_k + h v_k + h²/2 a_k,     v_{k+1} = v_k + h a_k
    Over the horizon k = 1..N the plan minimises
        Σ_k  w_p |p_k - o_k|²  +  w_v |v_k - ȯ_k|²  +  w_a |a_k|²     s.t.  |a_k| ≤ a_max
    where o_k is the object's PREDICTED path (constant-velocity tracker, fitted from
    perception since the start of the run) plus the phase offset (pre-grasp height,
    grasp point, ...). Matching ȯ makes the gripper arrive moving with the object.
    Condensed, each axis is a bounded least-squares problem solved exactly (BVLS);
    the first step becomes the WBC's EE reference, then the horizon recedes.
    """

    def __init__(self, p, R, horizon=15, h=0.1, w_p=1.0, w_v=0.3, w_a=0.02):
        self.p, self.v, self.R = p.copy(), np.zeros(3), R.copy()
        self.N, self.h = horizon, h
        self.k = np.arange(1, horizon + 1)
        Gp, Gv = np.zeros((horizon, horizon)), np.zeros((horizon, horizon))
        for i, kk in enumerate(self.k):
            for j in range(kk):
                Gp[i, j] = (kk - j - 0.5) * h * h
                Gv[i, j] = h
        self.sw = np.sqrt([w_p, w_v, w_a])
        self.A = np.vstack([self.sw[0] * Gp, self.sw[1] * Gv, self.sw[2] * np.eye(horizon)])
        self.Gp = Gp
        self.plan = None

    def step(self, obj_p, obj_v, offset, goal_R, a_max, v_max, dt):
        a0, plan = np.zeros(3), np.zeros((self.N, 3))
        for ax in range(3):
            o = obj_p[ax] + offset[ax] + obj_v[ax] * self.k * self.h
            free_p = self.p[ax] + self.k * self.h * self.v[ax]
            b = np.concatenate([self.sw[0] * (o - free_p),
                                self.sw[1] * (obj_v[ax] - self.v[ax]) * np.ones(self.N),
                                np.zeros(self.N)])
            acc = lsq_linear(self.A, b, bounds=(-a_max, a_max), method="bvls").x
            a0[ax] = acc[0]
            plan[:, ax] = free_p + self.Gp @ acc
        self.plan = plan
        v_new = self.v + a0 * dt
        n = np.linalg.norm(v_new - obj_v)
        if n > v_max:  # speed limit relative to the object
            v_new = obj_v + (v_new - obj_v) * (v_max / n)
        step_p = 0.5 * (self.v + v_new) * dt
        self.p, self.v = self.p + step_p, v_new
        rv = log_so3(goal_R @ self.R.T)
        ang = np.linalg.norm(rv)
        rstep = rv if ang <= EE_ROT_SPEED * dt else rv * (EE_ROT_SPEED * dt / ang)
        self.R = exp_so3(rstep) @ self.R
        return Reference(self.p.copy(), self.v.copy(), self.R.copy(), rstep / dt, "mpc")


class ActivePerceptionWBC:
    def __init__(self, backend, world, kin, args):
        self.b, self.w, self.kin, self.a = backend, world, kin, args
        self.dt = 1.0 / args.rate
        self.sim = isinstance(backend, MujocoSimBackend)
        self.d_safe = args.d_safe

    # ── helpers ─────────────────────────────────────────────────────────────
    def _enter(self, state, msg=""):
        print(f"\n[{self.state} → {state}] {msg}")
        self.state, self.t_state = state, self.t

    def _settled(self, st, q_ref, tol=0.02):
        return np.max(np.abs(np.asarray(q_ref) - st.q)) < tol and np.max(np.abs(self.u_prev)) < 0.03

    def _pan_to(self, st, p_world):
        """Shoulder-pan angle that points the arm at a world point."""
        self.kin.set_state(st)
        sh = self.kin.data.xpos[mujoco.mj_name2id(self.kin.model, mujoco.mjtObj.mjOBJ_BODY, "shoulder_link")]
        d = rot_z(st.base[2]).T @ (p_world - sh)
        return float(np.arctan2(d[1], d[0]))

    def _tracked(self):
        """Object path known well enough to act on (recent, several detections)."""
        return self.tracker.p is not None and len(self.tracker.hist) >= 5 and self.t - self.last_seen < 1.0

    def _perceive(self):
        """Perception runs in every phase from the start of the run: the head (pan-tilt)
        camera while driving, then wrist and head camera alternately. Every detection
        goes through the gated least-squares tracker."""
        if self.n_tick % self.a.render_every:
            return
        cams = self.w.CAMERAS
        if self.state in ("DRIVE", "BACKUP"):
            cams = [c for c in cams if c != "wrist_cam"] or cams
        elif self.state in ("DESCEND", "GRASP", "LIFT"):
            cams = [c for c in cams if c != "pan_tilt_cam"] or cams
        cam = cams[(self.n_tick // self.a.render_every) % len(cams)]
        p = self.w.detect(cam)
        if p is not None and self.tracker.update(p, self.t):
            self.last_seen = self.t
            self.n_seen[cam] = self.n_seen.get(cam, 0) + 1

    def _mpc(self, offset, goal_R, a_max, v_max, obj_moves=True):
        obj = self.tracker.predict(self.t)
        return self.mpc.step(obj, self.tracker.v if obj_moves else np.zeros(3), np.asarray(offset, float),
                             goal_R, a_max, v_max, self.dt)

    # ── state machine: returns (ee_ref, base_ref, q_nom) ────────────────────
    def update(self, st):
        a, t, dt = self.a, self.t, self.dt
        hold = BaseReference(self.base_hold, np.zeros(3))
        ts = t - self.t_state
        if self.state not in ("DONE", "ABORT"):
            self._perceive()

        if self.state == "DRIVE":
            front = self.w.front_distance()
            yaw = st.base[2]
            speed = a.drive_speed if front > LIDAR_SLOW_DIST else SLOW_SPEED if front > LIDAR_STOP_DIST else 0.0
            travelled = np.linalg.norm(st.base[:2] - self.drive_start[:2])
            if self.sim and st.base[0] >= SIM_ARM_STOP_X:
                speed = 0.0
            if speed == 0.0 or travelled >= a.max_drive:
                self.base_hold = st.base.copy()
                why = f"LiDAR {front:.3f} m" if front <= LIDAR_STOP_DIST else f"travelled {travelled:.2f} m"
                self._enter("BACKUP" if front < LIDAR_EMERGENCY_DIST else "SETTLE_ARM",
                            why + (f" | object tracked since t={self.tracker.hist[0][0]:.1f} s"
                                   if self.tracker.hist else " | object not seen yet"))
                return None, BaseReference(self.base_hold, np.zeros(3)), DRIVE_POSE
            return None, BaseReference(st.base.copy(), np.array([speed * np.cos(yaw), speed * np.sin(yaw), 0.0])), DRIVE_POSE

        if self.state == "BACKUP":
            if self.w.front_distance() >= LIDAR_STOP_DIST + 0.05:
                self.base_hold = st.base.copy()
                self._enter("SETTLE_ARM", "clearance restored")
                return None, BaseReference(self.base_hold, np.zeros(3)), DRIVE_POSE
            yaw = st.base[2]
            return None, BaseReference(st.base.copy(), -BACKUP_SPEED * np.array([np.cos(yaw), np.sin(yaw), 0.0])), DRIVE_POSE

        if self.state == "SETTLE_ARM":
            q = list(SCAN_SEED_Q)
            if self.tracker.p is not None:  # look at where the object is predicted to be
                q[0] = self._pan_to(st, self.tracker.predict(t))
            # the pan target follows the moving object, so allow residual motion
            if (self._settled(st, q, tol=0.04) and np.max(np.abs(self.u_prev)) < 0.06) or ts > 6.0:
                if self._tracked():
                    self._begin_grasp_planning(st, "object path known from perception since the start")
                else:
                    self.pan_cmd, self.pan_dir = st.q[0], -1.0
                    self._enter("COARSE_DETECT", "object not tracked yet -> sweeping shoulder_pan")
            return None, hold, q

        if self.state == "COARSE_DETECT":
            cfg = PerceptionConfig()
            if self.pan_cmd <= cfg.scan_start:
                self.pan_dir = 1.0
            elif self.pan_cmd >= cfg.scan_end:
                self.pan_dir = -1.0
            self.pan_cmd += self.pan_dir * cfg.scan_vel * dt
            q = list(SCAN_SEED_Q)
            q[0] = self.pan_cmd
            if self._tracked():
                self._begin_grasp_planning(st, "object found")
            elif ts > a.search_timeout:
                self._enter("ABORT", "object not found")
            return None, hold, q

        if self.state == "VGN_SCAN":
            q = list(SCAN_SEED_Q)
            if self.view_i < len(self.view_pans):
                q[0] = self.view_pans[self.view_i]
                if abs(st.q[0] - q[0]) < 0.03 and np.max(np.abs(self.u_prev)) < 0.03:
                    n = self.w.vgn_view(self.tracker.predict(t))
                    print(f"  VGN view {n}/{len(self.view_pans)} at pan={st.q[0]:.3f} rad")
                    self.view_i += 1
                return None, hold, q
            print("  running VGN inference ...")
            best = self.w.vgn_best()
            if best is not None:
                pos, rot, width, score = best
                gt = self.w.ground_truth()
                print(f"  VGN grasp {np.round(pos, 3)}  score={score:.2f}  width={width * 100:.1f} cm"
                      + (f"  error {np.linalg.norm(pos - gt) * 100:.1f} cm" if gt is not None else ""))
                if a.vgn_orientation == "yaw":
                    # keep top-down approach, rotate about z to VGN's finger axis (assumed VGN y-axis)
                    ax = rot[:, 1]
                    self.grasp_R = rot_z(np.arctan2(ax[1], ax[0]) - np.pi / 2) @ R_TOPDOWN
            else:
                print("  VGN found no grasp above threshold -> tracked position, top-down")
            self._enter("PRE_GRASP", "MPC interception of the predicted object path")
            return None, hold, q

        if self.state in ("PRE_GRASP", "DESCEND", "GRASP"):
            if t - self.last_seen > a.lost_timeout and self.state == "PRE_GRASP":
                self._enter("ABORT", f"object lost for {t - self.last_seen:.1f} s")
                return None, BaseReference(st.base.copy(), np.zeros(3)), ARM_HOME_Q
            obj, v_obj = self.tracker.predict(t), self.tracker.v
            if self.state != "GRASP":
                t_exit = self.w.support_exit_time(obj, v_obj)
                if t_exit < 1.5:
                    self._enter("ABORT", f"predicted path leaves the table in {t_exit:.1f} s — "
                                         "object cannot be intercepted on the table")
                    return None, BaseReference(st.base.copy(), np.zeros(3)), ARM_HOME_Q

            if self.state == "PRE_GRASP":
                off = [0.0, 0.0, PREGRASP_HEIGHT]
                ref = self._mpc(off, self.grasp_R, a_max=0.6, v_max=EE_APPROACH_SPEED)
                goal = obj + off
                if (np.linalg.norm(self.ee - goal) < 0.01 and np.linalg.norm(self.mpc.v - v_obj) < 0.01
                        and ts > 1.0):
                    a.d_safe = a.d_safe_grasp  # fingers may come close to the table now
                    self._enter("DESCEND", f"moving with the object above it | margin -> {a.d_safe_grasp * 100:.1f} cm")
                elif ts > 30.0:
                    self._enter("ABORT", "pre-grasp not reached")
                return ref, None, ARM_HOME_Q

            off = [0.0, 0.0, a.grasp_dz]
            ref = self._mpc(off, self.grasp_R, a_max=0.3, v_max=EE_DESCEND_SPEED)
            goal = obj + off
            if self.state == "DESCEND":
                stuck = ts > 2.0 and np.linalg.norm(self.u_prev[3:]) < 0.02  # stopped by the table margin
                if np.linalg.norm(self.ee - goal) < 0.006 or stuck or ts > 12.0:
                    self.w.gripper(True)
                    gt = self.w.ground_truth()
                    self._enter("GRASP", f"EE {np.linalg.norm(self.ee - goal) * 100:.1f} cm from grasp point"
                                + (f", {np.linalg.norm((self.ee - off) - gt) * 100:.1f} cm from true object"
                                   if gt is not None else "")
                                + (" (limited by table clearance)" if stuck else ""))
            elif ts >= GRASP_TIME:
                a.d_safe = self.d_safe
                self.lift_goal = self.ee + [0, 0, LIFT_HEIGHT]
                self._enter("LIFT", f"margin -> {self.d_safe * 100:.1f} cm")
            return ref, None, ARM_HOME_Q

        if self.state == "LIFT":
            ref = self.mpc.step(self.lift_goal, np.zeros(3), np.zeros(3), self.grasp_R, 0.4, 0.08, dt)
            if np.linalg.norm(self.ee - self.lift_goal) < 0.01 or ts > 10.0:
                gt = self.w.ground_truth()
                if gt is not None:
                    ok = gt[2] > 0.90
                    self.result = "success" if ok else "object not lifted"
                    self._enter("DONE", f"object z = {gt[2]:.3f} m -> {'LIFTED' if ok else 'not lifted'}")
                else:
                    self.result = "done"
                    self._enter("DONE", "lift complete")
            return ref, None, ARM_HOME_Q

        return None, BaseReference(st.base.copy(), np.zeros(3)), list(st.q)

    def _begin_grasp_planning(self, st, why):
        obj = self.tracker.predict(self.t)
        gt = self.w.ground_truth()
        err = f" | estimate error {np.linalg.norm(obj - gt) * 100:.1f} cm" if gt is not None else ""
        print(f"  object at {np.round(obj, 3)}, velocity {np.round(self.tracker.v, 3)} m/s "
              f"({len(self.tracker.hist)} detections in window, {self.tracker.rejected} rejected){err}")
        if self.w.vgn_available():
            c = self._pan_to(st, obj)
            self.view_pans = list(np.linspace(c - 0.20, c + 0.20, self.a.vgn_views))
            self.view_i = 0
            self.w.vgn_init(obj)
            self._enter("VGN_SCAN", f"{why}; {self.a.vgn_views} TSDF views around pan={c:.2f} rad")
        else:
            self._enter("PRE_GRASP", f"{why}; MPC interception of the predicted path (VGN off)")

    # ── main loop ─────────────────────────────────────────────────────────────
    def run(self):
        a = self.a
        st = self.b.wait_for_state(a.wait_timeout)
        if st is None:
            return 1
        self.ctrl = WholeBodyController(self.kin, a, q_nom=DRIVE_POSE, yaw_nom=st.base[2])
        self.u_prev = np.zeros(NU)
        self.state, self.t, self.t0 = "DRIVE", 0.0, self.b.now()
        self.t_state, self.n_tick = 0.0, 0
        self.base_hold, self.drive_start = st.base.copy(), st.base.copy()
        self.tracker, self.last_seen, self.n_seen = ObjectTracker(), -np.inf, {}
        self.grasp_R, self.result = R_TOPDOWN.copy(), "aborted"
        self.kin.set_state(st)
        p0, R0 = self.kin.ee_pose()
        self.mpc, self.ee = None, p0
        rows, next_print = [], 0.0
        print(f"\n{'═' * 74}\n  Steve active perception + grasping — unified joint-velocity WBC  [{a.mode}]"
              f"\n{'═' * 74}\n  VGN: {'on' if self.w.vgn_available() else 'off'}   object: "
              f"{getattr(a, 'object_type', '-')}   collision env: {a.collision_env}")

        try:
            while self.b.ok() and self.state not in ("DONE", "ABORT"):
                self.t = self.b.now() - self.t0
                st = self.b.read_state()
                if st is None:
                    self._enter("ABORT", "no valid robot state")
                    break
                self.kin.set_state(st)
                self.ee, R_ee = self.kin.ee_pose()
                if self.mpc is None and self.state == "PRE_GRASP":
                    self.mpc = InterceptionMPC(self.ee, R_ee)

                ee_ref, base_ref, q_nom = self.update(st)
                if ee_ref is None and self.state in ("PRE_GRASP", "DESCEND", "GRASP", "LIFT"):
                    continue  # state just changed; next tick produces the reference
                self.ctrl.q_nom = np.asarray(q_nom, float)
                self.ctrl.yaw_nom = self.base_hold[2]
                tic = time.perf_counter()
                u, info = self.ctrl.step(st, ee_ref, self.u_prev, self.dt, base_ref)
                solve_ms = (time.perf_counter() - tic) * 1e3
                if not np.all(np.isfinite(u)):
                    self._enter("ABORT", "non-finite command")
                    break
                if ee_ref is not None and np.linalg.norm(info["e_p"]) > a.max_track_err:
                    self._enter("ABORT", f"EE tracking error {np.linalg.norm(info['e_p']) * 100:.1f} cm")
                    break

                self.b.send(u)
                if self.sim and ee_ref is not None:
                    self.b.draw_live(ee_ref.p, self.ee)
                if ee_ref is not None:
                    rows.append([self.t, *ee_ref.p, *self.ee, np.linalg.norm(info["e_p"]),
                                 np.linalg.norm(info["e_o"]), *u, *st.base, *st.q, solve_ms, info["d_min"]])
                if self.t >= next_print:
                    next_print += 1.0
                    gt = self.w.ground_truth()
                    est = (f" obj_est_err={np.linalg.norm(self.tracker.predict(self.t) - gt) * 100:.1f}cm"
                           if gt is not None and self.tracker.p is not None else "")
                    clr = f" clr={info['d_min'] * 100:.1f}cm" if np.isfinite(info["d_min"]) else ""
                    print(f"  t={self.t:5.1f}s [{self.state:13s}] base=({st.base[0]:+.2f},{st.base[1]:+.2f},"
                          f"{np.degrees(st.base[2]):+4.0f}°) u_b=({u[0]:+.2f},{u[1]:+.2f},{u[2]:+.2f}) "
                          f"|dq|={np.abs(u[3:]).max():.2f}{clr}{est}")
                self.u_prev = u
                self.n_tick += 1
                self.b.tick()
        except KeyboardInterrupt:
            print("\n[STOP] Ctrl+C")
        finally:
            self.a.d_safe = self.d_safe
            self.b.stop()

        print(f"\n{'─' * 74}\n  Result: {self.result}  (final state {self.state}, t={self.t:.1f} s)")
        print(f"  Detections used: {self.n_seen}   rejected by the gate: {self.tracker.rejected}")
        summarize(rows, a)
        return 0 if self.result in ("success", "done") else 1


def main(argv=None):
    p = build_arg_parser("Steve active perception + VGN grasping — unified joint-velocity WBC")
    g = p.add_argument_group("active perception")
    g.add_argument("--object_type", choices=["sphere", "box"], default="sphere", help="sim object")
    g.add_argument("--object_speed", type=float, default=0.02, help="sim object speed along +y [m/s]")
    g.add_argument("--no_vgn", action="store_true", help="skip the VGN scan (coarse position, top-down)")
    g.add_argument("--vgn_views", type=int, default=4)
    g.add_argument("--vgn_orientation", choices=["topdown", "yaw"], default="topdown",
                   help="topdown, or top-down rotated to VGN's finger axis")
    g.add_argument("--render_every", type=int, default=3, help="perception every N control ticks")
    g.add_argument("--min_detect_range", type=float, default=0.20,
                   help="ignore detections closer than this to the camera [m] (occlusion)")
    g.add_argument("--drive_speed", type=float, default=0.25, help="[m/s]")
    g.add_argument("--max_drive", type=float, default=1.5, help="max drive distance [m]")
    g.add_argument("--grasp_dz", type=float, default=-0.015,
                   help="pinch point relative to object centre [m] (pads then close at the equator)")
    g.add_argument("--d_safe_grasp", type=float, default=0.005, help="collision margin for the final descent [m]")
    g.add_argument("--search_timeout", type=float, default=30.0, help="[s]")
    g.add_argument("--lost_timeout", type=float, default=3.0, help="abort if the object is unseen for [s]")
    g = p.add_argument_group("active perception — ROS")
    g.add_argument("--scan_topic", default="/lidar_1/scan_filtered", help="front LiDAR (sensor_msgs/LaserScan)")
    g.add_argument("--lidar_sector_deg", type=float, default=60.0, help="sector around the scan's 0° used")
    g.add_argument("--target_topic", default="/steve_perception/target_point",
                   help="object position (geometry_msgs/PointStamped, odom or base_link frame)")
    g.add_argument("--target_timeout", type=float, default=0.5, help="max age of a target message [s]")
    g.add_argument("--gripper_action", default="/gripper_controller/gripper_cmd")
    g.add_argument("--grip_effort", type=float, default=50.0)
    p.set_defaults(sim_q0=DRIVE_POSE, max_base_v=0.30, max_base_w=0.50, max_arm_vel=0.8)
    a = parse_args(argv, p)
    if a.mode == "kin":
        sys.exit("active_perception_wbc needs perception: use --mode sim or --mode ros")

    model = mujoco.MjModel.from_xml_path(str(resolve_model(a.model)))
    kin = WholeBodyKinematics(model)
    dt = 1.0 / a.rate
    if a.mode == "sim":
        backend = MujocoSimBackend(model, a, dt)
        world = SimWorld(backend, a)
        if backend.viewer:
            backend.viewer.cam.lookat[:] = [1.4, 0.0, 0.8]
            backend.viewer.cam.distance, backend.viewer.cam.elevation = 3.5, -25
    else:
        try:
            ros = RosBackend(a, dt)
        except ImportError:
            sys.exit("[ERROR] rclpy not found — source ROS 2 first")
        backend = DryRunBackend(ros, dt) if a.dry_run else ros
        # base pose for base_link -> odom target transforms (the shadow state in a dry run)
        get_base = ((lambda: backend.st.base.copy()) if a.dry_run
                    else (lambda: None if ros.base is None else ros.base.copy()))
        world = RosWorld(ros.node, a, get_base, a.dry_run)
    try:
        return ActivePerceptionWBC(backend, world, kin, a).run()
    finally:
        backend.shutdown()


if __name__ == "__main__":
    sys.exit(main())
