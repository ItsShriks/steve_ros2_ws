#!/usr/bin/env python3
"""
steve_figure8_vertical_ros2.py
==============================
Real Robot (and Hardware-in-the-Loop Simulation) Vertical Figure-8 WBC Controller
for Steve Butler (MMO-700 Base + UR5e Arm).

Description:
------------
Executes a vertical figure-8 (lemniscate) motion in the frontal YZ plane:
  - Base Y (lateral): Slides side-to-side to assist in lateral reach.
  - Arm (UR5e): Kinematic WBC (Jacobian pseudoinverse IK) tracks full EE (X, Y, Z)
                holding constant forward reach depth X and tracing height Z,
                while compensating for residual base Y motion.

ROS 2 Interfaces:
-----------------
  - Subscribes: /joint_states (sensor_msgs/msg/JointState)
  - Publishes : /cmd_vel (geometry_msgs/msg/Twist) for omnidirectional base
  - Publishes : /scaled_joint_trajectory_controller/joint_trajectory
                or /joint_trajectory_controller/joint_trajectory (trajectory_msgs/msg/JointTrajectory)

Safety Features:
----------------
  - Max base Y velocity clamping (default: 0.25 m/s)
  - Max arm joint velocity clamping (default: 0.50 rad/s)
  - Smooth initial ramp into trajectory start
  - Emergency stop / graceful exit on Ctrl+C (sends 0 cmd_vel)
  - Dry-run mode option (`--dry_run`) to test motion logic without sending hardware commands

Run on Steve:
-------------
  cd ~/steve_ros2_ws
  python3 scripts/steve_figure8_vertical_ros2.py [--amplitude 0.15] [--loops 3] [--dry_run]
"""

import argparse
import sys
import time
from pathlib import Path
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = object
    Twist = object
    JointState = object
    JointTrajectory = object
    JointTrajectoryPoint = object

import mujoco

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_DIR = SCRIPT_DIR.parent.parent
MODEL_PATH = WORKSPACE_DIR / "porta_ros2_ws" / "robots" / "mmo_700_wbc.xml"

if not MODEL_PATH.exists():
    # Fallback lookup relative to home or current workspace
    MODEL_PATH = Path.home() / "Master_Thesis" / "porta_ros2_ws" / "robots" / "mmo_700_wbc.xml"

# ── Arm Joint Names (Standard UR5e vs Steve prefixes) ─────────────────────────
STANDARD_ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Alternate joint prefixes used in ROS 2 hardware drivers
ALT_ARM_JOINTS_MAP = {
    "ur5e_": ["ur5e_shoulder_pan_joint", "ur5e_shoulder_lift_joint", "ur5e_elbow_joint",
              "ur5e_wrist_1_joint", "ur5e_wrist_2_joint", "ur5e_wrist_3_joint"],
    "ur5":   ["ur5eshoulder_pan_joint", "ur5eshoulder_lift_joint", "ur5eelbow_joint",
              "ur5ewrist_1_joint", "ur5ewrist_2_joint", "ur5ewrist_3_joint"]
}


class DummyLogger:
    def info(self, msg): print(f"[INFO] {msg}")
    def warn(self, msg): print(f"[WARN] {msg}")
    def error(self, msg): print(f"[ERROR] {msg}")

class SteveVerticalFigure8Node(Node):
    def __init__(self, args):
        if ROS2_AVAILABLE:
            super().__init__("steve_figure8_vertical_controller")
            self._logger = self.get_logger()
        else:
            self._logger = DummyLogger()

        self.args = args
        self.amplitude = args.amplitude
        self.frequency = args.frequency
        self.loops = args.loops
        self.dry_run = args.dry_run
        self.cz_target = args.cz

        self.log_info("Initializing Steve Vertical Figure-8 WBC Node...")

        # ── 1. Load Kinematic Engine (MuJoCo XML) ─────────────────────────────
        if not MODEL_PATH.exists():
            self.log_error(f"MuJoCo model not found at {MODEL_PATH}")
            sys.exit(1)

        self.log_info(f"Loading kinematic solver model: {MODEL_PATH}")
        self.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        self.data = mujoco.MjData(self.model)

        self.pinch_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "pinch")
        self.mj_arm_jids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)
                            for j in STANDARD_ARM_JOINTS]
        self.mj_arm_qpos_ids = [self.model.jnt_qposadr[jid] for jid in self.mj_arm_jids]
        self.mj_arm_dof_ids = [self.model.jnt_dofadr[jid] for jid in self.mj_arm_jids]

        # Base DOF indices in MuJoCo model
        self.base_x_qpos = self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "base_x")]
        self.base_y_qpos = self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "base_y")]

        # ── 2. Joint State Tracking ───────────────────────────────────────────
        self.current_arm_q = None
        self.joint_name_map = {}
        if ROS2_AVAILABLE:
            self.sub_joint_states = self.create_subscription(
                JointState,
                "/joint_states",
                self._joint_state_cb,
                10
            )

            # ── 3. Publishers ─────────────────────────────────────────────────────
            self.pub_cmd_vel = self.create_publisher(Twist, args.cmd_vel_topic, 10)
            self.pub_arm_traj = self.create_publisher(JointTrajectory, args.arm_topic, 10)

        # Active joint names sent to arm trajectory controller
        self.active_ros_joint_names = STANDARD_ARM_JOINTS

        # Forward facing target orientation matrix for end-effector
        # (wrist pointing +X forward)
        self.R_FORWARD = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], dtype=float)

        self.log_info("Node initialized.")

    def log_info(self, msg):
        self._logger.info(msg)

    def log_warn(self, msg):
        self._logger.warn(msg)

    def log_error(self, msg):
        self._logger.error(msg)

    def _joint_state_cb(self, msg: JointState):
        """Callback to store joint state feedback and resolve joint name prefixes."""
        if not self.joint_name_map:
            # Detect prefix in published joint names
            for target_name in STANDARD_ARM_JOINTS:
                if target_name in msg.name:
                    self.joint_name_map[target_name] = target_name
                else:
                    # Check alternate prefixes
                    for alt_prefix, alt_list in ALT_ARM_JOINTS_MAP.items():
                        for alt_name in alt_list:
                            if alt_name in msg.name and target_name in alt_name:
                                self.joint_name_map[target_name] = alt_name

            if self.joint_name_map:
                self.active_ros_joint_names = [self.joint_name_map.get(j, j) for j in STANDARD_ARM_JOINTS]
                self.get_logger().info(f"Mapped ROS 2 Arm Joint Names: {self.active_ros_joint_names}")

        # Extract current arm joint positions
        if len(self.joint_name_map) == 6:
            q_cur = []
            for jname in STANDARD_ARM_JOINTS:
                ros_name = self.joint_name_map[jname]
                idx = msg.name.index(ros_name)
                q_cur.append(msg.position[idx])
            self.current_arm_q = np.array(q_cur, dtype=float)

    def wait_for_joints(self, timeout_sec=10.0):
        """Block until /joint_states is received or set fallback for dry-run."""
        if not ROS2_AVAILABLE or self.dry_run:
            if self.current_arm_q is None:
                # Default home reach pose: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
                self.current_arm_q = np.array([0.0, -1.2, 1.2, -1.6, -1.57, 0.0], dtype=float)
                self.log_warn("Using default dry-run initial arm joint angles.")
            return True

        start_t = time.time()
        while rclpy.ok() and self.current_arm_q is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_t > timeout_sec:
                self.log_error("Timeout waiting for /joint_states!")
                return False
        return True

    def compute_kinematic_wbc(self, target_pos, target_vel):
        """
        Velocity-resolved IK for the 6-DOF UR5e arm.
        """
        Jp = np.zeros((3, self.model.nv))
        Jr = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, Jp, Jr, self.pinch_id)

        J6 = np.vstack([Jp[:, self.mj_arm_dof_ids], Jr[:, self.mj_arm_dof_ids]])

        p_ee = self.data.site_xpos[self.pinch_id]
        p_err = target_pos - p_ee

        # Rotation error axis-angle
        R_cur = self.data.site_xmat[self.pinch_id].reshape(3, 3)
        R_err = self.R_FORWARD @ R_cur.T
        angle = np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1.0, 1.0))
        if abs(angle) > 1e-6:
            ax = np.array([R_err[2, 1] - R_err[1, 2],
                           R_err[0, 2] - R_err[2, 0],
                           R_err[1, 0] - R_err[0, 1]]) / (2 * np.sin(angle))
            r_err = angle * ax
        else:
            r_err = np.zeros(3)

        v_des = np.concatenate([25.0 * p_err + target_vel, 8.0 * r_err])

        # Damped pseudoinverse
        lambda_sq = 1e-4
        dq = J6.T @ np.linalg.solve(J6 @ J6.T + lambda_sq * np.eye(6), v_des)
        return dq

    def send_base_velocity(self, vy: float):
        """Publish linear Y velocity to /cmd_vel with safety clamping."""
        vy_clamped = float(np.clip(vy, -self.args.max_base_vy, self.args.max_base_vy))
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = vy_clamped
        twist.angular.z = 0.0

        if not self.dry_run:
            self.pub_cmd_vel.publish(twist)

    def send_arm_trajectory_point(self, target_q: np.ndarray, target_dq: np.ndarray, dt: float):
        """Publish JointTrajectory message for UR5e controller."""
        traj = JointTrajectory()
        traj.joint_names = self.active_ros_joint_names

        pt = JointTrajectoryPoint()
        pt.positions = target_q.tolist()
        pt.velocities = target_dq.tolist()
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = int(dt * 1e9)

        traj.points.append(pt)

        if not self.dry_run:
            self.pub_arm_traj.publish(traj)

    def run_vertical_figure8(self):
        """Execute Phase 2 Vertical Frontal Figure-8."""
        if not self.wait_for_joints():
            return

        self.log_info("Setting up initial kinematic state...")

        # Update MuJoCo solver with current joint positions
        for i, qi in enumerate(self.mj_arm_qpos_ids):
            self.data.qpos[qi] = self.current_arm_q[i]
        self.data.qpos[self.base_x_qpos] = 0.0
        self.data.qpos[self.base_y_qpos] = 0.0
        mujoco.mj_forward(self.model, self.data)

        ee_init = self.data.site_xpos[self.pinch_id].copy()
        cx = float(ee_init[0])          # fixed reach depth in X
        cy_world = float(ee_init[1])    # lateral centre in Y
        cz = float(self.cz_target if self.cz_target is not None else ee_init[2])

        self.log_info(f"Start End-Effector Pose : [{ee_init[0]:.3f}, {ee_init[1]:.3f}, {ee_init[2]:.3f}]")
        self.log_info(f"Figure-8 Centre (XYZ)   : [{cx:.3f}, {cy_world:.3f}, {cz:.3f}]")
        self.log_info(f"Amplitude: {self.amplitude:.3f}m | Frequency: {self.frequency:.3f} rad/s")

        omega = self.frequency
        period = 2.0 * np.pi / omega
        total_duration = self.loops * period
        dt = 0.02  # 50 Hz update rate

        rate_sec = dt
        sim_t = 0.0

        current_q_cmd = self.current_arm_q.copy()
        base_y_sim = 0.0

        self.log_info(f"Starting Vertical Figure-8 execution for {self.loops} loops (~{total_duration:.1f}s)...")

        try:
            while (rclpy.ok() if ROS2_AVAILABLE else True) and sim_t <= total_duration:
                t_loop_start = time.time()

                # Process ROS callbacks for fresh state
                if ROS2_AVAILABLE:
                    rclpy.spin_once(self, timeout_sec=0.0)

                # ── Lemniscate Target equations (frontal YZ plane) ───────────
                s = np.sin(omega * sim_t)
                c = np.cos(omega * sim_t)

                y_target = cy_world + self.amplitude * s * c
                z_target = cz + self.amplitude * s
                x_target = cx

                dy_target = self.amplitude * omega * (c * c - s * s)
                dz_target = self.amplitude * omega * c
                dx_target = 0.0

                tgt_pos = np.array([x_target, y_target, z_target])
                tgt_vel = np.array([dx_target, dy_target, dz_target])

                # 1.5s smooth ramp-in from initial pose to avoid trajectory jump
                ramp = min(1.0, sim_t / 1.5)
                tgt_pos = (1.0 - ramp) * ee_init + ramp * tgt_pos
                tgt_vel = tgt_vel * ramp

                # ── Base Y Motion ─────────────────────────────────────────────
                # Base tracks the Y component of the figure-8
                vy_ff = tgt_vel[1]
                vy_cmd = ramp * (vy_ff + 3.0 * (y_target - base_y_sim))
                self.send_base_velocity(vy_cmd)
                base_y_sim += vy_cmd * dt

                # Update MuJoCo base pose for IK solver
                self.data.qpos[self.base_y_qpos] = base_y_sim
                for i, qi in enumerate(self.mj_arm_qpos_ids):
                    self.data.qpos[qi] = current_q_cmd[i]
                mujoco.mj_forward(self.model, self.data)

                # ── Arm Kinematic WBC ─────────────────────────────────────────
                dq_arm = self.compute_kinematic_wbc(tgt_pos, tgt_vel)

                # Clamp arm joint velocities for physical robot safety
                dq_arm = np.clip(dq_arm, -self.args.max_arm_vel, self.args.max_arm_vel)

                # Integrate joint command
                current_q_cmd += dq_arm * dt

                # Send command to arm trajectory controller
                if ROS2_AVAILABLE:
                    self.send_arm_trajectory_point(current_q_cmd, dq_arm, dt)

                if int(sim_t / dt) % 25 == 0:  # Log every 0.5s
                    ee_now = self.data.site_xpos[self.pinch_id]
                    err = np.linalg.norm(ee_now - tgt_pos)
                    self.log_info(
                        f"t={sim_t:5.1f}s | EE=[{ee_now[0]:+.3f}, {ee_now[1]:+.3f}, {ee_now[2]:+.3f}] | "
                        f"Err={err*100:.1f}cm | Base Vy={vy_cmd:+.3f}m/s"
                    )

                sim_t += dt
                elapsed = time.time() - t_loop_start
                sleep_time = rate_sec - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            self.get_logger().warn("KeyboardInterrupt caught! Stopping robot...")
        finally:
            self.stop_robot()

    def stop_robot(self):
        """Send zero velocity to base on stop."""
        self.get_logger().info("Stopping robot motion...")
        self.send_base_velocity(0.0)
        self.get_logger().info("Robot safely stopped.")


def main():
    parser = argparse.ArgumentParser(description="Steve Vertical Figure-8 WBC Controller (ROS 2 Real Robot)")
    parser.add_argument("--amplitude", type=float, default=0.15, help="Figure-8 half amplitude in meters (default: 0.15)")
    parser.add_argument("--frequency", type=float, default=0.45, help="Figure-8 frequency omega in rad/s (default: 0.45)")
    parser.add_argument("--loops", type=int, default=3, help="Number of complete figure-8 loops (default: 3)")
    parser.add_argument("--cz", type=float, default=None, help="Target Z centre height in meters (default: current EE height)")
    parser.add_argument("--max_base_vy", type=float, default=0.25, help="Max base Y velocity limit in m/s (default: 0.25)")
    parser.add_argument("--max_arm_vel", type=float, default=0.50, help="Max arm joint velocity limit in rad/s (default: 0.50)")
    parser.add_argument("--cmd_vel_topic", type=str, default="/cmd_vel", help="Base velocity command topic")
    parser.add_argument("--arm_topic", type=str, default="/scaled_joint_trajectory_controller/joint_trajectory",
                        help="Arm joint trajectory command topic")
    parser.add_argument("--dry_run", action="store_true", help="Run without publishing control commands to ROS 2 hardware")

    args = parser.parse_args(sys.argv[1:])

    if not ROS2_AVAILABLE and not args.dry_run:
        print("[ERROR] rclpy is not installed or ROS 2 environment is not sourced.")
        print("Please source ROS 2 first: source /opt/ros/humble/setup.bash")
        sys.exit(1)

    if ROS2_AVAILABLE:
        rclpy.init()
    node = SteveVerticalFigure8Node(args)
    try:
        node.run_vertical_figure8()
    finally:
        if ROS2_AVAILABLE:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
