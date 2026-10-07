#!/usr/bin/env python3
"""
Simple arm + gripper API for Steve (UR5e + Robotiq 2F-85) on top of MoveIt 2.

Python:
    import rclpy
    from steve_manipulation.manipulator import SteveManipulator

    rclpy.init()
    arm = SteveManipulator()
    arm.move_to_named("ready")
    arm.move_to_pose([0.6, 0.0, 1.0], rpy=[3.14159, 0.0, 0.0])   # gripper_tcp in base_link
    arm.close_gripper()
    arm.destroy_node(); rclpy.shutdown()

CLI  (ros2 run steve_manipulation manipulator ...):
    list
    named ready
    joints 0 -1.57 1.57 -1.57 -1.57 0
    --direct named table_reach              (no MoveIt: straight to the trajectory controller,
    --direct --duration 20 joints ...        no collision checking)
    pose 0.6 0.0 1.0 --rpy 3.14159 0 0 [--frame base_link]
    gripper open | close | 0.4
    ... [--vel 0.2] [--acc 0.2] [--plan_only]

Works identically in Gazebo and on the real robot (the launch file maps the
controllers); the gripper action is /gripper_controller/gripper_cmd in both.

Safety stop (steve_manipulation/safety_stop.py, --safety): no arm or gripper motion is started
while the robot's EM-stop system reports a stop (EM stop, scanner protective field), and a
running arm motion is cancelled as soon as it does. Without an EM-stop system (Gazebo) the
LiDAR fallback is used; on the real robot a missing EM-stop state blocks motion.
"""

import argparse
import math
import os
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory, GripperCommand
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, JointConstraint, MoveItErrorCodes,
                             OrientationConstraint, PositionConstraint)
from shape_msgs.msg import SolidPrimitive

from steve_manipulation.safety_stop import SCAN_TOPICS, EmStopMonitor, LidarSafety, ScanPoints

ARM_GROUP = "ur_manipulator"
GRIPPER_GROUP = "gripper"
TCP_LINK = "gripper_tcp"
BASE_FRAME = "base_link"
GRIPPER_JOINT = "robotiq_85_left_knuckle_joint"
GRIPPER_OPEN, GRIPPER_CLOSED = 0.0, 0.7929
ERROR_NAMES = {v: k for k, v in vars(MoveItErrorCodes).items() if k.isupper() and isinstance(v, int)}


def rpy_to_quat(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy]


def load_named_states(srdf_path=None):
    """{group: {state: {joint: value}}} from the package SRDF."""
    if srdf_path is None:
        from ament_index_python.packages import get_package_share_directory
        srdf_path = os.path.join(get_package_share_directory("steve_manipulation"), "config", "steve.srdf")
    states = {}
    for gs in ET.parse(srdf_path).getroot().findall("group_state"):
        states.setdefault(gs.get("group"), {})[gs.get("name")] = {
            j.get("name"): float(j.get("value")) for j in gs.findall("joint")}
    return states


class SteveManipulator(Node):
    def __init__(self, node_name="steve_manipulator", velocity_scaling=0.2, acceleration_scaling=0.2,
                 planning_time=5.0, gripper_action="/gripper_controller/gripper_cmd", safety="auto"):
        super().__init__(node_name)
        self._safety_mode, self._safety = safety, None
        self._em = EmStopMonitor(self) if safety in ("auto", "em_stop") else None
        self.velocity_scaling = velocity_scaling
        self.acceleration_scaling = acceleration_scaling
        self.planning_time = planning_time
        self.named = load_named_states()
        self._move = ActionClient(self, MoveGroup, "move_action")
        self._grip = ActionClient(self, GripperCommand, gripper_action)
        self._gripper_action = gripper_action

    # ── safety stop ─────────────────────────────────────────────────────────
    def _spin_for(self, cond, timeout):
        t0 = time.monotonic()
        while not cond() and time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _safety_source(self):
        """EM-stop system if it runs (required with safety=em_stop); LiDAR only without one."""
        if self._safety is None and self._safety_mode != "off":
            if self._em is not None:
                self._spin_for(lambda: self._em.available, 3.0)
            if self._em is not None and (self._em.available or self._safety_mode == "em_stop"):
                self._safety = self._em
                src = "EM-stop system"
            else:
                sim = bool(self.get_parameter("use_sim_time").value)
                scans = ScanPoints(self, SCAN_TOPICS["gazebo" if sim else "real"])
                self._safety = LidarSafety(scans.points)
                self._spin_for(lambda: scans.points() is not None, 3.0)
                src = f"LiDAR fallback ({', '.join(scans.topics)})"
            self.get_logger().info(f"safety stop source: {src}")
        return self._safety

    def safety_status(self):
        """(red, reason) of the safety stop; (False, '') with safety=off."""
        src = self._safety_source()
        return src.status() if src is not None else (False, "")

    def _check_safe_to_move(self):
        red, why = self.safety_status()
        if red:
            raise RuntimeError(f"safety stop active ({why}) — not moving")

    # ── helpers ─────────────────────────────────────────────────────────────
    def _wait(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError("action did not answer in time")
        return future.result()

    def _execute(self, group, constraints, plan_only=False, timeout=60.0):
        if not self._move.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("MoveIt 'move_action' server not available — is move_group running?")
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = group
        req.num_planning_attempts = 5
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = self.velocity_scaling
        req.max_acceleration_scaling_factor = self.acceleration_scaling
        req.goal_constraints = [constraints]
        req.workspace_parameters.header.frame_id = BASE_FRAME
        req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y = -2.0
        req.workspace_parameters.min_corner.z = -0.5
        req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y = 2.0
        req.workspace_parameters.max_corner.z = 2.5
        req.start_state.is_diff = True  # plan from the current state
        goal.planning_options.plan_only = plan_only
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        if not plan_only:
            self._check_safe_to_move()
        handle = self._wait(self._move.send_goal_async(goal), 10.0)
        if not handle.accepted:
            raise RuntimeError("MoveIt rejected the goal")
        fut, t0 = handle.get_result_async(), time.monotonic()
        while not fut.done():  # watch the safety stop while MoveIt executes
            rclpy.spin_once(self, timeout_sec=0.05)
            red, why = (False, "") if plan_only else self.safety_status()
            if red:
                self.get_logger().error(f"[SAFETY STOP] {why} -> cancelling the arm motion")
                self._wait(handle.cancel_goal_async(), 5.0)
                return False
            if time.monotonic() - t0 > timeout:
                raise TimeoutError("action did not answer in time")
        result = fut.result().result
        code = result.error_code.val
        ok = code == MoveItErrorCodes.SUCCESS
        (self.get_logger().info if ok else self.get_logger().error)(
            f"{group}: {'planned' if plan_only else 'executed'} -> {ERROR_NAMES.get(code, code)}")
        return ok

    @staticmethod
    def _joint_constraints(joints, tol=0.001):
        c = Constraints()
        for name, value in joints.items():
            c.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(value), tolerance_above=tol, tolerance_below=tol, weight=1.0))
        return c

    # ── arm ─────────────────────────────────────────────────────────────────
    def named_states(self, group=ARM_GROUP):
        return sorted(self.named.get(group, {}))

    def move_to_named(self, name, plan_only=False):
        for group in (ARM_GROUP, GRIPPER_GROUP):
            if name in self.named.get(group, {}):
                return self._execute(group, self._joint_constraints(self.named[group][name]), plan_only)
        raise KeyError(f"unknown named state '{name}' — known: {self.named}")

    # ── direct (no MoveIt) ───────────────────────────────────────────────────
    def move_direct(self, positions, duration=None, max_vel=1.0,
                    action="/scaled_joint_trajectory_controller/follow_joint_trajectory"):
        """Joint goal straight to the arm's trajectory controller — no MoveIt, so NO collision
        checking: a joint-space straight line. positions: dict {joint: rad} or 6 values (UR order).
        duration: seconds; default from the largest joint change at velocity_scaling * max_vel."""
        if not isinstance(positions, dict):
            names = list(next(iter(self.named[ARM_GROUP].values())).keys())
            if len(positions) != len(names):
                raise ValueError(f"expected {len(names)} joint values")
            positions = dict(zip(names, positions))
        self._check_safe_to_move()
        cur = {}
        sub = self.create_subscription(JointState, "/joint_states",
                                       lambda m: cur.update(zip(m.name, m.position)), 10)
        self._spin_for(lambda: all(j in cur for j in positions), 3.0)
        self.destroy_subscription(sub)
        if not all(j in cur for j in positions):
            raise RuntimeError("no /joint_states for the arm joints")
        dq = max(abs(positions[j] - cur[j]) for j in positions)
        T = float(duration) if duration else max(3.0, dq / max(1e-3, self.velocity_scaling * max_vel))
        client = ActionClient(self, FollowJointTrajectory, action)
        if not client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"{action} not available (trajectory controller active?)")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(positions)
        pt = JointTrajectoryPoint(positions=[float(positions[j]) for j in positions])
        pt.time_from_start.sec, pt.time_from_start.nanosec = int(T), int((T % 1) * 1e9)
        goal.trajectory.points = [pt]
        self.get_logger().info(f"direct move: largest joint change {dq:.2f} rad in {T:.1f} s (no collision check)")
        handle = self._wait(client.send_goal_async(goal), 5.0)
        if not handle.accepted:
            raise RuntimeError("trajectory goal rejected (controller inactive / program not playing?)")
        res = handle.get_result_async()
        t_end = time.monotonic() + T + 10.0
        while not res.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            red, why = self.safety_status()
            if red:
                handle.cancel_goal_async()
                raise RuntimeError(f"safety stop ({why}) — motion cancelled")
            if time.monotonic() > t_end:
                handle.cancel_goal_async()
                raise TimeoutError("direct move did not finish in time")
        code = res.result().result.error_code
        if code != 0:
            raise RuntimeError(f"trajectory controller error {code}: {res.result().result.error_string}")
        return True

    def move_to_joints(self, positions, plan_only=False):
        """positions: dict {joint: rad} or 6 values in UR order (pan, lift, elbow, w1, w2, w3)."""
        if not isinstance(positions, dict):
            names = list(next(iter(self.named[ARM_GROUP].values())).keys())
            if len(positions) != len(names):
                raise ValueError(f"expected {len(names)} joint values")
            positions = dict(zip(names, positions))
        return self._execute(ARM_GROUP, self._joint_constraints(positions), plan_only)

    def move_to_pose(self, xyz, quat=None, rpy=None, frame=BASE_FRAME, pos_tol=0.005, rot_tol=0.02,
                     plan_only=False):
        """Move gripper_tcp to xyz [m] with orientation quat [x,y,z,w] or rpy [rad] in `frame`."""
        q = quat if quat is not None else rpy_to_quat(*(rpy if rpy is not None else (math.pi, 0.0, 0.0)))
        target = PoseStamped()
        target.header.frame_id = frame
        target.pose = Pose()
        target.pose.position.x, target.pose.position.y, target.pose.position.z = map(float, xyz)
        (target.pose.orientation.x, target.pose.orientation.y,
         target.pose.orientation.z, target.pose.orientation.w) = map(float, q)

        c = Constraints()
        region = BoundingVolume()
        region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[pos_tol]))
        region.primitive_poses.append(target.pose)
        c.position_constraints.append(PositionConstraint(
            header=target.header, link_name=TCP_LINK, constraint_region=region, weight=1.0))
        c.orientation_constraints.append(OrientationConstraint(
            header=target.header, link_name=TCP_LINK, orientation=target.pose.orientation,
            absolute_x_axis_tolerance=rot_tol, absolute_y_axis_tolerance=rot_tol,
            absolute_z_axis_tolerance=rot_tol, weight=1.0))
        return self._execute(ARM_GROUP, c, plan_only)

    # ── gripper ─────────────────────────────────────────────────────────────
    def set_gripper(self, position, max_effort=50.0, timeout=10.0):
        """position: 0.0 (open) ... 0.7929 (closed), knuckle angle [rad]."""
        self._check_safe_to_move()
        if not self._grip.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"gripper action {self._gripper_action} not available "
                               "(real robot: launch with gripper_hw:=mock or robotiq_driver)")
        goal = GripperCommand.Goal()
        goal.command.position = float(min(max(position, GRIPPER_OPEN), GRIPPER_CLOSED))
        goal.command.max_effort = float(max_effort)
        handle = self._wait(self._grip.send_goal_async(goal), 5.0)
        if not handle.accepted:
            raise RuntimeError("gripper goal rejected")
        r = self._wait(handle.get_result_async(), timeout).result
        self.get_logger().info(f"gripper -> {r.position:.3f} rad  reached={r.reached_goal} stalled={r.stalled}")
        return r.reached_goal or r.stalled  # stalled = holding an object

    def open_gripper(self):
        return self.set_gripper(GRIPPER_OPEN)

    def close_gripper(self, max_effort=50.0):
        return self.set_gripper(GRIPPER_CLOSED, max_effort)


def main(argv=None):
    argv = rclpy.utilities.remove_ros_args(sys.argv if argv is None else argv)[1:]
    p = argparse.ArgumentParser(prog="manipulator", description="Steve arm + gripper commands (MoveIt 2)")
    p.add_argument("--vel", type=float, default=0.2, help="velocity scaling 0..1")
    p.add_argument("--acc", type=float, default=0.2, help="acceleration scaling 0..1")
    p.add_argument("--plan_only", action="store_true", help="plan but do not execute")
    p.add_argument("--direct", action="store_true",
                   help="named / joints: send the goal straight to the trajectory controller, without "
                        "MoveIt (no collision checking — joint-space straight line)")
    p.add_argument("--duration", type=float, default=None,
                   help="--direct: motion time [s] (default: from the largest joint change and --vel)")
    p.add_argument("--safety", choices=["auto", "em_stop", "lidar", "off"], default="auto",
                   help="safety stop source: the robot's EM-stop system (auto if running; em_stop = "
                        "required, real robot) or the LiDAR fallback (Gazebo)")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list", help="list named states")
    s = sub.add_parser("named", help="move to a named state from the SRDF")
    s.add_argument("name")
    s = sub.add_parser("joints", help="move to 6 joint angles [rad]")
    s.add_argument("q", type=float, nargs=6)
    s = sub.add_parser("pose", help="move gripper_tcp to x y z")
    s.add_argument("xyz", type=float, nargs=3)
    g = s.add_mutually_exclusive_group()
    g.add_argument("--quat", type=float, nargs=4, metavar=("QX", "QY", "QZ", "QW"))
    g.add_argument("--rpy", type=float, nargs=3, metavar=("R", "P", "Y"),
                   help="default: 3.14159 0 0 (gripper pointing down)")
    s.add_argument("--frame", default=BASE_FRAME)
    s = sub.add_parser("gripper", help="open | close | knuckle angle 0..0.7929")
    s.add_argument("command")
    args = p.parse_args(argv)

    rclpy.init()
    m = SteveManipulator(velocity_scaling=args.vel, acceleration_scaling=args.acc, safety=args.safety)
    try:
        if args.cmd == "list":
            for group, states in m.named.items():
                print(f"{group}: {', '.join(sorted(states))}")
            ok = True
        elif args.cmd == "named" and args.direct:
            if args.name not in m.named.get(ARM_GROUP, {}):
                raise KeyError(f"unknown arm state '{args.name}' — known: {sorted(m.named.get(ARM_GROUP, {}))}")
            ok = m.move_direct(m.named[ARM_GROUP][args.name], args.duration)
        elif args.cmd == "joints" and args.direct:
            ok = m.move_direct(args.q, args.duration)
        elif args.cmd == "named":
            ok = m.move_to_named(args.name, args.plan_only)
        elif args.cmd == "joints":
            ok = m.move_to_joints(args.q, args.plan_only)
        elif args.cmd == "pose":
            ok = m.move_to_pose(args.xyz, quat=args.quat, rpy=args.rpy, frame=args.frame,
                                plan_only=args.plan_only)
        else:
            c = args.command.lower()
            ok = (m.open_gripper() if c == "open" else m.close_gripper() if c == "close"
                  else m.set_gripper(float(c)))
    except (RuntimeError, KeyError, ValueError, TimeoutError) as e:
        m.get_logger().error(str(e))
        ok = False
    finally:
        m.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
