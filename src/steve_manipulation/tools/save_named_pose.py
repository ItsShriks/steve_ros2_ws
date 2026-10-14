#!/usr/bin/env python3
"""
Save the arm's CURRENT joint positions (live /joint_states of the real robot) as a named state
in config/steve.srdf, e.g. a parking pose:

    python3 tools/save_named_pose.py parked            # read, show, write (replaces an existing 'parked')
    python3 tools/save_named_pose.py parked --dry_run  # read and show only

Then `manipulator.move_to_named("parked")` / MoveIt use it (with --symlink-install no rebuild; on
the robot PC pull the changed SRDF). Needs the UR driver running (arm joints in /joint_states).
Values are in the ROS / URDF convention of the SRDF (shoulder_pan -pi/2 = arm facing base +x).
"""

import argparse
import re
import sys
import time
from pathlib import Path

SRDF = Path(__file__).resolve().parents[1] / "config" / "steve.srdf"
ARM = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")


def read_arm(timeout):
    import rclpy
    from sensor_msgs.msg import JointState
    rclpy.init()
    n = rclpy.create_node("save_named_pose")
    got = {}

    def cb(m):
        names = [next((x for x in m.name if x.endswith(j)), None) for j in ARM]
        if None not in names:
            pos = dict(zip(m.name, m.position))
            got["q"] = [(x, pos[x]) for x in names]
    n.create_subscription(JointState, "/joint_states", cb, 10)
    t0 = time.monotonic()
    while "q" not in got and time.monotonic() - t0 < timeout:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_node()
    rclpy.try_shutdown()
    if "q" not in got:
        sys.exit(f"[ERROR] no UR5e joints in /joint_states within {timeout:.0f} s — is the UR driver running "
                 "(arm powered on, External Control playing)? ROS_DOMAIN_ID / DDS profile of the robot set?")
    return got["q"]


def write_state(srdf, name, group, joints):
    text = srdf.read_text()
    body = "".join(f'        <joint name="{j}" value="{v:.4f}"/>\n' for j, v in joints)
    block = f'    <group_state name="{name}" group="{group}">\n{body}    </group_state>\n'
    old = re.compile(rf'    <group_state name="{re.escape(name)}" group="{re.escape(group)}">\n.*?</group_state>\n', re.S)
    if old.search(text):
        return srdf.write_text(old.sub(block, text, count=1)), "replaced"
    last = list(re.finditer(rf'<group_state name="[^"]+" group="{re.escape(group)}">.*?</group_state>\n', text, re.S))
    if not last:
        sys.exit(f"[ERROR] no group_state of group '{group}' in {srdf}")
    i = last[-1].end()
    return srdf.write_text(text[:i] + block + text[i:]), "added"


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("name", help="named state, e.g. parked")
    p.add_argument("--group", default="ur_manipulator")
    p.add_argument("--srdf", type=Path, default=SRDF)
    p.add_argument("--timeout", type=float, default=10.0)
    p.add_argument("--dry_run", action="store_true", help="only read and print")
    a = p.parse_args()
    q = read_arm(a.timeout)
    for j, v in q:
        print(f"  {j:28s} {v:+.4f} rad  ({v * 57.29578:+8.2f} deg)")
    if a.dry_run:
        return 0
    _, how = write_state(a.srdf, a.name, a.group, q)
    print(f"'{a.name}' ({a.group}) {how} in {a.srdf}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
