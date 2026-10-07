"""
Detect an already running robot bringup on the ROS graph.

On the real robot ROS_AUTOSTART.sh starts steve_hardware_bringup (UR driver = /controller_manager,
steve_robot_state_publisher, ...) in a screen session on every login. Launch files that can also
start the drivers (steve_manipulation, steve_wbc) call probe_running_bringup() first so that they
never start a second UR driver / controller_manager.
"""

import time

PROBE_TIME = 8.0  # s  ROS graph discovery before deciding whether a bringup is already running
# processes of a bringup on THIS computer: found without ROS discovery, which can be slow on a busy
# network (a 3 s graph probe once missed the running bringup and a second UR driver was started)
LOCAL_PROCESSES = {"ur_ros2_control_node": "/controller_manager",
                   "robot_state_publisher": "/steve_robot_state_publisher"}


def local_bringup_processes():
    """ROS node names whose driver process runs on this computer (from /proc, no ROS needed)."""
    import os
    found = set()
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                cmd = f.read().split(b"\0")
        except OSError:
            continue
        exe = os.path.basename(cmd[0].decode(errors="ignore")) if cmd and cmd[0] else ""
        if exe in LOCAL_PROCESSES:
            found.add(LOCAL_PROCESSES[exe])
    return found
WATCHED = ("/controller_manager", "/steve_robot_state_publisher", "/gripper_controller_manager")


def probe_running_bringup(timeout=PROBE_TIME):
    """Nodes of an already running robot bringup (e.g. from ROS_AUTOSTART.sh) and whether its
    robot_description contains the gripper. Uses its own rclpy context, torn down afterwards."""
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from rclpy.executors import SingleThreadedExecutor
    local = local_bringup_processes()
    if "/controller_manager" in local:
        timeout = min(timeout, 2.0)  # the driver runs here: the graph probe only checks the gripper
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    node = rclpy.create_node("steve_manipulation_bringup_probe", context=ctx)
    found, gripper_in_description = set(), None
    try:
        watched = set(WATCHED)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            names = {(ns.rstrip("/") + "/" + n) for n, ns in node.get_node_names_and_namespaces()}
            found = watched & names
            if "/controller_manager" in found and "/steve_robot_state_publisher" in found:
                break
            time.sleep(0.2)
        if "/steve_robot_state_publisher" in found:
            cli = node.create_client(GetParameters, "/steve_robot_state_publisher/get_parameters")
            if cli.wait_for_service(timeout_sec=2.0):
                fut = cli.call_async(GetParameters.Request(names=["robot_description"]))
                exe = SingleThreadedExecutor(context=ctx)
                exe.add_node(node)
                exe.spin_until_future_complete(fut, timeout_sec=2.0)
                if fut.result() is not None and fut.result().values:
                    gripper_in_description = "robotiq_85" in fut.result().values[0].string_value
    finally:
        node.destroy_node()
        rclpy.shutdown(context=ctx)
    return found | local, gripper_in_description


def bringup_running(timeout=PROBE_TIME):
    """True if a robot driver (/controller_manager) is already running."""
    return "/controller_manager" in probe_running_bringup(timeout)[0]
