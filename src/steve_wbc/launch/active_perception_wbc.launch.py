#!/usr/bin/env python3
"""
Active perception + MPC interception + grasping with the unified joint-velocity WBC.

    use_sim_time:=true   simulator:=mujoco  -> MuJoCo (default): cameras, LiDAR, VGN, gripper simulated
    use_sim_time:=true   simulator:=gazebo  -> Gazebo (steve_simulation, gripper) + WBC over ROS 2
    use_sim_time:=false                      -> physical robot + WBC over ROS 2

Over ROS 2 the object comes from `target_topic` (geometry_msgs/PointStamped in odom or
base_link, e.g. from steve_perception), the LiDAR from `scan_topic`, and the gripper is
commanded through steve_manipulation's /gripper_controller/gripper_cmd.

Usage:
    ros2 launch steve_wbc active_perception_wbc.launch.py
    ros2 launch steve_wbc active_perception_wbc.launch.py object_type:=box vgn:=false
    ros2 launch steve_wbc active_perception_wbc.launch.py use_sim_time:=false dry_run:=true
    ros2 launch steve_wbc active_perception_wbc.launch.py use_sim_time:=false launch_bringup:=false \\
        target_topic:=/steve_perception/target_point
"""

import os
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARM_TRAJ_TOPIC = {
    "gazebo": "/joint_trajectory_controller/joint_trajectory",
    "real": "/scaled_joint_trajectory_controller/joint_trajectory",
}
WAIT_TIMEOUT = {"gazebo": "120", "real": "15"}


def launch_setup(context: LaunchContext):
    cfg = lambda name: LaunchConfiguration(name).perform(context)
    is_sim = cfg("use_sim_time").lower() == "true"
    simulator = cfg("simulator").lower()
    if simulator not in ("gazebo", "mujoco"):
        raise RuntimeError(f"simulator must be 'gazebo' or 'mujoco', got '{simulator}'")
    env = simulator if is_sim else "real"
    print(f"[INFO] active_perception_wbc: environment = {env}")
    actions = []

    # --- 1. BRINGUP: robot with gripper + MoveIt-independent gripper controller ---
    if cfg("launch_bringup").lower() == "true" and env != "mujoco":
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("steve_manipulation"), "launch", "steve_manipulation.launch.py")),
            launch_arguments={
                "use_sim_time": "true" if env == "gazebo" else "false",
                "world": cfg("world"),
                "robot_ip": cfg("robot_ip"),
                "real_bringup": "full",          # base, LiDARs and arm are all needed
                "gripper_hw": cfg("gripper_hw"),
                "use_rviz": cfg("use_rviz"),
            }.items(),
        ))

    # --- 2. CONTROLLER ------------------------------------------------------------
    args = ["--object_type", cfg("object_type"), "--object_speed", cfg("object_speed")]
    if cfg("vgn").lower() != "true":
        args.append("--no_vgn")
    if cfg("log"):
        args += ["--log", cfg("log")]
    if env == "mujoco":
        args += ["--mode", "sim"]
        if cfg("headless").lower() == "true":
            args.append("--headless")
    else:
        args += [
            "--mode", "ros",
            "--arm_traj_topic", ARM_TRAJ_TOPIC[env],
            "--wait_timeout", WAIT_TIMEOUT[env],
            "--scan_topic", cfg("scan_topic"),
            "--target_topic", cfg("target_topic"),
            "--no_vgn",                          # VGN runs on the simulated depth camera only
        ]
        if cfg("dry_run").lower() == "true":
            args.append("--dry_run")
    args += shlex.split(cfg("extra_args"))

    actions.append(Node(
        package="steve_wbc",
        executable="active_perception_wbc",
        name="active_perception_wbc",
        output="screen",
        emulate_tty=True,
        arguments=args,
        parameters=[{"use_sim_time": env == "gazebo"}],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="true: simulation, false: physical robot"),
        DeclareLaunchArgument("simulator", default_value="mujoco",
                              description="Simulator when use_sim_time:=true: mujoco or gazebo"),
        DeclareLaunchArgument("launch_bringup", default_value="true",
                              description="Gazebo / robot: also start steve_manipulation's bringup"),
        DeclareLaunchArgument("world", default_value="neo_workshop", description="Gazebo world"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.102", description="UR5e IP"),
        DeclareLaunchArgument("gripper_hw", default_value="none",
                              description="Physical robot gripper: none, mock or robotiq_driver"),
        DeclareLaunchArgument("use_rviz", default_value="false", description="RViz (Gazebo / robot)"),
        DeclareLaunchArgument("headless", default_value="false", description="MuJoCo without viewer"),
        DeclareLaunchArgument("dry_run", default_value="false",
                              description="Read the robot state but publish no commands"),
        DeclareLaunchArgument("object_type", default_value="sphere", description="MuJoCo: sphere or box"),
        DeclareLaunchArgument("object_speed", default_value="0.02", description="MuJoCo: object speed [m/s]"),
        DeclareLaunchArgument("vgn", default_value="true",
                              description="MuJoCo: VGN grasp scan (needs the vgn_env conda env)"),
        DeclareLaunchArgument("scan_topic", default_value="/lidar_1/scan_filtered",
                              description="ROS: front LiDAR"),
        DeclareLaunchArgument("target_topic", default_value="/steve_perception/target_point",
                              description="ROS: object position (geometry_msgs/PointStamped)"),
        DeclareLaunchArgument("log", default_value="", description="CSV log path"),
        DeclareLaunchArgument("extra_args", default_value="",
                              description="Additional active_perception_wbc CLI arguments"),
        OpaqueFunction(function=launch_setup),
    ])
