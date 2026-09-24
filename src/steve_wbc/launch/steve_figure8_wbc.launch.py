#!/usr/bin/env python3
"""
Figure-8 whole-body controller for Steve — simulation or physical robot.

    use_sim_time:=true  simulator:=gazebo  -> Gazebo (steve_simulation) + WBC over ROS 2
    use_sim_time:=true  simulator:=mujoco  -> standalone MuJoCo physics, no ROS robot needed
    use_sim_time:=false                     -> real robot (steve_hardware_bringup) + WBC

Usage:
    ros2 launch steve_wbc steve_figure8_wbc.launch.py use_sim_time:=true simulator:=mujoco
    ros2 launch steve_wbc steve_figure8_wbc.launch.py use_sim_time:=true world:=neo_workshop
    ros2 launch steve_wbc steve_figure8_wbc.launch.py use_sim_time:=false launch_bringup:=false dry_run:=true
    ros2 launch steve_wbc steve_figure8_wbc.launch.py use_sim_time:=false launch_bringup:=false \\
        amplitude:=0.08 frequency:=0.25 loops:=1

Any other controller option (weights, limits, ...) can be passed through
extra_args, e.g. extra_args:="--max_base_v 0.1 --kp_pos 3.0".
See `ros2 run steve_wbc steve_figure8_wbc --help`.
"""

import os
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Arm trajectory controller that is active by default in each environment
ARM_TRAJ_TOPIC = {
    "gazebo": "/joint_trajectory_controller/joint_trajectory",
    "real": "/scaled_joint_trajectory_controller/joint_trajectory",
}
# Gazebo needs time to start, spawn the robot and activate controllers
WAIT_TIMEOUT = {"gazebo": "120", "real": "15"}


def launch_setup(context: LaunchContext):
    cfg = lambda name: LaunchConfiguration(name).perform(context)
    is_sim = cfg("use_sim_time").lower() == "true"
    simulator = cfg("simulator").lower()
    launch_bringup = cfg("launch_bringup").lower() == "true"
    if simulator not in ("gazebo", "mujoco"):
        raise RuntimeError(f"simulator must be 'gazebo' or 'mujoco', got '{simulator}'")

    env = ("mujoco" if simulator == "mujoco" else "gazebo") if is_sim else "real"
    print(f"[INFO] steve_wbc: environment = {env}")
    actions = []

    # --- 1. ROBOT BRINGUP -------------------------------------------------------
    if launch_bringup and env == "gazebo":
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("steve_simulation"), "launch", "simulation.launch.py")),
            launch_arguments={
                "my_robot": "mmo_700",
                "world": cfg("world"),
                "use_sim_time": "true",
                "arm_type": cfg("arm_type"),
                "include_pan_tilt": "true",
                "use_rviz": cfg("use_rviz"),
                "launch_map_server": "false",  # no navigation needed for the WBC
            }.items(),
        ))
    elif launch_bringup and env == "real":
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("steve_navigation"), "launch", "robot_bringup.launch.py")),
            launch_arguments={
                "arm_type": cfg("arm_type"),
                "enable_camera": cfg("enable_camera"),
                "enable_joystick": cfg("enable_joystick"),
            }.items(),
        ))

    # --- 2. WHOLE-BODY CONTROLLER ----------------------------------------------
    args = [
        "--amplitude", cfg("amplitude"),
        "--frequency", cfg("frequency"),
        "--loops", cfg("loops"),
        "--plane", cfg("plane"),
        "--orientation", cfg("orientation"),
    ]
    if cfg("cz"):
        args += ["--cz", cfg("cz")]
    if cfg("log"):
        args += ["--log", cfg("log")]

    if env == "mujoco":
        args += ["--mode", "sim"]
        if cfg("headless").lower() == "true":
            args.append("--headless")
    else:
        args += [
            "--mode", "ros",
            "--arm_interface", cfg("arm_interface"),
            "--arm_traj_topic", cfg("arm_traj_topic") or ARM_TRAJ_TOPIC[env],
            "--wait_timeout", WAIT_TIMEOUT[env],
        ]
        if cfg("dry_run").lower() == "true":
            args.append("--dry_run")
    args += shlex.split(cfg("extra_args"))

    actions.append(Node(
        package="steve_wbc",
        executable="steve_figure8_wbc",
        name="steve_figure8_wbc",
        output="screen",
        emulate_tty=True,
        arguments=args,
        parameters=[{"use_sim_time": is_sim and env == "gazebo"}],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        # --- environment (same convention as steve_navigation) ---
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="true: simulation, false: physical robot"),
        DeclareLaunchArgument("simulator", default_value="gazebo",
                              description="Simulator when use_sim_time:=true: gazebo or mujoco"),
        DeclareLaunchArgument("launch_bringup", default_value="true",
                              description="Also start Gazebo / the robot hardware bringup "
                                          "(false if it is already running)"),
        DeclareLaunchArgument("world", default_value="neo_workshop",
                              description="Gazebo world (simulation only)"),
        DeclareLaunchArgument("use_rviz", default_value="false",
                              description="RViz with the Gazebo simulation"),
        DeclareLaunchArgument("arm_type", default_value="ur5e", description="UR arm type"),
        DeclareLaunchArgument("enable_camera", default_value="false",
                              description="Real robot: start the L515 camera"),
        DeclareLaunchArgument("enable_joystick", default_value="true",
                              description="Real robot: start the joystick (dead-man / manual override)"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="MuJoCo: run without the viewer"),
        # --- controller / safety ---
        DeclareLaunchArgument("dry_run", default_value="false",
                              description="Read the robot state but publish no commands"),
        DeclareLaunchArgument("arm_interface", default_value="trajectory",
                              description="trajectory or velocity (forward_velocity_controller)"),
        DeclareLaunchArgument("arm_traj_topic", default_value="",
                              description="Override the arm JointTrajectory topic "
                                          "(default: per environment)"),
        # --- figure-8 ---
        DeclareLaunchArgument("amplitude", default_value="0.15", description="[m]"),
        DeclareLaunchArgument("frequency", default_value="0.45", description="[rad/s]"),
        DeclareLaunchArgument("loops", default_value="3"),
        DeclareLaunchArgument("plane", default_value="vertical", description="vertical or horizontal"),
        DeclareLaunchArgument("orientation", default_value="hold", description="hold or forward"),
        DeclareLaunchArgument("cz", default_value="", description="figure-8 centre height [m]"),
        DeclareLaunchArgument("log", default_value="", description="CSV log path"),
        DeclareLaunchArgument("extra_args", default_value="",
                              description="Additional steve_figure8_wbc CLI arguments"),
        OpaqueFunction(function=launch_setup),
    ])
