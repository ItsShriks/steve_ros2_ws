#!/usr/bin/env python3
"""
Arm + gripper control for Steve (UR5e + Robotiq 2F-85 on the Neobotix MMO-700) with MoveIt 2.
The mobile base is not commanded — MoveIt plans in base_link.

    use_sim_time:=true   Gazebo (steve_simulation, with gripper) + MoveIt
    use_sim_time:=false  physical robot (UR driver [+ gripper driver]) + MoveIt

Usage:
    ros2 launch steve_manipulation steve_manipulation.launch.py use_sim_time:=true
    ros2 launch steve_manipulation steve_manipulation.launch.py use_sim_time:=false robot_ip:=192.168.1.102
    ros2 launch steve_manipulation steve_manipulation.launch.py use_sim_time:=false gripper_hw:=mock
    ros2 launch steve_manipulation steve_manipulation.launch.py use_sim_time:=false launch_bringup:=false

Then command it, e.g.:
    ros2 run steve_manipulation manipulator named ready
    ros2 run steve_manipulation manipulator gripper close

Real-robot bringup (launch_bringup:=true):
    real_bringup:=arm   UR5e driver + robot_state_publisher only (default — base, lidars, teleop off)
    real_bringup:=full  complete steve_hardware_bringup (base, lidars, pan-tilt, teleop, UR5e)
Real-robot gripper (gripper_hw):
    none            no gripper control, arm only (default: gripper interface not decided yet)
    mock            simulated gripper controller (MoveIt + API work, nothing moves)
    robotiq_driver  Robotiq RS-485 via the UR tool port (/tmp/ttyUR) — needs ros2_robotiq_gripper
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import xacro

ARM_TYPE = "ur5e"
GRIPPER_HW = ("none", "mock", "robotiq_driver")


def launch_setup(context: LaunchContext):
    cfg = lambda name: LaunchConfiguration(name).perform(context)
    is_sim = cfg("use_sim_time").lower() == "true"
    launch_bringup = cfg("launch_bringup").lower() == "true"
    gripper_hw = "gazebo" if is_sim else cfg("gripper_hw")
    if not is_sim and gripper_hw not in GRIPPER_HW:
        raise RuntimeError(f"gripper_hw must be one of {GRIPPER_HW}, got '{gripper_hw}'")
    print(f"[INFO] steve_manipulation: {'Gazebo simulation' if is_sim else 'physical robot'}"
          f" | gripper: {gripper_hw}")

    pkg = get_package_share_directory("steve_manipulation")
    sim_pkg = get_package_share_directory("steve_simulation")
    robot_xacro = os.path.join(sim_pkg, "robots", "mmo_700", "mmo_700.urdf.xacro")
    mappings = {
        "use_gazebo": "true" if is_sim else "false",
        "arm_type": ARM_TYPE,
        "arm_tool": "robotiq_2f_85",
        "gripper_hw": "gazebo" if is_sim else "none",  # real gripper runs its own controller manager
        "include_pan_tilt": "true",
        "include_wrist_camera": "true",
    }
    actions = []

    # --- 1. ROBOT BRINGUP -------------------------------------------------------
    if launch_bringup and is_sim:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim_pkg, "launch", "simulation.launch.py")),
            launch_arguments={
                "my_robot": "mmo_700",
                "world": cfg("world"),
                "use_sim_time": "true",
                "arm_type": ARM_TYPE,
                "arm_tool": "robotiq_2f_85",
                "include_pan_tilt": "true",
                "use_rviz": "false",
                "launch_map_server": "false",
            }.items(),
        ))
    elif launch_bringup and cfg("real_bringup") == "full":
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("steve_hardware_bringup"), "launch", "hardware_bringup.launch.py")),
            launch_arguments={
                "arm_type": ARM_TYPE,
                "robot_ip": cfg("robot_ip"),
                "enable_camera": cfg("enable_camera"),
                "enable_pan_tilt": "true",
                "arm_tool": "robotiq_2f_85",
            }.items(),
        ))
    elif launch_bringup:  # real_bringup:=arm — UR driver + TF only
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("steve_hardware_bringup"), "launch", "ur5e_arm.launch.py")),
            launch_arguments={
                "ur_type": ARM_TYPE,
                "robot_ip": cfg("robot_ip"),
                "tf_prefix": ARM_TYPE,
                "use_tool_communication": "true" if gripper_hw == "robotiq_driver" else "false",
            }.items(),
        ))
        actions.append(Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="steve_robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": xacro.process_file(robot_xacro, mappings=mappings).toxml()}],
        ))

    # --- 2. REAL GRIPPER: stand-alone controller manager -----------------------
    if not is_sim and gripper_hw != "none":
        gripper_description = xacro.process_file(
            os.path.join(pkg, "urdf", "gripper_only.urdf.xacro"),
            mappings={"gripper_hw": gripper_hw, "com_port": cfg("gripper_com_port")}).toxml()
        controllers = os.path.join(pkg, "config", "gripper_controllers.yaml")
        actions.append(Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="gripper_controller_manager",
            output="screen",
            parameters=[{"robot_description": gripper_description}, controllers],
        ))
        spawn = ["gripper_joint_state_broadcaster", "gripper_controller"]
        if gripper_hw == "robotiq_driver":
            spawn.append("robotiq_activation_controller")
        actions.append(TimerAction(period=2.0, actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=[*spawn, "-c", "/gripper_controller_manager"],
            output="screen",
        )]))

    # --- 3. MOVEIT ----------------------------------------------------------------
    moveit_config = (
        MoveItConfigsBuilder("mmo_700", package_name="steve_manipulation")
        .robot_description(file_path=robot_xacro, mappings=mappings)
        .robot_description_semantic(file_path="config/steve.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path=f"config/moveit_controllers_{'sim' if is_sim else 'real'}.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .to_moveit_configs()
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": is_sim,
                "publish_robot_description_semantic": True,
                # Gazebo / the UR driver own execution speed; allow a bit of controller lag
                "trajectory_execution.allowed_execution_duration_scaling": 2.0,
                "trajectory_execution.allowed_start_tolerance": 0.05,
            },
        ],
    )
    # In Gazebo the controllers come up a few seconds after the simulator
    actions.append(TimerAction(period=float(cfg("moveit_delay")), actions=[move_group]))

    if cfg("use_rviz").lower() == "true":
        actions.append(TimerAction(period=float(cfg("moveit_delay")), actions=[Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_moveit",
            output="log",
            arguments=["-d", os.path.join(pkg, "rviz", "moveit.rviz")],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
                {"use_sim_time": is_sim},
            ],
        )]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        # --- environment (same convention as steve_navigation / steve_wbc) ---
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="true: Gazebo simulation, false: physical robot"),
        DeclareLaunchArgument("launch_bringup", default_value="true",
                              description="Also start Gazebo / the robot drivers "
                                          "(false if they are already running)"),
        DeclareLaunchArgument("real_bringup", default_value="arm",
                              description="Physical robot: arm (UR driver + TF only) or full "
                                          "(complete steve_hardware_bringup)"),
        DeclareLaunchArgument("world", default_value="neo_workshop",
                              description="Gazebo world (simulation only)"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.102",
                              description="UR5e controller IP (physical robot)"),
        DeclareLaunchArgument("enable_camera", default_value="false",
                              description="Physical robot, real_bringup:=full: start the L515"),
        # --- gripper ---
        DeclareLaunchArgument("gripper_hw", default_value="none",
                              description="Physical robot gripper: none, mock or robotiq_driver "
                                          "(Gazebo always simulates it)"),
        DeclareLaunchArgument("gripper_com_port", default_value="/tmp/ttyUR",
                              description="Robotiq RS-485 port (UR tool communication)"),
        # --- MoveIt / RViz ---
        DeclareLaunchArgument("use_rviz", default_value="true", description="RViz with MotionPlanning"),
        DeclareLaunchArgument("moveit_delay", default_value="8.0",
                              description="Seconds before move_group starts (controllers must be up)"),
        OpaqueFunction(function=launch_setup),
    ])
