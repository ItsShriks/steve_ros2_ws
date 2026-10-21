#!/bin/bash
# Run the gripper RealSense (D415), plugged into this Mac, via pixi / RoboStack.
# Usage (from the workspace root):  pixi run gripper-camera [extra launch args, e.g. profile:=640x480x15]
#
# Starts steve_wbc's gripper_camera.launch.py (driver + calibrated mount TF ur5etool0 ->
# gripper_camera_color_optical_frame). macOS only lets librealsense claim the camera's USB
# interface as root, so this script re-executes itself under sudo while keeping the pixi
# environment (including the UDP-only DDS profile from scripts/mac_dds_env.sh).

if [ "$(id -u)" -ne 0 ]; then
    exec sudo -E env "PATH=$PATH" "HOME=$HOME" "$0" "$@"
fi

# Keep root-owned log files out of ~/.ros
export ROS_LOG_DIR="${TMPDIR:-/tmp}/gripper_camera_ros_logs"

exec ros2 launch steve_wbc gripper_camera.launch.py "$@"
