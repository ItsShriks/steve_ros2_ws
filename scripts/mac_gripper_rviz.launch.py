"""RViz on the Mac: the robot (model + TF from the robot over DDS) plus the gripper camera
(started separately with `pixi run gripper-camera`, needs sudo).

The camera runs without its own TF (only the calibrated mount ur5etool0 ->
gripper_camera_color_optical_frame), so the coloured point cloud is built here from the depth
image aligned to colour, which lives in that frame.

    pixi run gripper-rviz
"""

from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node

CAM = "/gripper_camera/gripper_camera"   # realsense2_camera >= 4.55: <namespace>/<name>


def generate_launch_description():
    rviz_config = Path(__file__).resolve().with_name("mac_gripper_camera.rviz")
    return LaunchDescription([
        Node(package="depth_image_proc", executable="point_cloud_xyzrgb_node", name="gripper_camera_points",
             remappings=[("rgb/camera_info", f"{CAM}/color/camera_info"),
                         ("rgb/image_rect_color", f"{CAM}/color/image_raw"),
                         ("depth_registered/image_rect", f"{CAM}/aligned_depth_to_color/image_raw"),
                         ("points", f"{CAM}/points")]),
        Node(package="rviz2", executable="rviz2", name="rviz2", arguments=["-d", str(rviz_config)]),
    ])
