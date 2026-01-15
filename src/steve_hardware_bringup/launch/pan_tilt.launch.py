#!/usr/bin/env python3
"""
Pan-Tilt Launch File
Launches pan-tilt Dynamixel motors and RealSense L515 camera
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch configurations
    robot_namespace = LaunchConfiguration('namespace', default='')
    enable_camera = LaunchConfiguration('enable_camera', default='true')
    
    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace for pan-tilt unit'
    )
    
    declare_camera_cmd = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable RealSense L515 camera'
    )
    
    # Pan-tilt controller
    pan_tilt_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('steve_pan_tilt_controller'),
                'launch',
                'steve_pan_tilt_controller.launch.py'
            )
        )
    )
    
    # RealSense L515 camera
    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'camera_name': 'pan_tilt_camera',
            'device_type': 'l515',
            'enable_color': 'true',
            'enable_depth': 'true',
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true'
        }.items(),
        condition=IfCondition(enable_camera)
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_camera_cmd)
    ld.add_action(pan_tilt_controller)
    ld.add_action(realsense_camera)
    
    return ld
