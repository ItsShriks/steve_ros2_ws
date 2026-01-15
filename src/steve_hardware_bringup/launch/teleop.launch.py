#!/usr/bin/env python3
"""
Teleop Launch File
Launches joystick node and neo_teleop2 for manual robot control
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('steve_hardware_bringup')
    
    # Launch configurations
    robot_namespace = LaunchConfiguration('namespace', default='')
    
    # Configuration file
    teleop_config = os.path.join(pkg_share, 'config', 'teleop.yaml')
    
    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace for teleop'
    )
    
    # Neo teleop node
    neo_teleop_node = Node(
        package='neo_teleop2',
        executable='neo_teleop2_node',
        name='steve_teleop_node',
        namespace=robot_namespace,
        output='screen',
        parameters=[teleop_config]
    )
    
    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='steve_joy_node',
        namespace=robot_namespace,
        output='screen',
        parameters=[
            {'dev': '/dev/input/js0'},
            {'deadzone': 0.12}
        ]
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(neo_teleop_node)
    ld.add_action(joy_node)
    
    return ld
