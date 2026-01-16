import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    """
    Main hardware bringup for Steve robot (MMO-700 platform)
    
    Launches all hardware components:
    1. Robot state publisher (URDF)
    2. LiDAR sensors (2x SICK S300)
    3. UR5e robotic arm
    4. Base mobility driver (placeholder)
    5. Pan-tilt controller
    6. RealSense camera
    """
    
    hardware_pkg = get_package_share_directory('steve_hardware_bringup')
    
    # --- 1. ROBOT STATE PUBLISHER ---
    # Process the URDF with xacro (use main URDF from neo_simulation2 directly)
    neo_sim_pkg = get_package_share_directory('neo_simulation2')
    urdf_file = os.path.join(neo_sim_pkg, 'robots', 'mmo_700', 'mmo_700.urdf.xacro')
    robot_description = xacro.process_file(
        urdf_file,
        mappings={
            'use_gazebo': 'false',
            'arm_type': 'ur5e',
            'include_wrist_camera': 'true',
            'include_depth_camera': 'false',
            'include_pan_tilt': 'true'
        }
    ).toxml()
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': robot_description
        }]
    )
    
    # --- 2. LIDAR SENSORS ---
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hardware_pkg, 'launch', 'lidar_bringup.launch.py')
        )
    )
    
    # --- 3. UR5e ARM ---
    # Delayed to allow robot state publisher to initialize
    ur5e_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(hardware_pkg, 'launch', 'ur5e_bringup.launch.py')
                )
            )
        ]
    )
    
    # --- 4. BASE DRIVER ---
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hardware_pkg, 'launch', 'base_bringup.launch.py')
        )
    )
    
    # --- 5. PAN-TILT CONTROLLER ---
    # Will be added after converting from ROS1
    # pan_tilt_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('steve_pan_tilt_controller'), 
    #                      'launch', 'pan_tilt_bringup.launch.py')
    #     )
    # )
    
    # --- 6. REALSENSE CAMERA ---
    # Delayed to allow USB and TF tree to stabilize
    realsense_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('realsense2_camera'), 
                                 'launch', 'rs_launch.py')
                ),
                launch_arguments={
                    'enable_pointcloud': 'true',
                    'align_depth.enable': 'true',
                    'initial_reset': 'false',
                    'enable_sync': 'true',
                    'reconnect_timeout': '6.0'
                }.items()
            )
        ]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        base_launch,
        lidar_launch,
        ur5e_launch,
        # pan_tilt_launch,  # Uncomment after conversion
        realsense_launch
    ])
