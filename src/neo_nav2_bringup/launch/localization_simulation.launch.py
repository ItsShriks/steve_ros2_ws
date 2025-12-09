import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Declare map argument
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(get_package_share_directory('neo_simulation2'), 'maps', 'neo_track1.yaml'),
        description='Full path to map yaml file to load'
    )

    # Launch Simulation
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('neo_simulation2'), 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'my_robot': 'mmo_700',
            'world': 'neo_workshop',
            'use_sim_time': 'true',
            'arm_type': 'ur5e'
        }.items()
    )

    # Launch Localization
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('neo_nav2_bringup'), 'launch', 'localization_amcl.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map': LaunchConfiguration('map'),
            'params_file': os.path.join(get_package_share_directory('neo_nav2_bringup'), 'config', 'localization.yaml')
        }.items()
    )

    ld.add_action(declare_map_arg)
    ld.add_action(simulation_launch)
    ld.add_action(localization_launch)

    return ld
