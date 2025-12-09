# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription, LaunchContext
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.event_handlers import OnShutdown
import os
import xacro

"""
Description:

This launch file is used to start a ROS2 simulation for a Neobotix robot in a specified environment. 
It sets up the Gazebo simulator with the chosen robot and environment, 
optionally starts the robot state publisher, and enables keyboard teleoperation.

You can launch this file using the following terminal commands:

1. `ros2 launch neo_simulation2 simulation.launch.py --show-args`
   This command shows the arguments that can be passed to the launch file.
2. `ros2 launch neo_simulation2 simulation.launch.py my_robot:=mpo_500 world:=neo_track1 arm_type:=ur5e`
   This command launches the simulation with sample values for the arguments.
   !(only mpo_700 and mpo_500 support arms)
"""

# OpaqueFunction is used to perform setup actions during launch through a Python function
def launch_setup(context: LaunchContext, my_neo_robot_arg, my_neo_env_arg, robot_arm_arg, docking_adapter_arg, 
                 include_wrist_camera_arg, include_depth_camera_arg, include_pan_tilt_arg):
    # Create a list to hold all the nodes
    launch_actions = []
    # The perform method of a LaunchConfiguration is called to evaluate its value.
    my_neo_robot = my_neo_robot_arg.perform(context)
    my_neo_environment = my_neo_env_arg.perform(context)
    robot_arm_type = robot_arm_arg.perform(context)
    use_docking_adapter = docking_adapter_arg.perform(context)
    include_wrist_camera = include_wrist_camera_arg.perform(context)
    include_depth_camera = include_depth_camera_arg.perform(context)

    include_pan_tilt = include_pan_tilt_arg.perform(context)
    use_sim_time = True
    
    print("\n" + "="*70)
    print("  Neobotix ROS2 Simulation Launch")
    print("="*70)
    print(f"[INFO] Requested Robot: {my_neo_robot}")
    print(f"[INFO] Requested World: {my_neo_environment}")
    print(f"[INFO] Requested Arm Type: {robot_arm_type if robot_arm_type else 'None'}")
    print(f"[INFO] Docking Adapter: {use_docking_adapter}")
    print("="*70 + "\n")

    robots = ["mpo_700", "mp_400", "mp_500", "mpo_500", "mmo_700"]

    # Checking if the user has selected a robot that is valid
    if my_neo_robot not in robots:
        # Incase of an invalid selection
        print(f"[WARNING] Invalid robot '{my_neo_robot}' selected!")
        print(f"[WARNING] Available robots: {', '.join(robots)}")
        print("[WARNING] Defaulting to 'mpo_700'")
        my_neo_robot = "mpo_700"
    else:
        print(f"[INFO] Robot validation successful: {my_neo_robot}")

    with open('robot_name.txt', 'w') as file:
        file.write(my_neo_robot)

    # Remove arm_type if robot does not support it
    if (robot_arm_type != ''):
        if (my_neo_robot != "mpo_700" and my_neo_robot != "mpo_500" and my_neo_robot != "mmo_700"):
            print(f"[WARNING] Robot '{my_neo_robot}' does not support arm integration")
            print("[WARNING] Arm support is only available for: mpo_700, mpo_500, mmo_700")
            print("[WARNING] Disabling arm_type")
            robot_arm_type = ''
        else:
            print(f"[INFO] Arm integration enabled: {robot_arm_type}")

    # Get the required paths for the world and robot robot_description_urdf
    if (my_neo_environment == "neo_workshop" or my_neo_environment == "neo_track1"):
        world_path = os.path.join(
            get_package_share_directory('neo_simulation2'),
            'worlds',
            my_neo_environment + '.world')
        print(f"[INFO] Using built-in world: {my_neo_environment}")
    else:
        world_path = my_neo_environment
        print(f"[INFO] Using custom world file: {world_path}")
    print(f"[INFO] World path: {world_path}")

    # Setting the world and starting the Gazebo
    # Adding /opt/ros/humble/share to GAZEBO_MODEL_PATH to fix model:// URI delays
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += os.pathsep + '/opt/ros/humble/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  '/opt/ros/humble/share'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'true',
        }.items()
    )

    # Getting the robot description xacro
    robot_description_xacro = os.path.join(
        get_package_share_directory('neo_simulation2'),
        'robots/'+my_neo_robot+'/',
        my_neo_robot+'.urdf.xacro')
    print(f"[INFO] Robot URDF path: {robot_description_xacro}")
    
    # Docking adapter is only for MPO 700
    if (my_neo_robot != "mpo_700"):
        if use_docking_adapter == 'True':
            print(f"[WARNING] Docking adapter only supported for mpo_700")
            print("[WARNING] Disabling docking adapter")
        use_docking_adapter = False
        
     # use_gazebo is set to True since this code launches the robot in simulation
    xacro_args = {
        'use_gazebo': 'true',
        'arm_type': robot_arm_type,
        'use_docking_adapter': use_docking_adapter,
        'include_wrist_camera': include_wrist_camera,
        'include_depth_camera': include_depth_camera,
        'include_pan_tilt': include_pan_tilt
    }
    print("[INFO] Processing URDF with xacro...")
    print(f"[INFO] Xacro arguments: {xacro_args}")

    # Use xacro to process the file with the argunments above
    robot_description_file = xacro.process_file(
        robot_description_xacro, 
        mappings=xacro_args
        ).toxml()
    print("[INFO] URDF processing complete")

    # Spawning the robot
    # Spawning the robot
    # Using /usr/bin/python3 explicitly to avoid Anaconda conflicts
    spawn_entity = Node(
        package=None,
        executable='/usr/bin/python3',
        arguments=[
            os.path.join(get_package_prefix('gazebo_ros'), 'lib', 'gazebo_ros', 'spawn_entity.py'),
            '-entity', my_neo_robot,
            '-topic', '/robot_description'
        ],
        output='screen'
    )

    # Start the robot state publisher node
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 
                    'robot_description': robot_description_file}]
    )



    # Starting the teleop node
    teleop = Node(
        package='teleop_twist_keyboard',
        executable="teleop_twist_keyboard",
        output='screen',
        prefix = 'xterm -e',
        name='teleop'
    )

    # RViz for visualization and joint control
    rviz_config = os.path.join(
        get_package_share_directory('neo_simulation2'),
        'rviz',
        'robot_description_rviz.rviz'
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    pan_tilt_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pan_tilt_controller", "-c", "/controller_manager"],
    )



    # See Issue: https://github.com/ros2/rclpy/issues/1287
    # Cannot delete the newly create file. The user has to delete it on his own
    # Refer documentation for more info
    # shutdown_event = RegisterEventHandler(
    #         OnShutdown(
    #             on_shutdown=[os.remove('robot_name.txt')]
    #         )
    #     )

    # The required nodes can just be appended to the launch_actions list
    print("\n[INFO] Launching nodes...")
    print("[INFO] - Robot State Publisher")
    launch_actions.append(start_robot_state_publisher_cmd)
    if robot_arm_type != '':
        print("[INFO] - Joint State Broadcaster")
        print("[INFO] - Joint Trajectory Controller")
        launch_actions.append(joint_state_broadcaster_spawner)
        launch_actions.append(initial_joint_controller_spawner_stopped)
    
    if include_pan_tilt == 'true':
        print("[INFO] - Pan-Tilt Controller")
        launch_actions.append(pan_tilt_controller_spawner)



    print("[INFO] - Gazebo Simulator")
    launch_actions.append(gazebo)
    print(f"[INFO] - Spawn Entity ({my_neo_robot})")
    launch_actions.append(spawn_entity)



    print("[INFO] - RViz2 (for visualization and joint control)")
    launch_actions.append(rviz)
    print("[INFO] - Teleop Twist Keyboard")
    launch_actions.append(teleop)
    # launch_actions.append(shutdown_event)
    print("\n" + "="*70)
    print("  Launch Configuration Complete")
    print("="*70)
    print(f"  Robot: {my_neo_robot}")
    print(f"  World: {my_neo_environment}")
    print(f"  Arm: {robot_arm_type if robot_arm_type else 'Disabled'}")
    print(f"  Arm: {robot_arm_type if robot_arm_type else 'Disabled'}")
    print(f"  RViz: Enabled (joint control available)")
    print(f"  Nodes: {len(launch_actions)} total")
    print("="*70 + "\n")

    return launch_actions

def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments 'my_robot' and 'world' with default values and descriptions
    declare_my_robot_arg = DeclareLaunchArgument(
        'my_robot', 
        default_value='mpo_700',
        description='Robot Types: "mpo_700", "mpo_500", "mp_400", "mp_500"'
    ) 
    
    declare_world_name_arg = DeclareLaunchArgument(
        'world',
        default_value='neo_workshop',
        description='Available worlds: "neo_track1", "neo_workshop"'
    )

    declare_arm_type_cmd = DeclareLaunchArgument(
        'arm_type', default_value='',
        description='Arm Types:\n'
        '\t Elite Arms: ec66, cs66\n'
        '\t Universal Robotics: ur5, ur10, ur5e, ur10e'        
    )

    declare_docking_adapter_cmd = DeclareLaunchArgument(
        'use_docking_adapter', default_value='False',
        description='Set True to use the docking adapter for the robot\n'
        '\t Neobotix: docking_adapter'
    )

    declare_wrist_camera_cmd = DeclareLaunchArgument(
        'include_wrist_camera', default_value='true',
        description='Include wrist D405 camera on arm'
    )

    declare_depth_camera_cmd = DeclareLaunchArgument(
        'include_depth_camera', default_value='false',
        description='Include front depth camera'
    )

    declare_pan_tilt_cmd = DeclareLaunchArgument(
        'include_pan_tilt', default_value='false',
        description='Include pan-tilt camera tower'
    )



    # Create launch configuration variables for the robot and map name
    my_neo_robot_arg = LaunchConfiguration('my_robot')
    my_neo_env_arg = LaunchConfiguration('world')
    robot_arm_arg = LaunchConfiguration('arm_type')
    docking_adapter_arg = LaunchConfiguration('use_docking_adapter')
    include_wrist_camera_arg = LaunchConfiguration('include_wrist_camera')
    include_depth_camera_arg = LaunchConfiguration('include_depth_camera')

    include_pan_tilt_arg = LaunchConfiguration('include_pan_tilt')


    ld.add_action(declare_my_robot_arg)
    ld.add_action(declare_world_name_arg)
    ld.add_action(declare_arm_type_cmd)
    ld.add_action(declare_docking_adapter_cmd)
    ld.add_action(declare_wrist_camera_cmd)
    ld.add_action(declare_depth_camera_cmd)

    ld.add_action(declare_pan_tilt_cmd)


    context_arguments = [
        my_neo_robot_arg, 
        my_neo_env_arg, 
        robot_arm_arg, 
        docking_adapter_arg,
        include_wrist_camera_arg,
        include_depth_camera_arg,

        include_pan_tilt_arg
    ]

    opq_function = OpaqueFunction(
        function=launch_setup, 
        args=context_arguments
    )

    ld.add_action(opq_function)

    return ld

