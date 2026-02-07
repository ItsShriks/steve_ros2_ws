# steve_ros2_ws

ROS2 Humble workspace for the Steve Butler robot project - an omnidirectional mobile manipulator with advanced perception and navigation capabilities.

## Contributors

- **Shrikar Nakhye** - [GitHub](https://github.com/Itsshriks) | [Email](mailto:nakhyeshrikar@icloud.com)
- **Pratik Adhikari** - [GitHub](https://github.com/pratik-adhikari) | [Email](mailto:pratikadhikari.de@gmail.com)
- **Riddhesh More** - [GitHub](https://github.com/RiddheshMore) | [Email](mailto:riddheshmore311@gmail.com)

---

## Overview

This workspace provides a complete ROS2 stack for the Steve Butler robot, supporting both simulation and real hardware deployment. The robot features:
- MMO-700 omnidirectional mobile base
- UR5e robotic arm with Robotiq gripper
- L515 RealSense camera on pan-tilt unit
- Dual LiDAR sensors for 360° obstacle detection
- Autonomous navigation with Nav2
- SLAM mapping capabilities

---

## Robot Communication (On Ubuntu 20/22)

When connecting to the physical robot:

- **Static IP**: `10.7.4.213`
- **Ethernet Port**: `192.168.60.90`

Configure your network settings to match the robot's subnet for proper communication.

---

## Package Descriptions

### steve_perception
Perception pipeline for object detection, scene understanding, and visual processing using the L515 RealSense camera.

**See detailed documentation**: [steve_perception/README.md](src/steve_perception/README.md)

### steve_navigation
Navigation stack for autonomous path planning and obstacle avoidance. 

**See detailed documentation**: [steve_navigation/README.md](src/steve_navigation/README.md)

### steve_simulation
Gazebo-based simulation environment for testing navigation, manipulation, and perception.

**See detailed documentation**: [steve_simulation/README.md](src/steve_simulation/README.md)

### steve_command_grounding
Natural language command understanding and grounding to robot actions, enabling semantic navigation and task execution.

**See detailed documentation**: [steve_command_grounding/README.md](src/steve_command_grounding/README.md)

### steve_hardware_bringup
Hardware initialization and driver management for the physical robot.

**See detailed documentation**: [steve_hardware_bringup/README.md](src/steve_hardware_bringup/README.md)

### steve_essentials
Core utilities, configuration files, and shared resources used across all packages.

### steve_manipulation
Manipulation control for the UR5e arm and Robotiq gripper, including motion planning and grasp execution.
(IN PROGRESS)

---

## Docker Support

This workspace is configured with a DevContainer for easy setup and consistent development environments.

### Option 1: VSCode DevContainer (Recommended)
1. Open the workspace in VSCode
2. Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac)
3. Select **"Dev Containers: Reopen in Container"**
4. VSCode will automatically build and configure the environment

### Option 2: Manual Docker Build
Build the Docker image manually:

```bash
docker build \
  --network=host \
  --build-arg DOCKER_REPO=osrf/ros \
  --build-arg ROS_DISTRO=humble \
  --build-arg IMAGE_SUFFIX=-desktop-full \
  --build-arg USERNAME=$(whoami) \
  --build-arg USER_UID=$(id -u) \
  --build-arg USER_GID=$(id -g) \
  -t steve_ros2:latest \
  -f .devcontainer/Dockerfile .
```

Run the container:

```bash
docker run -it --rm --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd):/home/steve_ros2_ws \
  -v ~/.ssh:/home/$(whoami)/.ssh \
  -w /home/steve_ros2_ws \
  steve_ros2:latest
```

---

## Dependencies

### Required ROS2 Packages
```bash
sudo apt install \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-joy \
  ros-humble-teleop-twist-joy \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-realsense2-camera
```

### Additional Dependencies
- **neo_mpo_700-2**: Base platform drivers
- **ur_robot_driver**: UR arm control


---

## Troubleshooting

### Build Issues
- Ensure all submodules are cloned: `git submodule update --init --recursive`
- Source ROS2: `source /opt/ros/humble/setup.bash`
- Clean build: `rm -rf build install log && colcon build`

### Simulation Issues
- Check Gazebo version: `gazebo --version`
- Verify DISPLAY variable: `echo $DISPLAY`
- For Docker: Enable X11 forwarding with `xhost +local:docker`

### Robot Connection Issues
- Verify network configuration matches robot IP
- Check ping: `ping 10.7.4.213`
- Ensure firewall allows ROS2 DDS traffic

For detailed troubleshooting, see the [navigation package README](src/steve_navigation/README.md#troubleshooting).

---

## Support

For issues or questions:
1. Check package-specific documentation
2. Review troubleshooting sections
3. Verify hardware connections (for real robot)
4. Check ROS2 logs: `ros2 launch --log-level debug`

---

## License

This project is developed at the Humanoid Robots Lab, University of Bonn.

---

**Note**: Package naming is currently in transition. Several packages (`steve_navigation`, `steve_simulation`, `steve_hardware_bringup`) will be renamed to follow the `steve_*` naming convention in future updates.
