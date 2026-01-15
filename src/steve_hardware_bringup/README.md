# steve_hardware_bringup

Hardware bringup package for the Steve robot. This package provides complete hardware initialization for the real robot, eliminating dependency on `mpo_700_workspace`.

## Overview

This package initializes all hardware components of the Steve robot:
- **Robot Base**: MPO-700 platform with relayboard and omnidirectional kinematics
- **UR5e Arm**: Universal Robots UR5e manipulator
- **Joystick Control**: Logitech joystick for manual teleoperation
- **Pan-Tilt Motors**: Dynamixel XH430-W250 motors for camera positioning
- **Dual LiDAR**: Two SICK S300 laser scanners with filtering
- **Camera**: Intel RealSense L515 depth camera on pan-tilt unit

## Hardware Requirements

### Device Ports
Ensure the following devices are properly connected and have the correct permissions:
- `/dev/neo-relayboard` - Neobotix relayboard
- `/dev/neo-s300-1` - SICK S300 LiDAR 1
- `/dev/neo-s300-2` - SICK S300 LiDAR 2
- `/dev/ttyUSB0` - Dynamixel pan-tilt motors
- `/dev/input/js0` - Logitech joystick
- `can0` - CAN interface for motor control

### Network Configuration
- UR5e arm must be accessible at IP: `192.168.1.102` (configurable)
- Ensure network connectivity between robot PC and UR5e controller

## Installation

Build the package in your ROS2 workspace:

```bash
cd /Users/shrikar/neo_700/steve_ros2_ws
colcon build --packages-select steve_hardware_bringup
source install/setup.bash
```

## Usage

### Full System Bringup

Launch all hardware components:

```bash
ros2 launch steve_hardware_bringup hardware_bringup.launch.py
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `robot_namespace` | `''` | Top-level namespace for the robot |
| `arm_type` | `ur5e` | UR arm type (ur5, ur5e, ur10, ur10e) |
| `robot_ip` | `192.168.1.102` | IP address of the UR arm |
| `enable_camera` | `true` | Enable RealSense L515 camera |
| `enable_pan_tilt` | `true` | Enable pan-tilt motors |

### Example with Custom Arguments

```bash
# Launch with namespace
ros2 launch steve_hardware_bringup hardware_bringup.launch.py robot_namespace:=steve

# Launch without camera
ros2 launch steve_hardware_bringup hardware_bringup.launch.py enable_camera:=false

# Launch with different UR arm IP
ros2 launch steve_hardware_bringup hardware_bringup.launch.py robot_ip:=192.168.1.105
```

### Individual Component Launch

You can also launch individual components separately:

#### Robot Base Only
```bash
ros2 launch steve_hardware_bringup robot_base.launch.py
```

#### LiDAR Sensors Only
```bash
ros2 launch steve_hardware_bringup lidar.launch.py
```

#### Joystick Control Only
```bash
ros2 launch steve_hardware_bringup teleop.launch.py
```

#### UR5e Arm Only
```bash
ros2 launch steve_hardware_bringup ur5e_arm.launch.py
```

#### Pan-Tilt Unit Only
```bash
ros2 launch steve_hardware_bringup pan_tilt.launch.py
```

## Configuration

Configuration files are located in the `config/` directory:

- `relayboard_v2.yaml` - Relayboard configuration
- `kinematics.yaml` - Omnidrive kinematics parameters
- `socket.yaml` - SocketCAN motor communication
- `teleop.yaml` - Joystick button mappings and velocity limits
- `lidar/s300_1.yaml` - LiDAR 1 configuration
- `lidar/s300_2.yaml` - LiDAR 2 configuration
- `lidar/s300_filter_1.yaml` - LiDAR 1 scan filter
- `lidar/s300_filter_2.yaml` - LiDAR 2 scan filter

### Modifying Configurations

To adjust robot behavior, edit the relevant YAML files:

**Velocity Limits** (teleop.yaml):
```yaml
scale_angular_z: 0.6  # rad/s
scale_linear_x: 0.6   # m/s
scale_linear_y: 0.6   # m/s
```

**LiDAR Scan Range** (s300_filter_*.yaml):
```yaml
scan_intervals: [[-2.27, 2.27]]  # ±85° in radians
```

## Topics

### Published Topics

- `/joint_states` - Robot joint states (base + arm)
- `/scan` - Merged LiDAR scan data
- `/lidar_1/scan_filtered` - Filtered scan from LiDAR 1
- `/lidar_2/scan_filtered` - Filtered scan from LiDAR 2
- `/cmd_vel` - Velocity commands to robot base
- `/joy` - Joystick input data
- `/pan_tilt_camera/color/image_raw` - RGB camera image
- `/pan_tilt_camera/depth/image_rect_raw` - Depth image
- `/pan_tilt_camera/depth/color/points` - Point cloud

### Subscribed Topics

- `/cmd_vel` - Velocity commands for base movement
- `/ur5e/scaled_joint_trajectory_controller/joint_trajectory` - Arm trajectory commands

## Troubleshooting

### Device Permission Issues

If you encounter permission errors for devices:

```bash
# Add user to dialout group for serial devices
sudo usermod -a -G dialout $USER

# Set permissions for CAN interface
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
```

### UR5e Connection Issues

1. Verify network connectivity:
   ```bash
   ping 192.168.1.102
   ```

2. Check UR5e is in remote control mode
3. Ensure external control program is running on UR5e pendant

### LiDAR Not Detected

1. Check device symlinks:
   ```bash
   ls -l /dev/neo-s300-*
   ```

2. Verify baud rate matches configuration (500000)

### Joystick Not Responding

1. Test joystick input:
   ```bash
   jstest /dev/input/js0
   ```

2. Check deadman button is pressed (button 5 by default)

### Pan-Tilt Motors Not Moving

1. Verify Dynamixel connection:
   ```bash
   ls -l /dev/ttyUSB0
   ```

2. Check motor IDs match configuration (IDs 1 and 2)
3. Ensure motors are powered

## Dependencies

This package depends on the following ROS2 packages:
- `neo_relayboard_v2-2`
- `neo_kinematics_omnidrive2`
- `neo_sick_s300-2`
- `neo_teleop2`
- `joy`
- `ur_robot_driver`
- `ur_description`
- `steve_pan_tilt_controller`
- `realsense2_camera`
- `robot_state_publisher`
- `topic_tools`

## License

MIT

## Maintainer

Your Name <your_email@example.com>
