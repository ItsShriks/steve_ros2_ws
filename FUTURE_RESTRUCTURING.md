# Future Workspace Restructuring Guide

This document outlines the planned modular restructuring of the Steve ROS2 workspace into 6 major package groups. This restructuring will improve organization, maintainability, and clarity of the codebase.

## Proposed Structure

```
steve_ros2_ws/src/
├── steve_navigation/          # Navigation stack
├── steve_perception/          # Sensors and perception
├── steve_manipulation/        # Arm control
├── steve_bringup/            # Core robot bringup
├── steve_essentials/         # Core dependencies
└── steve_command/            # Teleoperation and commands
```

## Package Organization

### 1. steve_navigation
**Purpose**: All navigation-related packages and configurations

**Packages to include**:
- `neo_nav2_bringup` - Localization, mapping, and navigation launch files
- Navigation configs (AMCL, Nav2, etc.)

**Key files**:
- `launch/slam.launch.py`
- `launch/localization.launch.py`
- `launch/navigation_neo.launch.py`
- `launch/mapping.launch.py`
- `config/localization.yaml`
- `config/navigation.yaml`
- `config/mapping.yaml`

---

### 2. steve_perception
**Purpose**: Sensor drivers, camera processing, and perception pipelines

**Packages to include**:
- `steve_perception` (existing perception package)
- `realsense-ros` - RealSense camera drivers
- `steve_pan_tilt_controller` - Pan-tilt unit control

**Key capabilities**:
- Camera drivers (L515, D405)
- Pan-tilt motor control
- Point cloud processing
- Object detection/recognition

---

### 3. steve_manipulation
**Purpose**: Robotic arm control and manipulation

**Packages to include**:
- UR arm drivers and controllers
- Gripper/end-effector packages (when added)
- Manipulation planning and execution

**Key files**:
- UR5e configuration
- MoveIt configs (if added)
- Gripper control nodes

---

### 4. steve_bringup
**Purpose**: Core robot initialization for both simulation and real hardware

**Packages to include**:
- `steve_bringup` (current package)
- Robot description (URDF/xacro)
- Hardware interface launch files

**Key files**:
- `launch/steve_bringup.launch.py` - Unified bringup
- `launch/hardware_bringup.launch.py` - Real robot hardware
- `urdf/mmo_700.urdf.xacro` - Robot description
- `urdf/meshes/` - 3D meshes

---

### 5. steve_essentials
**Purpose**: Core ROS2 packages and dependencies shared across the system

**Packages to include**:
- `neo_simulation2` - Gazebo simulation
- `neo_common2` - Common utilities
- `neo_msgs2` - Custom message definitions
- `neo_srvs2` - Custom service definitions
- `neo_relayboard_v2-2` - Base controller interface
- `neo_kinematics_omnidrive2` - Omnidirectional kinematics
- `neo_sick_s300-2` - LiDAR drivers

**Key capabilities**:
- Simulation environments
- Message/service definitions
- Base platform drivers
- LiDAR sensor drivers

---

### 6. steve_command
**Purpose**: User interfaces for controlling the robot

**Packages to include**:
- `neo_teleop2` - Teleoperation nodes
- `joystick_drivers` - Joystick/gamepad support
- High-level command interfaces (future)

**Key capabilities**:
- Keyboard teleoperation
- Joystick control
- High-level task commands (future)

---

## Migration Strategy

When ready to implement this restructuring:

### Step 1: Create Directory Structure
```bash
cd /Users/shrikar/neo_700/steve_ros2_ws/src
mkdir steve_navigation steve_perception steve_manipulation steve_essentials steve_command
```

### Step 2: Move Packages
Move packages into their respective directories:
```bash
# Navigation
mv neo_nav2_bringup steve_navigation/

# Perception
mv steve_perception steve_pan_tilt_controller realsense-ros steve_perception/

# Manipulation
# (Create when manipulation packages are added)

# Bringup
# steve_bringup already exists

# Essentials
mv neo_simulation2 neo_common2 neo_msgs2 neo_srvs2 neo_relayboard_v2-2 \
   neo_kinematics_omnidrive2 neo_sick_s300-2 steve_essentials/

# Command
mv neo_teleop2 joystick_drivers steve_command/
```

### Step 3: Update Dependencies
Update `package.xml` files to reflect new package locations if needed.

### Step 4: Rebuild Workspace
```bash
cd /Users/shrikar/neo_700/steve_ros2_ws
colcon build
source install/setup.bash
```

### Step 5: Test All Functionality
- Test simulation mode
- Test real robot mode (when available)
- Verify all launch files work correctly

---

## Benefits of This Structure

1. **Clarity**: Clear separation of concerns makes it easy to find relevant code
2. **Maintainability**: Changes to one subsystem don't affect others
3. **Scalability**: Easy to add new packages to appropriate categories
4. **Collaboration**: Different team members can work on different subsystems
5. **Documentation**: Easier to document and understand the system architecture

---

## Notes

- This restructuring should be done when the system is stable
- Ensure all tests pass before and after restructuring
- Update all documentation and README files
- Consider creating a migration script to automate the process
- Keep git history intact by using `git mv` instead of regular `mv`
