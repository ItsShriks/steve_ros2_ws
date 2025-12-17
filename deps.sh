#!/bin/bash
# ROS2 Navigation Simulation - Dependency Installation Script
# This script installs all required dependencies for the neo_simulation2 workspace

set -e  # Exit on error

echo "=========================================="
echo "Installing ROS2 Navigation Dependencies"
echo "=========================================="

# Update package lists
echo ""
echo "[1/4] Updating package lists..."
sudo apt update

# Install ROS2 control packages
echo ""
echo "[2/4] Installing ROS2 control packages..."
sudo apt install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    ros-humble-robotiq-description  

# Install Nav2 packages
echo ""
echo "[3/4] Installing Nav2 navigation packages..."
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-common \
    ros-humble-nav2-core \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-msgs \
    ros-humble-nav2-util \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-controller \
    ros-humble-nav2-planner \
    ros-humble-nav2-behaviors \
    ros-humble-nav2-waypoint-follower \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-navfn-planner \
    ros-humble-nav2-smac-planner \
    ros-humble-nav2-theta-star-planner

# Install SLAM and localization packages
echo ""
echo "[4/4] Installing SLAM and localization packages..."
sudo apt install -y \
    ros-humble-slam-toolbox \
    ros-humble-nav2-amcl

# Install additional useful packages
echo ""
echo "Installing additional utilities..."
sudo apt install -y \
    ros-humble-teleop-twist-keyboard \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro

echo ""
echo "=========================================="
echo "✅ All dependencies installed successfully!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Clone the serial package (if not already done):"
echo "   cd src"
echo "   git clone -b ros2 https://github.com/tylerjw/serial.git"
echo "   cd .."
echo ""
echo "2. Build the workspace:"
echo "   source /opt/ros/humble/setup.bash"
echo "   colcon build --symlink-install"
echo ""
echo "3. Source the workspace:"
echo "   source install/setup.bash"
echo ""
echo "4. Launch the simulation:"
echo "   ros2 launch neo_nav2_bringup localization_simulation.launch.py"
echo ""
