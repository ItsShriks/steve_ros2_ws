# steve_ros2_ws

ROS2 Humble workspace for the Steve Butler robot project.

## Quick Links

- **Main Documentation**: See [neo_nav2_bringup/README.md](src/neo_nav2_bringup/README.md)
- **Navigation Package**: [neo_nav2_bringup](src/neo_nav2_bringup/)
- **Simulation Package**: [neo_simulation2](src/neo_simulation2/)

## Quick Start

```bash
# Clone repository
git clone --recurse-submodules git@gitlab.igg.uni-bonn.de:hrl_students/ws2526_butler/steve_ros2_ws.git -b dev

# Build workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select neo_simulation2 neo_nav2_bringup

# Source workspace
source install/setup.bash

# Launch simulation with navigation
ros2 launch neo_nav2_bringup localization_simulation.launch.py
```

For detailed documentation, see [neo_nav2_bringup/README.md](src/neo_nav2_bringup/README.md).
