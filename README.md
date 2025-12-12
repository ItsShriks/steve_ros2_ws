
## How to Run the Simulation

### 1. Clone the repository (with submodules)
```bash
git clone --recurse-submodules <url> -b test_nav2
```

### 2. Build the workspace
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select neo_simulation2 neo_nav2_bringup
```

### 3. Source the workspace
```bash
source install/setup.bash
```

### 4. Launch the simulation
Basic launch:
```bash
ros2 launch neo_simulation2 simulation.launch.py
```
<!--
#### Example: Launch with custom robot, world, arm, and pan-tilt
```bash
ros2 launch neo_simulation2 simulation.launch.py \
  my_robot:=mmo_700 world:=neo_workshop arm_type:=ur5e include_pan_tilt:=true
```-->

### 5. SLAM and Localization
We have provided dedicated launch files for running SLAM and Localization with the simulation.

#### SLAM Simulation
This launches the simulation with the MMO-700 robot and starts SLAM Toolbox for mapping.
```bash
ros2 launch neo_nav2_bringup slam_simulation.launch.py
```

#### Localization Simulation
This launches the simulation and starts AMCL for localization. You can specify a map file.
```bash
ros2 launch neo_nav2_bringup localization_simulation.launch.py map:=/path/to/your/map.yaml
```

### Visuals
![Gazebo Simulation](images/gazebo.png)
![RViz Visualization](images/rviz.png)


### 6. (Optional) Using Docker
This workspace is configured with a DevContainer for easy setup.

#### Option 1: VSCode DevContainer (Recommended)
1. Open the workspace in VSCode.
2. Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac) and select **"Dev Containers: Reopen in Container"**.
3. VSCode will automatically build the image and set up the environment.

#### Option 2: Build and Run Manually
Since the DevContainer is configured to build from source, you can also build and run the image manually:
1. Build the image:
   ```bash
docker build \
    --ssh default \
    --build-arg DOCKER_REPO=osrf/ros \
    --build-arg ROS_DISTRO=humble \
    --build-arg IMAGE_SUFFIX=-desktop-full \
    --build-arg USERNAME=$USER \
    -t test_nav2:latest \
    -f .devcontainer/Dockerfile .
   ```

2. Run the container:
   ```bash
   docker run -it --rm --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd):/home/steve_ros2_ws \
    -v ~/.ssh:/home/$USER/.ssh \
    test_nav2:latest
   ```
---
For more details, see the [Neobotix ROS2 simulation documentation](https://neobotix-docs.de/ros/ros2/simulation_classic.html) and the [modern Gazebo migration guide](https://neobotix-docs.de/ros/ros2/simulation_modern.html).

---
Since the classic gazebo has reached End of Life, There will be no further updates to this packages. 

All the robots in this packages have been migrated to modern Gazebo with some more additional features. More information about the installation and usage of the new modern Gazebo simulation can be [found in our documentation.](https://neobotix-docs.de/ros/ros2/simulation_modern.html)

