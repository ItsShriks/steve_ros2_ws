#!/bin/bash
# Clean any stale build artifacts to prevent CMake cache errors
git submodule update --init --recursive
rm -rf /home/ws/build /home/ws/install /home/ws/log

#mkdir -p src
sudo rosdep update
sudo rosdep install --from-paths /home/ws/src --ignore-src -y
sudo chown -R $(whoami)/home/ws/
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
