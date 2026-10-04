#!/bin/bash
set -e

# Get the directory where the script is located (scripts/)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
# Get the root of the workspace (parent of scripts/)
WS_ROOT="$(dirname "$SCRIPT_DIR")"

echo "Building Docker image 'steve_ros2' from workspace root: $WS_ROOT using .devcontainer/Dockerfile"

# Build the image
# We need to run docker build from the workspace root context
cd "$WS_ROOT"
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

echo "Build complete: steve_ros2:latest"
