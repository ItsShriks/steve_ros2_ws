#!/bin/bash
set -e

# Container name
CONTAINER_NAME="steve_ros2"
IMAGE_NAME="steve_ros2:latest"

# Check if container is already running

if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "Container $CONTAINER_NAME is already running. Attaching shell..."
    docker exec -it $CONTAINER_NAME /bin/bash
    exit 0
fi

# Check if container exists but is stopped
if [ "$(docker ps -aq -f status=exited -f name=$CONTAINER_NAME)" ]; then
    echo "Container $CONTAINER_NAME exists but is stopped. Restarting..."
    docker start $CONTAINER_NAME
    docker exec -it $CONTAINER_NAME /bin/bash
    exit 0
fi

# Get the directory where the script is located (scripts/)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
# Get the root of the workspace (parent of scripts/)
WS_ROOT="$(dirname "$SCRIPT_DIR")"

echo "Starting new container $CONTAINER_NAME from image $IMAGE_NAME..."

# Run the container
# Adjust volume mounts and device pass-through as needed
docker run -it --rm --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$WS_ROOT":/home/steve_ros2_ws \
    -v "$HOME/.ssh":/home/"$(whoami)"/.ssh \
    -w /home/steve_ros2_ws \
    $IMAGE_NAME
