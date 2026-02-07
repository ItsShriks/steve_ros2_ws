#!/bin/bash
set -e

# Get the directory where the script is located (scripts/)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
# Get the root of the workspace (parent of scripts/)
WS_ROOT="$(dirname "$SCRIPT_DIR")"

echo "Building Docker image 'butler2' from workspace root: $WS_ROOT using .devcontainer/Dockerfile"

# Build the image
# We need to run docker build from the workspace root context
cd "$WS_ROOT"
DOCKER_BUILDKIT=1 docker build -t butler2:latest -f .devcontainer/Dockerfile .

echo "Build complete: butler2:latest"
