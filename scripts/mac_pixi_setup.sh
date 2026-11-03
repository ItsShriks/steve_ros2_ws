#!/bin/bash
# One-time setup of RViz2 + DDS for the real robot in the workspace's pixi project (macOS).
# Usage (from the workspace root):  ./scripts/mac_pixi_setup.sh
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
TOML="$WS_ROOT/pixi.toml"
cd "$WS_ROOT"

if [ ! -f "$TOML" ]; then
    echo "No pixi.toml in $WS_ROOT"
    exit 1
fi

cp "$TOML" "$TOML.bak"
echo "Backed up pixi.toml -> pixi.toml.bak"

# 1. RoboStack channel
if ! grep -q "robostack-staging" "$TOML"; then
    pixi workspace channel add robostack-staging 2>/dev/null \
        || pixi project channel add robostack-staging
fi

# 2. ROS 2 CLI + RViz2 + Fast DDS
pixi add ros-humble-ros-base ros-humble-rviz2 ros-humble-rmw-fastrtps-cpp

# 3. Source the DDS env on every `pixi shell` / `pixi run`
if grep -q "mac_dds_env.sh" "$TOML"; then
    echo "DDS activation already present"
elif grep -q "^\[activation\]" "$TOML"; then
    echo "pixi.toml already has an [activation] section; add this line to it manually:"
    echo '  scripts = ["scripts/mac_dds_env.sh"]'
else
    printf '\n[activation]\nscripts = ["scripts/mac_dds_env.sh"]\n' >> "$TOML"
    echo "Added [activation] scripts = [\"scripts/mac_dds_env.sh\"]"
fi

echo ""
echo "Done. Next:"
echo "  pixi shell"
echo "  ros2 daemon stop && ros2 topic list   # robot topics should appear"
echo "  rviz2"
