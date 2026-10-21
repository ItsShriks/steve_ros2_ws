#!/bin/bash
# Source the Mac-only colcon overlay (built by `pixi run build-mac`) on every `pixi run` / `pixi shell`.
# Kept apart from build/ and install/, which hold the robot's Linux builds.

WS_ROOT="${PIXI_PROJECT_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)}"
if [ -f "$WS_ROOT/install_mac/local_setup.bash" ]; then
    # local_setup.* trips over unset variables when the caller runs with `set -u`
    set +u
    . "$WS_ROOT/install_mac/local_setup.bash"
    # rmw dlopen()s message type support by name (e.g. libneo_msgs2__rosidl_typesupport_fastrtps_c),
    # searching DYLD_LIBRARY_PATH; the overlay's local_setup doesn't set it on macOS.
    # pixi captures this environment through SIP-protected /usr/bin/env, which drops DYLD_*, so
    # the path is also kept as STEVE_MAC_LIBS and the pixi tasks set DYLD_LIBRARY_PATH from it.
    STEVE_MAC_LIBS=""
    for lib in "$WS_ROOT"/install_mac/*/lib; do
        STEVE_MAC_LIBS="$lib${STEVE_MAC_LIBS:+:$STEVE_MAC_LIBS}"
    done
    export STEVE_MAC_LIBS
    export DYLD_LIBRARY_PATH="$STEVE_MAC_LIBS${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
fi
