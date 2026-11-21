#!/usr/bin/env bash
# One-time setup of a laptop (Ubuntu 22.04 + ROS 2 Humble, NetworkManager) for Steve:
#
#   1. Ethernet: a NetworkManager connection "steve-robot" on the wired port, DHCP from the robot
#      (the laptop gets 192.168.60.x, the robot is 192.168.60.90). It never becomes the default
#      route, so internet stays on Wi-Fi.
#   2. ROS / DDS: ~/.ros/fastdds_steve.xml (steve_hardware_bringup/config/fastdds_remote_pc.xml:
#      unicast discovery, the lab Wi-Fi blocks multicast) and ~/.ros/steve_dds.sh (ROS_DOMAIN_ID 69;
#      picks Ethernet when the cable is in, else Wi-Fi), sourced from ~/.bashrc.
#   3. Check: lease on the wired port, ping the robot.
#
# Usage:  ./scripts/steve_laptop_setup.sh [wired interface]     (default: the first Ethernet device)
#         ./scripts/steve_laptop_setup.sh --dds-only             (skip the network connection)
# Safe to run again: it updates what is there.

set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CON=steve-robot
ROBOT_ETH=192.168.60.90
ROBOT_WIFI=10.7.4.213
PROFILE_SRC="$WS_DIR/src/steve_hardware_bringup/config/fastdds_remote_pc.xml"

if [ -t 1 ]; then
  BOLD=$'\e[1m'; RED=$'\e[31m'; GREEN=$'\e[32m'; YELLOW=$'\e[33m'; RESET=$'\e[0m'
else
  BOLD=""; RED=""; GREEN=""; YELLOW=""; RESET=""
fi
info() { echo "${BOLD}== $*${RESET}"; }
ok()   { echo "   ${GREEN}$*${RESET}"; }
warn() { echo "   ${YELLOW}$*${RESET}"; }
err()  { echo "   ${RED}$*${RESET}"; }

dds_only=0
iface=""
case "${1:-}" in
  -h|--help) awk 'NR > 1 && !/^#/ { exit } NR > 1 { sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"; exit 0 ;;
  --dds-only) dds_only=1 ;;
  "") ;;
  *) iface=$1 ;;
esac

# ── 1. Ethernet (DHCP from the robot) ─────────────────────────────────────────────────────────
if [ "$dds_only" -eq 0 ]; then
  info "Ethernet connection '$CON'"
  command -v nmcli >/dev/null || { err "nmcli not found (NetworkManager); set the wired port to DHCP by hand"; exit 1; }
  if [ -z "$iface" ]; then
    iface=$(nmcli -t -f DEVICE,TYPE device | awk -F: '$2 == "ethernet" { print $1; exit }')
  fi
  [ -n "$iface" ] || { err "no Ethernet device found (USB adapter plugged in?); pass its name, see: nmcli device"; exit 1; }
  ok "wired port: $iface"

  settings=(connection.interface-name "$iface" connection.autoconnect yes connection.autoconnect-priority 10
            ipv4.method auto ipv4.never-default yes ipv4.ignore-auto-dns yes ipv6.method disabled)
  if nmcli -t -f NAME connection show | grep -qx "$CON"; then
    nmcli connection modify "$CON" "${settings[@]}"
    ok "updated"
  else
    nmcli connection add type ethernet con-name "$CON" "${settings[@]}" >/dev/null
    ok "created"
  fi
  # other wired profiles on this port (e.g. "Wired connection 1") would compete with it
  while IFS=: read -r name type dev; do
    [ "$type" = 802-3-ethernet ] && [ "$name" != "$CON" ] || continue
    [ -z "$dev" ] || [ "$dev" = "$iface" ] || continue
    nmcli connection modify "$name" connection.autoconnect no && warn "'$name': autoconnect off (steve-robot takes the port)"
  done < <(nmcli -t -f NAME,TYPE,DEVICE connection show)

  if [ "$(cat "/sys/class/net/$iface/carrier" 2>/dev/null || echo 0)" = 1 ]; then
    nmcli connection up "$CON" >/dev/null 2>&1 || true
    ip=""
    for _ in $(seq 20); do
      ip=$(ip -4 -o addr show dev "$iface" | awk '$4 ~ /^192\.168\.60\./ { split($4, a, "/"); print a[1]; exit }')
      [ -n "$ip" ] && break
      sleep 1
    done
    if [ -n "$ip" ]; then
      ok "address from the robot: $ip"
      ping -c 2 -W 1 "$ROBOT_ETH" >/dev/null 2>&1 && ok "robot $ROBOT_ETH answers" || warn "robot $ROBOT_ETH does not answer ping"
    else
      warn "no 192.168.60.x address after 20 s: robot on? cable in the robot's Ethernet port?"
    fi
  else
    warn "no cable on $iface: plug it into the robot, the connection comes up by itself"
  fi
fi

# ── 2. ROS / DDS ────────────────────────────────────────────────────────────────────────────────
info "ROS 2 / Fast DDS"
mkdir -p "$HOME/.ros"
[ -f "$PROFILE_SRC" ] || { err "$PROFILE_SRC missing: run ./scripts/repos.sh first (clones steve_hardware_bringup)"; exit 1; }
for f in fastdds_steve.xml steve_dds.sh; do
  [ -f "$HOME/.ros/$f" ] && ! cmp -s "$HOME/.ros/$f" "$([ $f = steve_dds.sh ] && echo "$WS_DIR/scripts/steve_dds.sh" || echo "$PROFILE_SRC")" \
    && cp -p "$HOME/.ros/$f" "$HOME/.ros/$f.bak-$(date +%Y%m%d%H%M%S)" && warn "kept your old ~/.ros/$f as a .bak"
done
cp "$PROFILE_SRC" "$HOME/.ros/fastdds_steve.xml"
cp "$WS_DIR/scripts/steve_dds.sh" "$HOME/.ros/steve_dds.sh"
ok "~/.ros/fastdds_steve.xml and ~/.ros/steve_dds.sh installed (ROS_DOMAIN_ID 69)"

marker="# Steve: workspace + ROS 2 network (scripts/steve_laptop_setup.sh)"
if ! grep -qF "$marker" "$HOME/.bashrc" 2>/dev/null; then
  {
    echo ""
    echo "$marker"
    echo "source /opt/ros/humble/setup.bash"
    echo "[ -f \"$WS_DIR/install/setup.bash\" ] && source \"$WS_DIR/install/setup.bash\""
    echo "source ~/.ros/steve_dds.sh"
  } >> "$HOME/.bashrc"
  ok "added to ~/.bashrc"
else
  ok "~/.bashrc already set up"
fi

# ── 3. Summary ──────────────────────────────────────────────────────────────────────────────────
info "Done"
ping -c 1 -W 1 "$ROBOT_WIFI" >/dev/null 2>&1 && ok "robot Wi-Fi $ROBOT_WIFI answers too" || true
echo "   Open a NEW terminal (it prints Ethernet or Wi-Fi), then:  ros2 daemon stop && ros2 topic list"
echo "   Ethernet is preferred: over Wi-Fi, gaps stop the WBC (\"odom stale\")."
