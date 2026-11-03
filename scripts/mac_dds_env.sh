#!/bin/bash
# DDS environment for talking to the real Steve robot from a Mac (pixi / RoboStack).
# Sourced automatically by pixi via [activation] in pixi.toml, or manually:
#   source scripts/mac_dds_env.sh
#
# Override defaults before sourcing, e.g.:  ROBOT_IP=10.7.4.213 source scripts/mac_dds_env.sh

ROBOT_IP="${ROBOT_IP:-192.168.60.90}"

WS_ROOT="${PIXI_PROJECT_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)}"
PEERS_XML="$WS_ROOT/.pixi/fastdds_peers.xml"
mkdir -p "$WS_ROOT/.pixi"

# Unicast discovery: multicast over the Mac's ethernet adapter is unreliable,
# so point Fast DDS straight at the robot (and localhost for local nodes).
# The robot runs ~30 participants; the default maxInitialPeersRange (4) only
# probes the first 4, so raise it. UDP only, so root processes (gripper camera)
# and user processes can talk without shared memory.
cat > "$PEERS_XML" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>steve_udp</transport_id>
      <type>UDPv4</type>
      <maxInitialPeersRange>120</maxInitialPeersRange>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="steve_peers" is_default_profile="true">
    <rtps>
      <userTransports><transport_id>steve_udp</transport_id></userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
      <builtin>
        <initialPeersList>
          <locator><udpv4><address>${ROBOT_IP}</address></udpv4></locator>
          <locator><udpv4><address>127.0.0.1</address></udpv4></locator>
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>
EOF

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-74}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$PEERS_XML"
