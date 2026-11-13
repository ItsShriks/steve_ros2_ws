#!/bin/bash
# Arm poses for the gripper camera calibration (D435i on the 2F-85's DROID mount): moves the UR5e through 8 views of the
# ball, holding each one HOLD seconds — press Enter in the calibrator's terminal during the hold, then `s` at the end.
#
#   terminal 1:  source ~/.ros/steve_dds.sh && ros2 launch steve_wbc gripper_camera.launch.py
#   terminal 2:  source ~/.ros/steve_dds.sh && ros2 run steve_wbc gripper_camera_calibrator --mount robotiq_2f85
#   terminal 3:  ~/Master_Thesis/steve_ros2_ws/scripts/gripper_camera_calib_poses.sh [HOLD_S] [--from N] [--dry-run]
#
# Setup the poses were computed for (2026-11-11, WBC model with the 2F-85 + DROID mount):
#   * the ball (target profile my_ball) 0.70 m in front of the base centre (the NEOBOTIX side, base +x), on the
#     robot's centre line, on the table (top ~0.83 m); the table's near edge >= 0.50 m from the base centre
#   * the base does not move; nothing else on the table near the ball
#   * every view points the gripper at (a few cm beside) the ball from 0.40-0.55 m: whichever side the camera is
#     on, it sees the ball 5-15 deg off its image centre
# Checked in the model: joint limits, self collision, >= 6 cm above the table top, along the paths between the poses
# too (straight lines in joint space, as the trajectory controller moves). The moves themselves are NOT collision
# checked (steve_manipulation manipulator --direct): stand at the EM stop. The script refuses to move while the EM-stop
# system reports a stop; Ctrl+C here does NOT stop a move already sent — use the EM stop.

HOLD=30   # (no `set -u`: the ROS / colcon setup files read unset variables)
FROM=1
DRY=0
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY=1 ;;
    --from) FROM="$2"; shift ;;
    -h|--help) sed -n 2,22p "$0"; exit 0 ;;
    *) HOLD="$1" ;;
  esac
  shift
done

# name | move time [s] | ROS joint angles [rad]: shoulder_pan shoulder_lift elbow wrist_1 wrist_2 wrist_3
POSES=(
  "home   | 7  | -1.5708 -1.5708 1.5708 -1.5708 -1.5708 0.0000"
  "view 1 | 6  | -2.0112 -2.0617 1.8914 -1.6376 -1.4614 -0.4268"
  "view 2 | 6  | -2.2892 -2.1221 2.0701 -1.7412 -1.1928 -0.5008"
  "view 3 | 6  | -1.7320 -2.3697 2.3951 -2.2028 -1.6301 -0.4404"
  "view 4 | 6  | -1.1779 -2.3208 2.2524 -2.0943 -2.1189 -0.1266"
  "view 5 | 7  | -1.6311 -2.1130 2.1083 -2.0223 -1.8356 -1.6041"
  "view 6 | 6  | -2.2950 -2.4075 2.0710 -1.6424 -1.2413 -2.2220"
  "view 7 | 6  | -2.3929 -2.0440 1.9842 -1.5916 -1.0551 -2.4575"
  "view 8 | 6  | -2.2727 -1.9891 1.6370 -1.4244 -1.2866 -3.5507"
  "home   | 14 | -1.5708 -1.5708 1.5708 -1.5708 -1.5708 0.0000"
)

if [ "$DRY" = 0 ]; then
  # the workspace and the robot's ROS domain (as a sourced terminal)
  [ -z "${ROS_DISTRO:-}" ] && source /opt/ros/humble/setup.bash
  WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  source "$WS/install/setup.bash"
  [ -f "$HOME/.ros/steve_dds.sh" ] && source "$HOME/.ros/steve_dds.sh"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  link ${STEVE_ROS_LINK:-?}"
  echo
  echo "Ball 0.70 m in front of the base centre on the table, base still, calibrator running, you at the EM stop."
  read -r -p "Move the arm through ${#POSES[@]} poses, holding each ${HOLD} s? [y/N] " ok
  [ "$ok" = "y" ] || { echo "nothing moved"; exit 0; }
fi

n=0
for entry in "${POSES[@]}"; do
  n=$((n + 1))
  name="$(echo "$entry" | cut -d'|' -f1 | xargs)"
  dur="$(echo "$entry" | cut -d'|' -f2 | xargs)"
  q="$(echo "$entry" | cut -d'|' -f3 | xargs)"
  # --from N: start at view N (the first home is always driven first)
  if [ "$n" -gt 1 ] && [ "$n" -lt "${#POSES[@]}" ] && [ $((n - 1)) -lt "$FROM" ]; then continue; fi
  echo "── ${name} (step ${n}/${#POSES[@]}): ${q}  (${dur} s)"
  if [ "$DRY" = 1 ]; then
    echo "   ros2 run steve_manipulation manipulator --direct --duration ${dur} joints ${q}"
  elif ! ros2 run steve_manipulation manipulator --direct --duration "$dur" joints $q; then
    echo "!! the move to ${name} failed (EM stop / safety field / controller?) — stopping here"
    echo "   resume with: $0 ${HOLD} --from $((n > 1 ? n - 1 : 1))"
    exit 1
  fi
  if [[ "$name" == view* ]]; then
    for ((s = HOLD; s > 0; s--)); do
      printf "\r   arm still: press Enter in the calibrator now (%2d s) " "$s"
      [ "$DRY" = 1 ] && break
      sleep 1
    done
    echo
  fi
done
echo "done: in the calibrator press s (solve + save), then restart gripper_camera.launch.py"
