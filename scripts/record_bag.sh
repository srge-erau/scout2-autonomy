#!/usr/bin/env bash
# Record a rosbag on the Scout 2.0 Jetson with the standard sensor topics.
#
# Usage:
#   ./record_bag.sh NAME            record until Ctrl+C
#   ./record_bag.sh NAME SECONDS    record for a fixed time, then stop
#
# A NAME ending in _cmd also records /joy and /mux/cmd_vel (Xbox controller runs).
# Bags containing commands must never be played back on the powered robot.
#
# This script only records. It does not publish anything and cannot move the robot.
# Run it inside tmux, with the ZED already started (start-up guide, section 9.5).

set -euo pipefail

NAME="${1:?usage: record_bag.sh NAME [SECONDS]}"
DURATION="${2:-}"
BAG_DIR="$HOME/bags"

TOPICS=(
  /tf /tf_static
  /rslidar_points
  /zed/zed_node/imu/data /zed/zed_node/imu/data_raw
  /zed/zed_node/left/image_rect_color/compressed
  /zed/zed_node/right/image_rect_color/compressed
  /zed/zed_node/left/camera_info /zed/zed_node/right/camera_info
  /zed/zed_node/depth/depth_registered
  /zed/zed_node/odom
  /agilex_odom
)

if [[ "$NAME" == *_cmd ]]; then
  TOPICS+=(/joy /mux/cmd_vel)
fi

if [[ -n "$DURATION" && ! "$DURATION" =~ ^[0-9]+$ ]]; then
  echo "SECONDS must be a whole number." >&2
  exit 1
fi

# Bag timestamps are valid only after the clock has synchronised.
# The Jetson starts at 1970 on every cold boot (start-up guide, Known Issue 14).
if ! timedatectl show -p NTPSynchronized --value | grep -q yes; then
  echo "The clock is not synchronised. Wait, then check with: timedatectl" >&2
  exit 1
fi

# Without the ZED running, the bag records with no camera or IMU and gives no warning.
# Capture the list first: piping straight into grep -q makes ros2 fail with a broken
# pipe, which pipefail then reports as "not found".
TOPIC_LIST="$(ros2 topic list 2>/dev/null || true)"
if ! grep -qx '/zed/zed_node/imu/data' <<<"$TOPIC_LIST"; then
  echo "ZED topics not found. Start the camera first: tmux new -s zed, then: front" >&2
  exit 1
fi

if [[ -z "${TMUX:-}" ]]; then
  echo "Warning: not running inside tmux. A dropped connection will stop the recording." >&2
fi

mkdir -p "$BAG_DIR"
cd "$BAG_DIR"

if [[ -e "$NAME" ]]; then
  echo "$BAG_DIR/$NAME already exists. Choose another name." >&2
  exit 1
fi

echo "Recording ${#TOPICS[@]} topics to $BAG_DIR/$NAME"
echo "Wait for 'All requested topics are subscribed' before moving the robot."

if [[ -n "$DURATION" ]]; then
  timeout --foreground -s INT "$DURATION" ros2 bag record -o "$NAME" "${TOPICS[@]}" || true
else
  ros2 bag record -o "$NAME" "${TOPICS[@]}" || true
fi

echo
ros2 bag info "$NAME"
