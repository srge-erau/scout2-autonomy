#!/bin/bash
set -e

PROJECT="$(cd "$(dirname "$0")/.." && pwd)"

echo "Starting SCOUT2 LiDAR conversion..."

docker run -d --rm \
  --name scout2_scan \
  --network host \
  --entrypoint /bin/bash \
  -v "$PROJECT/cyclonedds.xml:/home/cyclonedds.xml:ro" \
  scout_autonomy:humble -lc '
    source /opt/ros/humble/setup.bash
    exec ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
      -r cloud_in:=/rslidar_points \
      -r scan:=/scan \
      -p target_frame:=base_link \
      -p min_height:=0.10 \
      -p max_height:=0.80 \
      -p angle_min:=-3.14159 \
      -p angle_max:=3.14159 \
      -p angle_increment:=0.00872665 \
      -p scan_time:=0.10 \
      -p range_min:=0.30 \
      -p range_max:=20.0 \
      -p use_inf:=true
  '

sleep 3

echo "Starting SLAM Toolbox..."

docker run -d --rm \
  --name scout2_slam \
  --network host \
  --entrypoint /bin/bash \
  -v "$PROJECT/cyclonedds.xml:/home/cyclonedds.xml:ro" \
  -v "$PROJECT/mapper_params_scout.yaml:/home/scout_autonomy/mapper_params_scout.yaml:ro" \
  -v "$PROJECT/maps:/home/scout_autonomy/maps" \
  scout_autonomy:humble -lc '
    source /opt/ros/humble/setup.bash
    exec ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
      --params-file /home/scout_autonomy/mapper_params_scout.yaml
  '

echo "SCOUT2 mapping processes started."
