#!/usr/bin/env bash
# Convert Isaac Sim /point_cloud (PointCloud2, frame=sensor) to /scan (LaserScan).
set -e
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

exec ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -r cloud_in:=/point_cloud \
  -r scan:=/scan \
  -p target_frame:=sensor \
  -p transform_tolerance:=0.05 \
  -p min_height:=-0.40 \
  -p max_height:=0.30 \
  -p angle_min:=-3.14159 \
  -p angle_max:=3.14159 \
  -p angle_increment:=0.0087 \
  -p scan_time:=0.10 \
  -p range_min:=0.20 \
  -p range_max:=12.0 \
  -p use_inf:=true \
  -p inf_epsilon:=1.0
