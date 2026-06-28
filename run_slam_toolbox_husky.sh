#!/usr/bin/env bash
# Run slam_toolbox for the Isaac Sim Husky lidar experiment.
set -e
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file "$SCRIPT_DIR/slam_toolbox_husky_params.yaml"
