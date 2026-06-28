#!/usr/bin/env bash
# Launch RViz2 (ROS2 Jazzy). Pass a config with: ./run_rviz.sh -d my_config.rviz
set -e
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
exec rviz2 "$@"
