#!/usr/bin/env bash
# Run lidar cube detector with ROS2 Jazzy environment.
# Use this wrapper instead of sourcing setup.bash manually from zsh.
set -e
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$SCRIPT_DIR/lidar_cube_detector.py"
