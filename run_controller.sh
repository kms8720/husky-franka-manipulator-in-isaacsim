#!/usr/bin/env bash
# Husky virtual joystick -> publishes /cmd_vel  (ROS2 Jazzy, Ubuntu 24.04 ARM64)
# NOTE: uses /usr/bin/python3 (3.12, what apt ROS2 targets), NOT conda's python 3.13.
set -e
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$SCRIPT_DIR/husky_controller.py"
