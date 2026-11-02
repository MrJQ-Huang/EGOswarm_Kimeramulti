#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
source /ego_ws/devel/setup.bash
unset ROS_HOSTNAME ROS_NAMESPACE
export ROS_MASTER_URI=http://192.168.5.21:11421
export ROS_IP=192.168.5.21
export ROS_LOG_DIR=/experiment/logs/ros
export PYTHONUNBUFFERED=1
export DISABLE_ROS1_EOL_WARNINGS=1
mkdir -p "$ROS_LOG_DIR"
exec "$@"
