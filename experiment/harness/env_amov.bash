#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
source /home/amov/ego_ws/devel/setup.bash
unset ROS_HOSTNAME ROS_NAMESPACE
export ROS_MASTER_URI=http://192.168.5.21:11421
export ROS_IP=192.168.5.6
export ROS_LOG_DIR=/home/amov/Drone/two_pc_ego_20261030/logs/ros
export PYTHONUNBUFFERED=1
mkdir -p "$ROS_LOG_DIR"
exec "$@"
