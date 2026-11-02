#!/usr/bin/env bash
set -euo pipefail
# Run only inside the isolated UROS ROS-Noetic container. AMOV already has these.
test "$(id -u)" -eq 0
test -f /.dockerenv
. /etc/os-release
test "$VERSION_ID" = 20.04
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends \
  build-essential cmake pkg-config file python3-catkin-tools python3-rospkg \
  libyaml-cpp-dev libgoogle-glog-dev libgflags-dev libgtest-dev \
  libprotobuf-dev protobuf-compiler libeigen3-dev \
  libboost-filesystem-dev libboost-system-dev \
  ros-noetic-cmake-modules ros-noetic-cv-bridge ros-noetic-image-geometry \
  ros-noetic-depth-image-proc ros-noetic-image-proc \
  ros-noetic-eigen-conversions ros-noetic-tf-conversions ros-noetic-tf2-eigen \
  ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-camera-info-manager \
  ros-noetic-interactive-markers ros-noetic-rviz ros-noetic-rostest ros-noetic-rosunit
# No upgrade/dist-upgrade, no GTSAM rebuild, and no AMOV binaries are imported.
