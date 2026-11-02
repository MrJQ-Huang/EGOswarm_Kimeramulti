#!/usr/bin/env bash
set -e
source /home/amov/Drone/kimera_multi_sim_20261030/environment.bash
if test "${NATIVE_USE_OPENGV_GUARD:-0}" = 1; then
  native_guard_root=/home/amov/Drone/two_pc_ego_20261030/native_full_stack_20261102/opengv_guarded
  (cd "$native_guard_root/install/lib" && sha256sum -c ../../runtime_gate.sha256 >&2)
  export LD_LIBRARY_PATH="$native_guard_root/install/lib:${LD_LIBRARY_PATH:-}"
fi
unset ROS_HOSTNAME ROS_NAMESPACE
export ROS_PACKAGE_PATH="/home/amov/Drone/two_pc_ego_20261030/harness/multimodal:${ROS_PACKAGE_PATH:-}"
export ROS_MASTER_URI="${NATIVE_ROS_MASTER_URI:-http://192.168.5.21:11431}"
export ROS_IP=192.168.5.6
export ROS_LOG_DIR=/home/amov/Drone/two_pc_ego_20261030/native_full_stack_20261102/logs/ros
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONUNBUFFERED=1
export DISABLE_ROS1_EOL_WARNINGS=1
mkdir -p "$ROS_LOG_DIR"
exec "$@"
