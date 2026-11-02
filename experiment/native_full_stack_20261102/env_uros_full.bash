#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
source /opt/kimera_ws/devel/setup.bash
if test -f /experiment/native_full_stack_20261102/uros_ws/devel/setup.bash; then
  source /experiment/native_full_stack_20261102/uros_ws/devel/setup.bash
fi
if test "${NATIVE_USE_OPENGV_GUARD:-0}" = 1; then
  native_guard_root=/experiment/native_full_stack_20261102/opengv_guarded
  (cd "$native_guard_root/install/lib" && sha256sum -c ../../runtime_gate.sha256 >&2)
  export LD_LIBRARY_PATH="$native_guard_root/install/lib:${LD_LIBRARY_PATH:-}"
fi
unset ROS_HOSTNAME ROS_NAMESPACE
export ROS_PACKAGE_PATH="/experiment/harness/multimodal:${ROS_PACKAGE_PATH:-}"
export ROS_MASTER_URI="${NATIVE_ROS_MASTER_URI:-http://192.168.5.21:11431}"
export ROS_IP=192.168.5.21
export ROS_LOG_DIR=/experiment/native_full_stack_20261102/logs/ros
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONUNBUFFERED=1
export DISABLE_ROS1_EOL_WARNINGS=1
mkdir -p "$ROS_LOG_DIR"
exec "$@"
