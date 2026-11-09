#!/usr/bin/env bash
set -eo pipefail
distributed_host="$1"
shift
distributed_command=("$@")
set --
source /opt/ros/noetic/setup.bash
case "$distributed_host" in
  amov)
    distributed_root=/home/amov/Drone/two_pc_ego_20261030
    source /home/amov/Drone/kimera_multi_sim_20261030/environment.bash
    distributed_overlay="$distributed_root/native_full_stack_20261102/amov_ws/devel/setup.bash"
    distributed_ip=192.168.5.6
    ;;
  uros)
    distributed_root=/experiment
    source /opt/kimera_ws/devel/setup.bash
    distributed_overlay="$distributed_root/native_full_stack_20261102/uros_ws/devel/setup.bash"
    distributed_ip=192.168.5.21
    ;;
  *) echo 'Expected amov or uros' >&2; exit 2 ;;
esac
if test -f "$distributed_overlay"; then source "$distributed_overlay"; fi
if test -f "$distributed_root/three_host_20261102/control_messages/devel/setup.bash"; then
  source "$distributed_root/three_host_20261102/control_messages/devel/setup.bash"
fi
if test "${NATIVE_USE_OPENGV_GUARD:-0}" = 1; then
  distributed_guard="$distributed_root/native_full_stack_20261102/opengv_guarded"
  (cd "$distributed_guard/install/lib" && sha256sum -c ../../runtime_gate.sha256 >&2)
  export LD_LIBRARY_PATH="$distributed_guard/install/lib:${LD_LIBRARY_PATH:-}"
fi
unset ROS_HOSTNAME ROS_NAMESPACE
export ROS_PACKAGE_PATH="$distributed_root/harness/multimodal:${ROS_PACKAGE_PATH:-}"
export ROS_IP="$distributed_ip"
export ROS_MASTER_URI="${NATIVE_ROS_MASTER_URI:-http://$distributed_ip:11431}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONUNBUFFERED=1
export DISABLE_ROS1_EOL_WARNINGS=1
exec "${distributed_command[@]}"
