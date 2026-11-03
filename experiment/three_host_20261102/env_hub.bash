#!/usr/bin/env bash
set -eo pipefail

# catkin setup files inspect positional parameters. Keep command arguments out
# of their parser, including commands whose arguments contain --help.
hub_command=("$@")
set --
source /opt/ros/noetic/setup.bash
source /opt/kimera_ws/devel/setup.bash
source /experiment/native_full_stack_20261102/uros_ws/devel/setup.bash
# Reuse the installed native backend ABI; runtime launch files choose the
# baseline completed-window protocol without enabling atomic-round extensions.
if test -f /experiment/three_host_20261102/hub_ws/devel/setup.bash; then
  source /experiment/three_host_20261102/hub_ws/devel/setup.bash
fi
if test -f /experiment/three_host_20261102/control_messages/devel/setup.bash; then
  source /experiment/three_host_20261102/control_messages/devel/setup.bash
fi
set -u
if test "${NATIVE_USE_OPENGV_GUARD:-0}" = 1; then
  hub_guard_root=/experiment/native_full_stack_20261102/opengv_guarded
  (cd "$hub_guard_root/install/lib" && sha256sum -c ../../runtime_gate.sha256 >&2)
  export LD_LIBRARY_PATH="$hub_guard_root/install/lib:${LD_LIBRARY_PATH:-}"
fi

unset ROS_HOSTNAME ROS_NAMESPACE
export ROS_PACKAGE_PATH="/experiment/harness/multimodal:${ROS_PACKAGE_PATH:-}"
export ROS_MASTER_URI="${NATIVE_ROS_MASTER_URI:-http://192.168.5.14:11431}"
export ROS_IP=192.168.5.14
export ROS_LOG_DIR="${THREE_HOST_ROS_LOG_DIR:-/experiment/three_host_20261102/logs/ros}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONUNBUFFERED=1
export DISABLE_ROS1_EOL_WARNINGS=1
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/tmp/host.Xauthority}"
export QT_X11_NO_MITSHM=1
case "${HUB_RENDER_MODE:-nvidia}" in
  nvidia)
    export LIBGL_ALWAYS_SOFTWARE=0
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    export QT_XCB_GL_INTEGRATION=xcb_glx
    ;;
  software)
    export LIBGL_ALWAYS_SOFTWARE=1
    unset __GLX_VENDOR_LIBRARY_NAME QT_XCB_GL_INTEGRATION
    ;;
  *) printf '%s\n' 'HUB_RENDER_MODE must be nvidia or software' >&2; exit 2 ;;
esac
mkdir -p "$ROS_LOG_DIR"
if test "${#hub_command[@]}" = 0; then
  printf '%s\n' 'env_hub.bash requires a command' >&2
  exit 2
fi
exec "${hub_command[@]}"
