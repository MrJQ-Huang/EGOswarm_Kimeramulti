#!/usr/bin/env bash
set -eo pipefail
ROOT=/experiment/native_full_stack_20261102
export GTSAM_DIR=/opt/kimera_ws/devel/lib/cmake/GTSAM
export GTSAM_UNSTABLE_DIR=/opt/kimera_ws/devel/lib/cmake/GTSAM_UNSTABLE
export KimeraRPGO_DIR=/opt/kimera_ws/devel/lib/cmake/KimeraRPGO
export OpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
set +e
bash "$ROOT/build_dense_overlay.sh" "$ROOT/uros_ws" /opt/kimera_ws/devel 2
rc=$?
printf '%s\n' "$rc" > "$ROOT/logs/build.exitcode"
exit "$rc"
