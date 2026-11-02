#!/usr/bin/env bash
set -eo pipefail
WS="${1:?usage: build_dense_overlay.sh WORKSPACE UNDERLAY [JOBS]}"
UNDERLAY="${2:?underlay devel prefix required}"
JOBS="${3:-2}"
case "$JOBS" in 1|2) ;; *) echo 'JOBS must be 1 or 2' >&2; exit 2;; esac
test -f "$WS/source_validation.json"
test -f "$UNDERLAY/setup.bash"
source /opt/ros/noetic/setup.bash
source "$UNDERLAY/setup.bash"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
mkdir -p "$WS/evidence"
python3 - "$WS" <<'PY'
import json,os,platform,subprocess,sys,time
from pathlib import Path
w=Path(sys.argv[1]); types=['pose_graph_tools_msgs/PoseGraph','pose_graph_tools_msgs/PoseGraphNode','pose_graph_tools_msgs/PoseGraphEdge']
md5={t:subprocess.check_output(['rosmsg','md5',t],text=True).strip() for t in types}
record={'wall_time':time.time(),'architecture':platform.machine(),'message_md5':md5,
        'underlay_cmake_prefix_path':os.environ.get('CMAKE_PREFIX_PATH',''),
        'compiler':subprocess.check_output(['c++','--version'],text=True).splitlines()[0]}
(w/'evidence/underlay_abi.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps(record))
PY
ARGS=(
  -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O2 -DNDEBUG'
  -DCATKIN_ENABLE_TESTING=ON -DBUILD_TESTING=OFF
  -DKIMERA_PGMO_ENABLE_TESTS=OFF -DCONFIG_UTILS_BUILD_TESTS=OFF
  -DCONFIG_UTILS_BUILD_DEMOS=OFF -DUSE_SYSTEM_EIGEN=ON
  -DUSE_SYSTEM_GLOG=ON -DUSE_SYSTEM_PROTOBUF=ON -DPYTHON_EXECUTABLE=/usr/bin/python3
)
# Optional ABI locations are discovered by the operator from the frozen underlay.
# They override CMake discovery only if supplied; no architecture-specific path is assumed.
for var in GTSAM_DIR GTSAM_UNSTABLE_DIR KimeraRPGO_DIR OpenCV_DIR; do
  if test -n "${!var:-}"; then ARGS+=("-D${var}=${!var}"); fi
done
printf '%s\n' "${ARGS[@]}" > "$WS/evidence/cmake_args.txt"
catkin config --workspace "$WS" --extend "$UNDERLAY" --merge-devel --cmake-args "${ARGS[@]}"
# The upstream PGMO package declares voxblox_msgs only as an exec dependency.
# Build it first so find_package(voxblox_msgs QUIET) enables its native frontend.
catkin build --workspace "$WS" -j"$JOBS" -p1 --no-status --summarize voxblox_msgs
catkin build --workspace "$WS" -j"$JOBS" -p1 --no-status --summarize --force-cmake \
  kimera_pgmo_ros kimera_pgmo_rviz kimera_semantics_ros disparity_image_proc mesh_rviz_plugins kimera_rviz_markers
source "$WS/devel/setup.bash"
python3 "$(dirname "$0")/verify_dense_overlay.py" --workspace "$WS" --underlay "$UNDERLAY"
