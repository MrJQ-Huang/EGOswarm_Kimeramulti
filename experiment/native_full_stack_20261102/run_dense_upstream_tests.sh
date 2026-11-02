#!/usr/bin/env bash
set -eo pipefail
WS="${1:?usage: run_dense_upstream_tests.sh WORKSPACE}"
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export ROS_MASTER_URI=http://127.0.0.1:11551 ROS_IP=127.0.0.1
export ROS_HOME="$WS/evidence/test_ros_home"
unset ROS_HOSTNAME ROS_NAMESPACE
mkdir -p "$WS/evidence/tests" "$ROS_HOME"
# Native PGMO geometric deformation, mesh compression, IO and graph tests.
catkin build --workspace "$WS" --no-deps -j1 -p1 --no-status --summarize --force-cmake \
  kimera_pgmo --cmake-args -DKIMERA_PGMO_ENABLE_TESTS=ON
source "$WS/devel/setup.bash"
(cd "$WS/build/kimera_pgmo" && ctest --output-on-failure) \
  2>&1 | tee "$WS/evidence/tests/kimera_pgmo_ctest.log"
# This builds/runs the actual native ROS conversion and PGMO/DPGMO rostests.
# Rostest manages its own temporary master; no experimental master is started.
catkin build --workspace "$WS" --no-deps -j1 -p1 --no-status --summarize \
  kimera_pgmo_ros --make-args run_tests \
  2>&1 | tee "$WS/evidence/tests/kimera_pgmo_ros_tests.log"
catkin_test_results "$WS/build/kimera_pgmo_ros/test_results" \
  2>&1 | tee "$WS/evidence/tests/catkin_results.log"
