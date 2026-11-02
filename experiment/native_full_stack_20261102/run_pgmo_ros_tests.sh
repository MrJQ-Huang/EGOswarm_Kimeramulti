#!/usr/bin/env bash
set -eo pipefail
WS="${1:?usage: run_pgmo_ros_tests.sh WORKSPACE}"
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export ROS_MASTER_URI=http://127.0.0.1:11551 ROS_IP=127.0.0.1
export ROS_HOME="$WS/evidence/pgmo_ros_tests/ros_home"
unset ROS_HOSTNAME ROS_NAMESPACE
mkdir -p "$WS/evidence/pgmo_ros_tests" "$ROS_HOME"
sha256sum "$WS/devel/lib/libkimera_pgmo.so" > "$WS/evidence/pgmo_ros_tests/native_library_before.sha256"
catkin build --workspace "$WS" --no-deps -j1 -p1 --no-status --summarize \
  kimera_pgmo_ros --make-args tests
set +e
catkin build --workspace "$WS" --no-deps -j1 -p1 --no-status --summarize \
  kimera_pgmo_ros --make-args run_tests
echo "$?" > "$WS/evidence/pgmo_ros_tests/run_tests.exitcode"
catkin_test_results "$WS/build/kimera_pgmo_ros/test_results" \
  > "$WS/evidence/pgmo_ros_tests/test_results.txt" 2>&1
echo "$?" > "$WS/evidence/pgmo_ros_tests/test_results.exitcode"
set -e
sha256sum -c "$WS/evidence/pgmo_ros_tests/native_library_before.sha256"
cat "$WS/evidence/pgmo_ros_tests/test_results.txt"
