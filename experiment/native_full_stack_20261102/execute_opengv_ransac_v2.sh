#!/usr/bin/env bash
set -eo pipefail
ROOT=/experiment/native_full_stack_20261102
c++ -O2 -std=c++14 -I/opt/kimera_ws/src/opengv/include -I/usr/include/eigen3 "$ROOT/opengv_full_ransac_test.cpp" -L/opt/kimera_ws/devel/lib -Wl,-rpath,/opt/kimera_ws/devel/lib -lopengv -o "$ROOT/opengv_guarded/validation/full_ransac_test_v2"
LD_BIND_NOW=1 timeout 60 "$ROOT/opengv_guarded/validation/full_ransac_test_v2" > "$ROOT/opengv_guarded/validation/baseline_full_ransac_v2.jsonl" 2> "$ROOT/opengv_guarded/validation/baseline_full_ransac_v2.stderr"
echo 0 > "$ROOT/opengv_guarded/validation/baseline_v2.exitcode"
