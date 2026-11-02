#!/usr/bin/env bash
set -eo pipefail
ROOT=/experiment/native_full_stack_20261102/opengv_guarded
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
set +e
cmake -S "$ROOT/source" -B "$ROOT/build" -DBUILD_SHARED_LIBS=ON -DBUILD_TESTS=OFF -DBUILD_PYTHON=OFF -DMARCH_NATIVE=OFF -DBUILD_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX="$ROOT/install" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
rc=$?
if test "$rc" -eq 0; then cmake --build "$ROOT/build" -j1; rc=$?; fi
if test "$rc" -eq 0; then cmake --install "$ROOT/build"; rc=$?; fi
echo "$rc" > "$ROOT/build.exitcode"
exit "$rc"
