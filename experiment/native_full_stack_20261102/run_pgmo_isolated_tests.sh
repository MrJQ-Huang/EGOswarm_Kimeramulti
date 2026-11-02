#!/usr/bin/env bash
set -eo pipefail
WS="${1:?usage: run_pgmo_isolated_tests.sh WORKSPACE}"
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
ROOT="$(dirname "$0")"
python3 "$ROOT/prepare_pgmo_test_harness.py" --workspace "$WS" \
  --fixture-patch "$ROOT/pgmo_test_fixture_only_initialization.patch"
H="$WS/evidence/pgmo_test_harness"
sha256sum "$WS/devel/lib/libkimera_pgmo.so" > "$H/native_library_before.sha256"
cmake -S "$H" -B "$H/build" -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O2 -DNDEBUG'
cmake --build "$H/build" -j1
ldd "$H/build/native_upstream_test/test_kimera_pgmo" > "$H/native_test_linkage.txt"
set +e
"$H/build/native_upstream_test/test_kimera_pgmo" --gtest_output="xml:$H/upstream_unmodified.xml" \
  > "$H/upstream_unmodified.log" 2>&1
echo "$?" > "$H/upstream_unmodified.exitcode"
"$H/build/test_pgmo_initialized_fixture" --gtest_filter='TestDeformationGraph.*' \
  --gtest_output="xml:$H/fixture_initialization.xml" > "$H/fixture_initialization.log" 2>&1
echo "$?" > "$H/fixture_initialization.exitcode"
set -e
sha256sum -c "$H/native_library_before.sha256"
python3 - "$H" <<'PY'
import json,sys,time,xml.etree.ElementTree as E
from pathlib import Path
h=Path(sys.argv[1]); results={}
for name in ['upstream_unmodified','fixture_initialization']:
    tree=E.parse(h/(name+'.xml')).getroot()
    results[name]={'tests':int(tree.get('tests','0')),'failures':int(tree.get('failures','0')),
        'disabled':int(tree.get('disabled','0')),'errors':int(tree.get('errors','0')),
        'exitcode':int((h/(name+'.exitcode')).read_text())}
record={'wall_time':time.time(),'actual_native_library_unchanged':True,'production_source_modified':False,
    'results':results,'caveat':'Fixture-only initialization is isolated from native sources and does not change unmodified-suite results.'}
(h/'summary.json').write_text(json.dumps(record,indent=2)+'\n');print(json.dumps(record,indent=2))
PY
