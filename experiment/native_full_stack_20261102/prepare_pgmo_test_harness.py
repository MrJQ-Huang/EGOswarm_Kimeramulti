#!/usr/bin/env python3
"""Build untouched upstream tests against the actual independently built PGMO lib."""
import argparse
from pathlib import Path
import shutil
import subprocess


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--workspace", type=Path, required=True)
    p.add_argument("--fixture-patch", type=Path, required=True)
    a = p.parse_args()
    w = a.workspace.resolve()
    test = w / "src/Kimera-PGMO/kimera_pgmo/test"
    h = w / "evidence/pgmo_test_harness"
    if not (w / "evidence/overlay_verification.json").is_file():
        raise SystemExit("Verify the actual native overlay before creating the test harness")
    if h.exists():
        raise SystemExit("Preserve the existing test harness; choose an explicit new test attempt")
    h.mkdir(parents=True)
    fixed = h / "test_deformation_graph_initialized_fixture.cpp"
    shutil.copyfile(test / "test_deformation_graph.cpp", fixed)
    shutil.copyfile(a.fixture_patch, h / "fixture_only_initialization.patch")
    subprocess.run(["patch", str(fixed), str(a.fixture_patch)], check=True)
    (h / "CMakeLists.txt").write_text(f'''cmake_minimum_required(VERSION 3.5)
project(kimera_pgmo VERSION 0.0.1 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(kimera_pgmo REQUIRED)
find_package(config_utilities REQUIRED)
find_package(GTest REQUIRED)
set_property(TARGET kimera_pgmo::kimera_pgmo PROPERTY IMPORTED_GLOBAL TRUE)
add_library(kimera_pgmo ALIAS kimera_pgmo::kimera_pgmo)
enable_testing()
add_subdirectory("{test}" native_upstream_test)
add_executable(test_pgmo_initialized_fixture
  "{test}/pgmo_fixtures.cpp" test_deformation_graph_initialized_fixture.cpp)
target_include_directories(test_pgmo_initialized_fixture PRIVATE "{test}"
  ${{CMAKE_CURRENT_BINARY_DIR}}/native_upstream_test)
target_link_libraries(test_pgmo_initialized_fixture kimera_pgmo
  config_utilities::config_utilities GTest::gtest_main)
''')
    print(h)


if __name__ == "__main__":
    main()
