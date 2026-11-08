#!/bin/bash
# The C++ gtests that need only Eigen and GoogleTest (header-only cores), compiled with g++ and run: nothing in the image builds tests (BUILD_TESTING=OFF), so this is their routine runner.
#   docker/run_cpp_tests.sh        needs: g++, libgtest-dev, libeigen3-dev (a laptop, or the robot image after apt-get install libgtest-dev)
# Not covered here (they need small_gicp, ROS or the built package): level_corrector test_registration, the node tests, socialtech_mapping, socialtech_simreal, socialtech_costmap.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
out=$(mktemp -d /tmp/cpp_tests.XXXXXX)
trap 'rm -r "$out"' EXIT
run() {   # run SOURCE INCLUDE_DIR
  local name; name=$(basename "$1" .cpp)
  g++ -std=c++17 -O1 -I "$here/$2" -I /usr/include/eigen3 "$here/$1" -lgtest -lgtest_main -pthread -o "$out/$name"
  echo "== $name"; "$out/$name" | grep -E '^\[  (PASSED|FAILED)'
}
run ../src/localization/gn_loc/test/test_gn_core.cpp ../src/localization/gn_loc/include
run ../src/localization/level_corrector/test/test_core.cpp ../src/localization/level_corrector/include
run ../src/localization/level_corrector/test/test_map_io.cpp ../src/localization/level_corrector/include
