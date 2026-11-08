#!/bin/bash
# Every test that needs neither ROS 2 nor a board: what CI runs. Run from anywhere: scripts/test.sh
# Tests that need the robot image (rclpy, nav2_msgs, fast_lio, a running stack) are left out of the pytest collection by the conftest.py next to them
# and run inside the image (see the README of each package).
#   needs: python3 with pytest numpy scipy pyyaml pillow; g++, libgtest-dev, libeigen3-dev (docker/run_cpp_tests.sh)
set -euo pipefail
cd "$(dirname "$0")/.."
python3 -m pytest -q
docker/run_cpp_tests.sh
docker/test_cyclone_uri.sh
docker/test_robot_stage.sh
src/bringup/socialtech_bringup/test/test_flags.sh
src/bringup/socialtech_bringup/test/test_fastlio_check.sh
ops/test_image_revision.sh
