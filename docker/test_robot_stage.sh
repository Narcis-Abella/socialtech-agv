#!/bin/bash
# Run: bash docker/test_robot_stage.sh
# ops/nav_live.sh SUPERVISOR=1 imports numpy and scipy in the robot image (the relocalizer worker needs them); scipy is not part of ROS and the robot stage once lost it.
here=$(cd "$(dirname "$0")" && pwd)
stage=$(awk '/^FROM .* AS robot$/ {g = 1; next} /^FROM / {g = 0} g' "$here/Dockerfile")
fail=0
grep -q 'python3-scipy' <<< "$stage" || { echo "FAIL: the robot stage of the Dockerfile does not install python3-scipy"; fail=1; }
grep -q 'import numpy, scipy' "$here/../ops/nav_live.sh" || { echo "FAIL: nav_live.sh no longer checks numpy and scipy (update this test)"; fail=1; }
# the Livox SDK is linked with the spdlog-hiding version script and the build checks it (without it livox_ros_driver2_node segfaults at exit in about 1 of 10 shutdowns)
grep -q -- '-Wl,--version-script=/opt/livox-sdk-build/livox_sdk_hide_spdlog.map' <<< "$stage" || { echo "FAIL: the Livox SDK build no longer passes the spdlog-hiding version script"; fail=1; }
grep -q 'bash /opt/livox-sdk-build/test_livox_sdk_symbols.sh' <<< "$stage" || { echo "FAIL: the Livox SDK build no longer runs test_livox_sdk_symbols.sh"; fail=1; }
((fail == 0)) && echo "ok"
exit $fail
