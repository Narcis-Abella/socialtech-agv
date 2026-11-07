#!/bin/bash
# usage (inside the robot image, with the package under test sourced): run_band_test.sh stock|band    one costmap node + band_test.py; exit code of the test
here=$(dirname "$(readlink -f "$0")"); source /ros_env.sh
[ -f /ov/install/setup.bash ] && source /ov/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-97}; export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
mode=$1
if [ "$mode" = band ]; then band="height_band_clearing: true"; exp=1; else band=""; exp=0; fi
sed -e "s|@BAND@|$band|" "$here/costmap_params.yaml.tpl" > /tmp/costmap_$mode.yaml
ros2 run nav2_costmap_2d nav2_costmap_2d --ros-args --params-file /tmp/costmap_$mode.yaml > /tmp/costmap_$mode.log 2>&1 & node=$!
sleep 3
echo "=== $mode"
python3 "$here/band_test.py" $exp; rc=$?
kill -TERM $node 2>/dev/null; wait $node 2>/dev/null; grep -iE "error|fail|exception" /tmp/costmap_$mode.log | head -3
exit $rc
