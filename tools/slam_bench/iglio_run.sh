#!/bin/bash
# Run iG-LIO (ROS 2 Jazzy port) on one PointCloud2 bag inside the robot image; /lio_odom goes to <out_dir>/odom.tum.
# usage (in container): iglio_run.sh <bag_dir> <lidar_topic> <imu_topic> <out_dir>
set -eo pipefail
bag=$1; lidar=$2; imu=$3; out=$4
source /opt/ros/jazzy/setup.bash
export CMAKE_PREFIX_PATH=/opt/socialtech:$CMAKE_PREFIX_PATH AMENT_PREFIX_PATH=/opt/socialtech:$AMENT_PREFIX_PATH
source /ws/install/setup.bash
mkdir -p "$out"
cfg=$out/iglio.yaml
sed -e "s|lidar_topic:.*|lidar_topic: \"$lidar\"|" -e "s|imu_topic:.*|imu_topic: \"$imu\"|" \
    -e "s|result_directory:.*|result_directory: \"$out\"|" -e "s|qos_reliability:.*|qos_reliability: \"${IGLIO_QOS:-best_effort}\"|" "$(ros2 pkg prefix ig_lio)/share/ig_lio/config/mid360.yaml" > "$cfg"
ros2 run ig_lio ig_lio_node --ros-args --params-file "$cfg" > "$out/node.log" 2>&1 &
node=$!
python3 /tools/odom_to_tum.py /lio_odom "$out/odom.tum" --idle 10 --max-wait 90 > "$out/rec.log" 2>&1 &
rec=$!
sleep 4
ros2 bag play "$bag" --rate "${PLAY_RATE:-1.0}" > "$out/play.log" 2>&1
wait $rec
kill -TERM $node 2>/dev/null || true
wc -l "$out/odom.tum"
