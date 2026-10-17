#!/bin/bash
# Run FAST-LIO2 (ROS 2 port) on one CustomMsg bag inside the robot image; /Odometry goes to <out_dir>/odom.tum.
# usage (in container): fastlio_run.sh <bag_dir> <lidar_topic> <imu_topic> <out_dir>
set -eo pipefail
bag=$1; lidar=$2; imu=$3; out=$4
source /opt/ros/jazzy/setup.bash
export CMAKE_PREFIX_PATH=/opt/socialtech:$CMAKE_PREFIX_PATH AMENT_PREFIX_PATH=/opt/socialtech:$AMENT_PREFIX_PATH
source /ws/install/setup.bash
mkdir -p "$out"
cfg=$out/fastlio.yaml
sed -e "s|lid_topic:.*|lid_topic: \"$lidar\"|" -e "s|imu_topic:.*|imu_topic: \"$imu\"|" \
    -e "s|pcd_save_en:.*|pcd_save_en: false|" -e "s|map_en:.*|map_en: false|" \
    -e "s|scan_publish_en:.*|scan_publish_en: false|" -e "s|scan_bodyframe_pub_en:.*|scan_bodyframe_pub_en: false|" \
    -e "s|path_en:.*|path_en: false|" "$(ros2 pkg prefix fast_lio)/share/fast_lio/config/mid360.yaml" > "$cfg"
ros2 run fast_lio fastlio_mapping --ros-args --params-file "$cfg" > "$out/node.log" 2>&1 &
node=$!
python3 /tools/odom_to_tum.py /Odometry "$out/odom.tum" --idle "${IDLE:-10}" --max-wait 90 > "$out/rec.log" 2>&1 &
rec=$!
sleep 4
ros2 bag play "$bag" --rate "${PLAY_RATE:-1.0}" > "$out/play.log" 2>&1
wait $rec            # the recorder ends itself after the odometry goes silent
kill -TERM $node 2>/dev/null || true
wc -l "$out/odom.tum"
