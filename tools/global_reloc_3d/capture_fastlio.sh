#!/usr/bin/env bash
# Run in socialtech:robot-nav. No reference/map/initialpose is mounted.
set -eo pipefail
bag=$1
offset=$2
duration=$3
out=$4
source /ros_env.sh
set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:?set a unique ROS_DOMAIN_ID}
mkdir -p "$out"
config=$(ros2 pkg prefix fast_lio)/share/fast_lio/config/mid360.yaml
sed -e 's|pcd_save_en:.*|pcd_save_en: false|' \
    -e 's|map_en:.*|map_en: false|' \
    -e 's|path_en:.*|path_en: false|' \
    -e 's|^\( *\)gyr_cov:.*|\1gyr_cov: 1.0|' \
    -e 's|^\( *\)acc_cov:.*|\1acc_cov: 1.0|' \
    "$config" > "$out/fastlio.yaml"
ros2 run fast_lio fastlio_mapping --ros-args --params-file "$out/fastlio.yaml" -p use_sim_time:=true > "$out/fastlio.log" 2>&1 &
lio=$!
python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
  ros2 bag record -s mcap -o "$out/lio_bag" /Odometry /cloud_registered_body > "$out/record.log" 2>&1 &
record=$!
cleanup() {
  kill -INT "$record" 2>/dev/null || true
  wait "$record" 2>/dev/null || true
  kill -TERM "$lio" 2>/dev/null || true
  wait "$lio" 2>/dev/null || true
}
trap cleanup EXIT
sleep 5
ros2 bag play "$bag" --clock --rate 1.0 --start-offset "$offset" --playback-duration "$duration" > "$out/play.log" 2>&1
sleep 2
