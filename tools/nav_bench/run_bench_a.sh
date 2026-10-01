#!/bin/bash
# Stage A of the nav bench: replay one CustomMsg bag over its own map with FAST-LIO2 odometry + AMCL, then report the errors against the Voxel-SLAM
# trajectory of the same bag (pose_eval.py). Run inside the robot image (see README.md).
# usage: run_bench_a.sh <bag_dir> <map.yaml> <alidarState.txt> <sensor_height_m> <out_dir>
# env:   PLAY_RATE (1.0)  DIST (1.0 m) and DYAW (20 deg): how far the initial pose is pushed  REF_DT (0 s)  PLAY_ARGS (e.g. "--playback-duration 60")
#        LIDAR (/livox/lidar)  IMU (/livox/imu)  ROS_DOMAIN_ID (79: GLIM 11, FAST-LIO2/iG-LIO 77, Voxel-SLAM 78)
set -eo pipefail
bag=$1; map=$2; ref=$3; h=$4; out=$5
tools=$(dirname "$(readlink -f "$0")")
source /ros_env.sh
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-79}
mkdir -p "$out"

sed -e "s|lid_topic:.*|lid_topic: \"${LIDAR:-/livox/lidar}\"|" -e "s|imu_topic:.*|imu_topic: \"${IMU:-/livox/imu}\"|" \
    -e "s|pcd_save_en:.*|pcd_save_en: false|" -e "s|map_en:.*|map_en: false|" -e "s|path_en:.*|path_en: false|" \
    "$(ros2 pkg prefix fast_lio)/share/fast_lio/config/mid360.yaml" > "$out/fastlio.yaml"
read -r x y yaw < <(python3 "$tools/pose_eval.py" init --ref "$ref" --map "$map" --dist "${DIST:-1.0}" --dyaw "${DYAW:-20}")
read -r qx qy qz qw < <(python3 "$tools/pose_eval.py" level --ref "$ref" --map "$map")
echo "initial pose (map frame): $x $y $yaw; level quaternion: $qx $qy $qz $qw" | tee "$out/initial_pose.txt"

ros2 launch "$tools/launch/bench_a.launch.py" fastlio_params:="$out/fastlio.yaml" map:="$map" init_x:="$x" init_y:="$y" init_yaw:="$yaw" \
    sensor_height:="$h" level_qx:="$qx" level_qy:="$qy" level_qz:="$qz" level_qw:="$qw" > "$out/launch.log" 2>&1 &
launch=$!
python3 "$tools/record_poses.py" /amcl_pose amcl "$out/amcl.tum" > "$out/rec_amcl.log" 2>&1 & rec_a=$!
python3 "$tools/record_poses.py" /Odometry odom "$out/odom.tum" > "$out/rec_odom.log" 2>&1 & rec_o=$!
sleep 15   # lifecycle bring-up (map_server, AMCL) before the first scan
# shellcheck disable=SC2086
ros2 bag play "$bag" --clock --rate "${PLAY_RATE:-1.0}" ${PLAY_ARGS:-} > "$out/play.log" 2>&1
wait "$rec_a" "$rec_o" || true   # each recorder ends itself after 15 s without messages
kill -INT "$launch" 2>/dev/null || true
wait "$launch" 2>/dev/null || true

[ -s "$out/amcl.tum" ] || { echo "FAIL: AMCL published no pose (see $out/launch.log)"; exit 1; }
python3 "$tools/pose_eval.py" report --est "$out/amcl.tum" --odom "$out/odom.tum" --ref "$ref" --map "$map" --ref-dt "${REF_DT:-0}" | tee "$out/report.json"
