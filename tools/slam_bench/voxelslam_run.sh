#!/bin/bash
# Run Voxel-SLAM (ROS 2 port) on one Livox CustomMsg bag (the port's PointCloud2 mode expects per-point time relative to header.stamp, ours is absolute) inside the robot image; the final poses are <out_dir>/vs/run/alidarState.txt.
# The poses are only written after the global stage, which is triggered by setting the `finish` parameter after playback.
# usage (in container): voxelslam_run.sh <bag_dir> <lidar_topic> <imu_topic> <out_dir>
set -eo pipefail
bag=$1; lidar=$2; imu=$3; out=$4
source /opt/ros/jazzy/setup.bash
export CMAKE_PREFIX_PATH=/opt/socialtech:$CMAKE_PREFIX_PATH AMENT_PREFIX_PATH=/opt/socialtech:$AMENT_PREFIX_PATH
source /ws/install/setup.bash
mkdir -p "$out/vs"
cfg=$out/voxelslam.yaml
sed -e "s|lid_topic:.*|lid_topic: \"$lidar\"|" -e "s|imu_topic:.*|imu_topic: \"$imu\"|" \
    -e "s|imu_acc_unit_is_g:.*|imu_acc_unit_is_g: true|" -e "s|save_path:.*|save_path: \"$out/vs/\"|" \
    -e "s|bagname:.*|bagname: \"run\"|" -e "s|is_save_map:.*|is_save_map: 1|" \
    "$(ros2 pkg prefix voxel_slam)/share/voxel_slam/config/mid360.yaml" > "$cfg"
[ -n "${VS_SED:-}" ] && sed -i -f "$VS_SED" "$cfg"   # optional parameter overrides, one sed expression per line (experiments)
ros2 run voxel_slam voxelslam --ros-args --params-file "$cfg" -p finish:=false > "$out/node.log" 2>&1 &
node=$!
sleep 6
ros2 bag play "$bag" --rate "${PLAY_RATE:-1.0}" > "$out/play.log" 2>&1
sleep 5
ros2 param set /voxel_slam finish true > "$out/finish.log" 2>&1 || true
f=$out/vs/run/alidarState.txt   # written once, at the end of the global stage
for _ in $(seq 1 120); do kill -0 $node 2>/dev/null || break; [ -f "$f" ] && [ $(( $(date +%s) - $(stat -c %Y "$f") )) -gt 20 ] && break; sleep 5; done   # up to 10 min for the global stage
kill -TERM $node 2>/dev/null || true
wc -l "$out/vs/run/alidarState.txt"
