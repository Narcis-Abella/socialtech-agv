#!/bin/bash
# Run Voxel-SLAM (ROS 2 port) on one Livox CustomMsg bag (the port's PointCloud2 mode expects per-point time relative to header.stamp, ours is absolute) inside the mapper image; the final poses are <out_dir>/vs/run/alidarState.txt.
# The poses are only written after the global stage, which is triggered by setting the `finish` parameter after playback.
# usage (in container): voxelslam_run.sh <bag_dir> <lidar_topic> <imu_topic> <out_dir>
# blind is 0.1 m (the Mid-360 blind zone; upstream 0.5): the near wall and door-jamb returns are kept. 2x replay gave identical poses (notes/mapeo_optimizacion_2026-10-30.md).
# env: PLAY_RATE (2: the node follows the data's timestamps, not the clock; poses were identical to 1x on the 366 s bag, N=1)   VS_SED (sed script applied to the generated config, one expression per line; voxelslam_degrade.sed raises degrade_bound)
set -eo pipefail
bag=$1; lidar=$2; imu=$3; out=$4
source /opt/ros/jazzy/setup.bash
export CMAKE_PREFIX_PATH=/opt/socialtech:$CMAKE_PREFIX_PATH AMENT_PREFIX_PATH=/opt/socialtech:$AMENT_PREFIX_PATH
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash   # optional overlay (the bench boards build Voxel-SLAM there); the mapper image has it under /opt/socialtech
mkdir -p "$out/vs"
rm -f "$out/vs/edge.txt"   # the end marker below: one of an earlier run would end the wait at once
cfg=$out/voxelslam.yaml
sed -e "s|lid_topic:.*|lid_topic: \"$lidar\"|" -e "s|imu_topic:.*|imu_topic: \"$imu\"|" \
    -e "s|imu_acc_unit_is_g:.*|imu_acc_unit_is_g: true|" -e "s|save_path:.*|save_path: \"$out/vs/\"|" \
    -e "s|bagname:.*|bagname: \"run\"|" -e "s|^\( *\)blind:.*|\1blind: 0.1|" -e "s|is_save_map:.*|is_save_map: 1|" \
    "$(ros2 pkg prefix voxel_slam)/share/voxel_slam/config/mid360.yaml" > "$cfg"
[ -n "${VS_SED:-}" ] && sed -i -f "$VS_SED" "$cfg"   # optional parameter overrides, one sed expression per line (experiments)
ros2 run voxel_slam voxelslam --ros-args --params-file "$cfg" -p finish:=false > "$out/node.log" 2>&1 &
node=$!
sleep 6   # the node creates its subscriptions
ros2 bag play "$bag" --rate "${PLAY_RATE:-2}" > "$out/play.log" 2>&1
sleep 5   # it drains its scan buffer: `finish` breaks its loop at once and would drop the last scans
ros2 param set /voxel_slam finish true > "$out/finish.log" 2>&1 || true
e=$out/vs/edge.txt   # the node writes it right after alidarState.txt is closed (the pose graph edges), so the poses are complete when it exists; the node never exits by itself
for _ in $(seq 1 1200); do [ -f "$e" ] && break; kill -0 $node 2>/dev/null || break; sleep 0.5; done   # up to 10 min for the global stage
kill -TERM $node 2>/dev/null || true
[ -f "$e" ] || { echo "voxelslam_run: no $e after the global stage (fewer than 100 poses, or Voxel-SLAM died): see $out/node.log" >&2; exit 1; }
wc -l "$out/vs/run/alidarState.txt"
