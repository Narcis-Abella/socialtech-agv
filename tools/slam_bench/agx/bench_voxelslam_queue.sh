#!/bin/bash
# Voxel-SLAM (CustomMsg bags) on the remaining benchmark sequences, one run each, real-time playback. Waits for the iG-LIO queue.
cd ~
log=~/bench_out/queue.log
until grep -q "iglio queue done" $log 2>/dev/null; do sleep 10; done
run() {
  echo "$(date -Is) start voxelslam $1" >> $log
  ~/bench_voxelslam.sh "$1" "$2" "$3" "$4" > ~/bench_out/voxelslam_$1.log 2>&1
  echo "$(date -Is) end voxelslam $1 rc=$? poses=$(wc -l < ~/bench_out/voxelslam_$1/vs/run/alidarState.txt 2>/dev/null)" >> $log
}
run tiers_office2 benchmarks/tiers_IndoorOffice2_custom /mid360/livox/lidar /mid360/livox/imu
for s in Corridor01 Dynamic01 Sha-turn01 Elevator01; do run m3dgr_$s benchmarks/m3dgr_${s}_custom /livox/mid360/lidar /livox/mid360/imu; done
echo "$(date -Is) voxelslam queue done" >> $log
