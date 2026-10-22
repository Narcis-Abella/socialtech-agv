#!/bin/bash
# FAST-LIO2 then Voxel-SLAM (CustomMsg bags in ~/rosbags/custom) on our own bags, 0.5x, one run each. Waits for queue14.
cd ~
until grep -q "queue14 done" ~/glim_eval/queue.log; do sleep 15; done
log=~/bench_out/queue.log
for bag in mapping_eco_-1_01 mapping_eco_-1_03 hospital_02; do
  echo "$(date -Is) start fastlio r05_own_$bag" >> $log
  ~/bench_fastlio.sh r05_own_$bag ../custom/${bag}_custom /livox/lidar /livox/imu > ~/bench_out/fastlio_r05_own_$bag.log 2>&1
  echo "$(date -Is) end fastlio r05_own_$bag rc=$? poses=$(wc -l < ~/bench_out/fastlio_r05_own_$bag/odom.tum)" >> $log
done
for bag in mapping_eco_-1_01 mapping_eco_-1_03 hospital_02; do
  echo "$(date -Is) start voxelslam own_$bag" >> $log
  ~/bench_voxelslam.sh own_$bag custom/${bag}_custom /livox/lidar /livox/imu > ~/bench_out/voxelslam_own_$bag.log 2>&1
  echo "$(date -Is) end voxelslam own_$bag rc=$? poses=$(cat ~/bench_out/voxelslam_own_$bag/vs/*/alidarState.txt 2>/dev/null | wc -l)" >> $log
done
echo "$(date -Is) own queue done" >> $log
