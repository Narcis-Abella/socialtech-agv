#!/bin/bash
# Voxel-SLAM on eco_-1_01 at 1x, alone on the AGX, with CPU / GPU / RAM logged each second (tegrastats) and per-container (docker stats) every 5 s.
# Experiment 1: does rotating the LiDAR->IMU extrinsic by the pooled IMU offset level the map? base = shipped mid360.yaml; rot_pos = offset (R_imu<-lidar = transpose of GLIM's T_lidar_imu); rot_neg = opposite sign (control: should get worse).
# usage: vs_queue1.sh variant...   (variant = base | a file ~/bench_tools/vs_ext/<variant>.sed)
cd ~
log=~/bench_out/queue.log
for v in "$@"; do
  if [ -n "$(docker ps -q)" ]; then echo "$(date -Is) abort $v: a container is running" >> $log; exit 1; fi
  name=eco01_x1_$v
  sedf=""; [ "$v" != base ] && sedf=/tools/vs_ext/$v.sed
  tegrastats --interval 1000 > ~/bench_out/mon_$name.log & tp=$!
  ( while true; do echo "$(date +%T) $(docker stats --no-stream --format '{{.CPUPerc}} {{.MemUsage}}' 2>/dev/null)" >> ~/bench_out/dstats_$name.log; sleep 5; done ) & dp=$!
  echo "$(date -Is) start voxelslam $name" >> $log
  VS_SED=$sedf PLAY_RATE=1.0 ~/bench_voxelslam.sh $name custom/mapping_eco_-1_01_custom /livox/lidar /livox/imu > ~/bench_out/voxelslam_$name.log 2>&1
  echo "$(date -Is) end voxelslam $name rc=$? poses=$(wc -l < ~/bench_out/voxelslam_$name/vs/run/alidarState.txt 2>/dev/null)" >> $log
  kill $tp $dp 2>/dev/null; wait $tp $dp 2>/dev/null
done
echo "$(date -Is) vs_queue1 done" >> $log
