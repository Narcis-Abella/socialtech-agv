#!/bin/bash
# Voxel-SLAM on eco_-1_01 at 1x, alone on the AGX, with CPU / GPU / RAM logged each second (tegrastats) and per-container (docker stats) every 5 s.
# usage: [VS_SED=<sed file on the container's /tools>] vs_queue1.sh run_name...   (one run per name; VS_SED, if set, edits the Voxel-SLAM parameters of every run)
cd ~
log=~/bench_out/queue.log
for v in "$@"; do
  if [ -n "$(docker ps -q)" ]; then echo "$(date -Is) abort $v: a container is running" >> $log; exit 1; fi
  name=eco01_x1_$v

  tegrastats --interval 1000 > ~/bench_out/mon_$name.log & tp=$!
  ( while true; do echo "$(date +%T) $(docker stats --no-stream --format '{{.CPUPerc}} {{.MemUsage}}' 2>/dev/null)" >> ~/bench_out/dstats_$name.log; sleep 5; done ) & dp=$!
  echo "$(date -Is) start voxelslam $name" >> $log
  PLAY_RATE=1.0 ~/bench_voxelslam.sh $name custom/mapping_eco_-1_01_custom /livox/lidar /livox/imu > ~/bench_out/voxelslam_$name.log 2>&1
  echo "$(date -Is) end voxelslam $name rc=$? poses=$(wc -l < ~/bench_out/voxelslam_$name/vs/run/alidarState.txt 2>/dev/null)" >> $log
  kill $tp $dp 2>/dev/null; wait $tp $dp 2>/dev/null
done
echo "$(date -Is) vs_queue1 done" >> $log
