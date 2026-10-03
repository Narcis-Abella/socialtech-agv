#!/bin/bash
# 1) Voxel-SLAM repeats (runs 2-5, 0.5x) on the two degenerate M3DGR sequences -> run-to-run variance.
# 2) FAST-LIO2 and Voxel-SLAM on mapping_real_01 (72 s LiDAR gap that aborts GLIM); IDLE=120 keeps the FAST-LIO2 recorder alive over the gap.
# 3) Voxel-SLAM cost at 1x on the (otherwise idle) AGX: CPU/memory sampled every 5 s with docker stats.
# Waits for nothing: run only when the AGX is idle (the cost measurement is meaningless otherwise).
cd ~
log=~/bench_out/queue.log
run_vs() {  # name bag lidar imu [PLAY_RATE]
  echo "$(date -Is) start voxelslam $1" >> $log
  PLAY_RATE=${5:-0.5} ~/bench_voxelslam.sh "$1" "$2" "$3" "$4" > ~/bench_out/voxelslam_$1.log 2>&1
  echo "$(date -Is) end voxelslam $1 rc=$? poses=$(cat ~/bench_out/voxelslam_$1/vs/*/alidarState.txt 2>/dev/null | wc -l)" >> $log
}
M3=(/livox/mid360/lidar /livox/mid360/imu)
for n in 2 3 4 5; do
  for s in Corridor01 Elevator01; do run_vs rep${n}_m3dgr_$s benchmarks/m3dgr_${s}_custom "${M3[@]}"; done
done
echo "$(date -Is) start fastlio r05_own_mapping_real_01" >> $log
IDLE=120 ~/bench_fastlio.sh r05_own_mapping_real_01 ../custom/mapping_real_01_custom /livox/lidar /livox/imu > ~/bench_out/fastlio_r05_own_mapping_real_01.log 2>&1
echo "$(date -Is) end fastlio r05_own_mapping_real_01 rc=$? poses=$(wc -l < ~/bench_out/fastlio_r05_own_mapping_real_01/odom.tum)" >> $log
run_vs own_mapping_real_01 custom/mapping_real_01_custom /livox/lidar /livox/imu
for s in Corridor01 Elevator01; do
  n=cost1x_m3dgr_$s
  ( while sleep 5; do docker stats --no-stream --format "$(date +%s) {{.CPUPerc}} {{.MemUsage}}" bench_voxelslam_$n 2>/dev/null; done > ~/bench_out/voxelslam_$n.stats ) &
  sampler=$!
  start=$(date +%s)
  run_vs $n benchmarks/m3dgr_${s}_custom "${M3[@]}" 1.0
  kill $sampler 2>/dev/null
  echo "$(date -Is) wall $n $(( $(date +%s) - start )) s" >> $log
done
echo "$(date -Is) reps queue done" >> $log
