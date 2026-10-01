#!/bin/bash
# The Voxel-SLAM cost queue on one Jetson, one bag at a time with nothing else running: a small smoke bag, then the three most demanding (the longest, the one with loop closures, the densest).
# Run it on the AGX, NX and Nano with the same rate so the numbers are comparable; if a board falls behind at 1.0, repeat it at 0.5.
# usage: [TAG=name] jetson_cost_queue.sh [rate=1.0] [image=socialtech:robot-gtsam]      logs: ~/bench_out/cost.log, mon_*/dstats_*/voxelslam_* per run (summarise with cost_summary.py)
rate=${1:-1.0}; export IMAGE=${2:-socialtech:robot-gtsam}
tag=${TAG:-r${rate/./p}}   # TAG=fix keeps a repeat from colliding with the folders of the first pass (the node refuses an existing one)
cd ~
~/jetson_cost.sh smoke_$tag custom/mapping_eco_-1_03_custom $rate
~/jetson_cost.sh hospital02_$tag custom/hospital_02_custom $rate
~/jetson_cost.sh eco01_$tag custom/mapping_eco_-1_01_custom $rate
~/jetson_cost.sh corridor01_$tag benchmarks/m3dgr_Corridor01_custom $rate /livox/mid360/lidar /livox/mid360/imu
echo "$(date -Is) cost queue $tag done" >> ~/bench_out/cost.log
