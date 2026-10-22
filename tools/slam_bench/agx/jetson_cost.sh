#!/bin/bash
# Cost of Voxel-SLAM on one CustomMsg bag at a given playback rate, alone on the Jetson: tegrastats every 1 s and docker stats every 5 s.
# Needs ~/bench_voxelslam.sh, ~/bench_tools/voxelslam_run.sh and the compiled port in ~/bench_ws (same layout as the AGX). Any Jetson (AGX, NX, Nano).
# usage: jetson_cost.sh <name> <bag dir under ~/rosbags, e.g. custom/hospital_02_custom> <rate: 1.0 | 0.5> [lidar_topic] [imu_topic]   (default /livox/lidar /livox/imu)
cd ~
name=$1; bag=$2; rate=$3; lidar=${4:-/livox/lidar}; imu=${5:-/livox/imu}
out=~/bench_out; mkdir -p $out
if [ -n "$(docker ps -q)" ]; then echo "abort: a container is running" | tee -a $out/cost.log; exit 1; fi
if [ ! -d ~/rosbags/$bag ]; then echo "abort $name: no bag ~/rosbags/$bag" | tee -a $out/cost.log; exit 1; fi
if [ -e $out/voxelslam_$name ]; then echo "abort $name: $out/voxelslam_$name already exists (rename it: the node refuses to write into it)" | tee -a $out/cost.log; exit 1; fi
udp_drops() { awk '/^Udp:/ && !h {for (i = 1; i <= NF; i++) if ($i == "RcvbufErrors") c = i; h = 1; next} /^Udp:/ {print $c}' /proc/net/snmp; }   # datagrams the kernel dropped for a full receive buffer
u0=$(udp_drops)
tegrastats --interval 1000 > $out/mon_$name.log & tp=$!
( while true; do echo "$(date +%s) $(docker stats --no-stream --format '{{.CPUPerc}} {{.MemUsage}}' 2>/dev/null)" >> $out/dstats_$name.log; sleep 5; done ) & dp=$!
s=$(date +%s)
PLAY_RATE=$rate ~/bench_voxelslam.sh $name $bag $lidar $imu > $out/voxelslam_$name.log 2>&1
e=$(date +%s)
kill $tp $dp 2>/dev/null; wait $tp $dp 2>/dev/null
echo "$(date -Is) $name bag=$bag rate=$rate wall=$((e - s))s poses=$(wc -l < $out/voxelslam_$name/vs/run/alidarState.txt 2>/dev/null) udp_drops=$(( $(udp_drops) - u0 )) rmem_max=$(cat /proc/sys/net/core/rmem_max) model=$(tr -d '\0' < /proc/device-tree/model)" | tee -a $out/cost.log
