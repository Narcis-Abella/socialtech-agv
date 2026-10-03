#!/bin/bash
# usage: bench_fastlio.sh <name> <bag_dir_under_benchmarks> <lidar_topic> <imu_topic>
cd ~
docker run --rm --name bench_fastlio_$1 --cpus 4 --network host -e ROS_DOMAIN_ID=77 -e PLAY_RATE=${PLAY_RATE:-0.5} -e IDLE=${IDLE:-10} -v ~/bench_ws:/ws -v ~/rosbags:/bags:ro -v ~/bench_out:/out -v ~/bench_tools:/tools:ro \
  socialtech:robot /tools/fastlio_run.sh /bags/benchmarks/$2 $3 $4 /out/fastlio_$1
