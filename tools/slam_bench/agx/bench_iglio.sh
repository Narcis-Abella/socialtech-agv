#!/bin/bash
# usage: bench_iglio.sh <name> <bag_dir_under_benchmarks> <lidar_topic> <imu_topic>
cd ~
docker run --rm --name bench_iglio_$1 --cpus 4 --network host -e ROS_DOMAIN_ID=77 -v ~/bench_ws:/ws -v ~/rosbags:/bags:ro -v ~/bench_out:/out -v ~/bench_tools:/tools:ro \
  socialtech:robot /tools/iglio_run.sh /bags/benchmarks/$2 $3 $4 /out/iglio_$1
