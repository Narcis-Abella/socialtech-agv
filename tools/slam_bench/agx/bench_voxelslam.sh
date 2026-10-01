#!/bin/bash
# usage: bench_voxelslam.sh <name> <bag_dir under ~/rosbags> <lidar_topic> <imu_topic>   (own DDS domain 78)
cd ~
docker run --rm --name bench_voxelslam_$1 --cpus 5 --network host -e ROS_DOMAIN_ID=78 -e PLAY_RATE=${PLAY_RATE:-0.5} -e IDLE=${IDLE:-10} -e VS_SED=${VS_SED:-} -v ~/bench_ws:/ws -v ~/rosbags:/bags:ro -v ~/bench_out:/out -v ~/bench_tools:/tools:ro \
  socialtech:robot /tools/voxelslam_run.sh /bags/$2 $3 $4 /out/voxelslam_$1
