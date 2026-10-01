#!/bin/bash
# IMAGE=<tag> overrides the image (default socialtech:robot). CYCLONEDDS_URI, if set in your shell, is passed to the container. DOMAIN=<id> sets the ROS domain (default 78); discovery is limited to localhost so runs on boards sharing a network never hear each other.
# usage: bench_voxelslam.sh <name> <bag_dir under ~/rosbags> <lidar_topic> <imu_topic>   (own DDS domain 78)
cd ~
docker run --rm --name bench_voxelslam_$1 --cpus 5 --network host -e ROS_DOMAIN_ID=${DOMAIN:-78} -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e CYCLONEDDS_URI -e PLAY_RATE=${PLAY_RATE:-0.5} -e IDLE=${IDLE:-10} -e VS_SED=${VS_SED:-} -v ~/bench_ws:/ws -v ~/rosbags:/bags:ro -v ~/bench_out:/out -v ~/bench_tools:/tools:ro \
  ${IMAGE:-socialtech:robot} /tools/voxelslam_run.sh /bags/$2 $3 $4 /out/voxelslam_$1
