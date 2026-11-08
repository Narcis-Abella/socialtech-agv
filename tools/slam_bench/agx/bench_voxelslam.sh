#!/bin/bash
# IMAGE=<tag> overrides the image (default socialtech:mapper, which carries Voxel-SLAM). CYCLONEDDS_URI, if set in your shell, is passed to the container. DOMAIN=<id> sets the ROS domain (default 78); discovery is limited to localhost so runs on boards sharing a network never hear each other.
# NOTE: with VS_SED unset Voxel-SLAM runs with the port's default degrade_bound 10 (it resets on bags with featureless stretches); ops/make_map.sh uses 100000 and PLAY_RATE 2: for the map pipeline's
# numbers pass VS_SED=/tools/voxelslam_degrade.sed (see voxelslam_degrade.sed). PLAY_RATE defaults to 0.5 here.
# usage: bench_voxelslam.sh <name> <bag_dir under ~/rosbags> <lidar_topic> <imu_topic>   (own DDS domain 78)
cd ~
docker run --rm --name bench_voxelslam_$1 --cpus 5 --network host -e ROS_DOMAIN_ID=${DOMAIN:-78} -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e CYCLONEDDS_URI -e PLAY_RATE=${PLAY_RATE:-0.5} -e VS_SED=${VS_SED:-} -v ~/bench_ws:/ws -v ~/rosbags:/bags:ro -v ~/bench_out:/out -v ~/bench_tools:/tools:ro \
  ${IMAGE:-socialtech:mapper} /tools/voxelslam_run.sh /bags/$2 $3 $4 /out/voxelslam_$1
