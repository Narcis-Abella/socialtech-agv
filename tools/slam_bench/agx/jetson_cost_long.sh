#!/bin/bash
# The longest bags we have, for RAM and time. hospital_02 (634 s) at the highest rate that kept the smoke bag intact (3x), Elevator01 (700 s, the longest valid one; the lift makes
# Voxel-SLAM split it into sessions) at 1x and 3x, and real_01 (1456 s) at 1x: its 73 s lidar gap makes Voxel-SLAM restart, so its map is not valid, only its resource use is.
# usage: jetson_cost_long.sh      (CYCLONEDDS_URI and DOMAIN from the shell, as for jetson_cost_queue.sh)
export IMAGE=${IMAGE:-socialtech:robot-gtsam}
cd ~
~/jetson_cost.sh hospital02_x3 custom/hospital_02_custom 3
~/jetson_cost.sh elevator01_fix benchmarks/m3dgr_Elevator01_custom 1.0 /livox/mid360/lidar /livox/mid360/imu
~/jetson_cost.sh elevator01_x3 benchmarks/m3dgr_Elevator01_custom 3 /livox/mid360/lidar /livox/mid360/imu
~/jetson_cost.sh real01_fix custom/mapping_real_01_custom 1.0
echo "$(date -Is) long queue done" >> ~/bench_out/cost.log
