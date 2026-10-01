#!/bin/bash
# Voxel-SLAM at several playback rates on one bag, one run after another and alone on the board (jetson_cost.sh does each run).
# Above the rate a board sustains, scans are lost without any warning (the node's input queues are bounded), so judge each run by its poses against the bag's
# scans, the UDP drops in cost.log and the trajectory against the 1x run (traj_compare.py), not by whether the node survives.
# usage: [IMAGE=tag] jetson_sweep.sh <name prefix> <bag dir under ~/rosbags> <rate> [rate...]   (CYCLONEDDS_URI and DOMAIN from the shell, as for jetson_cost_queue.sh)
prefix=$1; bag=$2; shift 2
export IMAGE=${IMAGE:-socialtech:robot-gtsam}
for r in "$@"; do ~/jetson_cost.sh ${prefix}_x${r/./p} $bag $r; done
echo "$(date -Is) sweep $prefix done" >> ~/bench_out/cost.log
