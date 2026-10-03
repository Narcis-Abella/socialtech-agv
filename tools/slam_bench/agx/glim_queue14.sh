#!/bin/bash
# Degeneracy experiments on GLIM `base`: Livox tag filter (bags pre-filtered) and IMU noise sweep. Waits for the competitor queues.
cd ~
log=~/glim_eval/queue.log
until grep -q "end voxelslam tiers_office1_r05" ~/bench_out/queue.log 2>/dev/null; do sleep 15; done
O=/tools/glim_eval
run() {  # name bag topics overlay...
  name=$1; bag=$2; topics=$3; shift 3
  [ -d ~/glim_eval/$name ] && { echo "$(date -Is) skip $name (exists)" >> $log; return; }
  echo "$(date -Is) start $name" >> $log
  docker run --rm --name eval_$name --cpus 10 --network host -e ROS_DOMAIN_ID=11 \
    -v ~/rosbags:/bags:ro -v ~/glim_eval:/eval -v ~/glim_eval_tools:/tools:ro socialtech:robot \
    /tools/glim_eval.sh /bags/benchmarks/$bag /eval/$name 3 $O/base.json $O/$topics.json "$@" >> $log 2>&1
  echo "$(date -Is) end $name rc=$?" >> $log
}
for mode in low high; do
  run tiers_IndoorOffice2__tag$mode tiers_IndoorOffice2_tag$mode tiers_topics
  run m3dgr_Corridor01__tag$mode m3dgr_Corridor01_tag$mode m3dgr_topics
  run m3dgr_Elevator01__tag$mode m3dgr_Elevator01_tag$mode m3dgr_topics
done
for imu in imu_d3 imu_d10 imu_d30 imu_x3; do
  run m3dgr_Corridor01__$imu m3dgr_Corridor01 m3dgr_topics $O/$imu.json
  run m3dgr_Elevator01__$imu m3dgr_Elevator01 m3dgr_topics $O/$imu.json
done
run m3dgr_Dynamic01__imu_d30 m3dgr_Dynamic01 m3dgr_topics $O/imu_d30.json
echo "$(date -Is) queue14 done" >> $log
