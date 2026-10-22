#!/bin/bash
# Leave-one-bag-out test of the IMU-vs-floor tilt correction (tools/slam_bench/static_offset.py): each bag runs with T_lidar_imu rotated by the
# CEILING offset pooled over the OTHER bags (overlay tilt_loo_<bag>.json). Baselines (sub_kvox, no correction) for the bags that lack one.
# Waits for queue19 (DDS domain 13 shared). 2 runs each, 6 cores.
cd ~
log=~/glim_eval/queue.log
until grep -q "queue19 done" $log; do sleep 15; done
O=/tools/glim_eval
run() {  # name bag overlay...
  name=$1; bag=$2; shift 2
  [ -d ~/glim_eval/$name ] && { echo "$(date -Is) skip $name (exists)" >> $log; return 0; }
  echo "$(date -Is) start $name" >> $log
  docker run --rm --name eval_$name --cpus 6 --network host -e ROS_DOMAIN_ID=13 \
    -v ~/rosbags:/bags:ro -v ~/glim_eval:/eval -v ~/glim_eval_tools:/tools:ro socialtech:robot \
    /tools/glim_eval.sh /bags/$bag /eval/$name 2 $O/base.json "$@" >> $log 2>&1
  echo "$(date -Is) end $name rc=$?" >> $log
}
NEW="mapping_seat_01 mapping_zona_01 mapping_zona_04 mapping_zona_05 mapping_zona_06"
for b in $NEW; do run own_${b}__sub_kvox $b $O/sub_kvox.json; done
for b in $NEW mapping_eco_-1_03 hospital_02; do run own_${b}__kvox_tiltloo $b $O/sub_kvox.json $O/tilt_loo_$b.json; done
run eco01_kvox_tiltloo mapping_eco_-1_01 $O/sub_kvox.json $O/tilt_loo_mapping_eco_-1_01.json
echo "$(date -Is) queue20 done" >> $log
