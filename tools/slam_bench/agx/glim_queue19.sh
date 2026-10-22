#!/bin/bash
# Does rotating T_lidar_imu by the measured IMU-gravity vs floor-normal offset (0.51 deg, see notes/slam_seleccion.md) level the map?
# tiltfix = correction, tiltfix_neg = opposite sign (negative control: must make the tilt worse). On top of sub_kvox (dense submaps for floor_tilt.py).
# DDS domain 13 so it can run next to queue18 (domain 11). 3 runs each, 6 cores.
cd ~
log=~/glim_eval/queue.log
O=/tools/glim_eval
run() {  # name bag overlay...
  name=$1; bag=$2; shift 2
  [ -d ~/glim_eval/$name ] && { echo "$(date -Is) skip $name (exists)" >> $log; return 0; }
  echo "$(date -Is) start $name" >> $log
  docker run --rm --name eval_$name --cpus 6 --network host -e ROS_DOMAIN_ID=13 \
    -v ~/rosbags:/bags:ro -v ~/glim_eval:/eval -v ~/glim_eval_tools:/tools:ro socialtech:robot \
    /tools/glim_eval.sh /bags/$bag /eval/$name 3 $O/base.json "$@" >> $log 2>&1
  echo "$(date -Is) end $name rc=$?" >> $log
}
run eco01_kvox_tiltfix mapping_eco_-1_01 $O/sub_kvox.json $O/tilt_fix.json
run eco01_kvox_tiltfix_neg mapping_eco_-1_01 $O/sub_kvox.json $O/tilt_fix_neg.json
run own_mapping_eco_-1_03__kvox_tiltfix mapping_eco_-1_03 $O/sub_kvox.json $O/tilt_fix.json
run own_hospital_02__kvox_tiltfix hospital_02 $O/sub_kvox.json $O/tilt_fix.json
echo "$(date -Is) queue19 done" >> $log
