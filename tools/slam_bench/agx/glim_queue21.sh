#!/bin/bash
# Pooled IMU-vs-ceiling tilt correction (overlay tilt_pooled12.json, median over 12 bags, NOT leave-one-out) on the bags that start still,
# plus plain baselines (sub_kvox) for the still-start bags that never ran. Two streams on separate DDS domains (A=11, B=13), 6 cores each. 2 runs each.
# usage: glim_queue21.sh A|B
cd ~
log=~/glim_eval/queue.log
O=/tools/glim_eval
dom=$([ "$1" = A ] && echo 11 || echo 13)
run() {  # name bag overlay...
  name=$1; bag=$2; shift 2
  [ -d ~/glim_eval/$name ] && { echo "$(date -Is) skip $name (exists)" >> $log; return 0; }
  echo "$(date -Is) start $name" >> $log
  docker run --rm --name eval_$name --cpus 6 --network host -e ROS_DOMAIN_ID=$dom \
    -v ~/rosbags:/bags:ro -v ~/glim_eval:/eval -v ~/glim_eval_tools:/tools:ro socialtech:robot \
    /tools/glim_eval.sh /bags/$bag /eval/$name 2 $O/base.json "$@" >> $log 2>&1
  echo "$(date -Is) end $name rc=$?" >> $log
}
LAB=lab_test_1/rosbag2_2026_03_27-16_48_46
if [ "$1" = A ]; then
  run own_lab_test_1__sub_kvox $LAB $O/sub_kvox.json
  run own_trash_SEAT_1__sub_kvox trash_nx/SEAT_1 $O/sub_kvox.json
  run own_trash_SEAT_3__sub_kvox trash_nx/SEAT_3 $O/sub_kvox.json
  run own_trash_economics_2__sub_kvox trash_nx/planta_-1_economics_2 $O/sub_kvox.json
  run own_trash_economics_3__sub_kvox trash_nx/planta_-1_economics_3 $O/sub_kvox.json
  run eco01_kvox_tilt12 mapping_eco_-1_01 $O/sub_kvox.json $O/tilt_pooled12.json
  run own_mapping_zona_01__kvox_tilt12 mapping_zona_01 $O/sub_kvox.json $O/tilt_pooled12.json
else
  run own_lab_test_1__kvox_tilt12 $LAB $O/sub_kvox.json $O/tilt_pooled12.json
  run own_trash_SEAT_1__kvox_tilt12 trash_nx/SEAT_1 $O/sub_kvox.json $O/tilt_pooled12.json
  run own_trash_SEAT_3__kvox_tilt12 trash_nx/SEAT_3 $O/sub_kvox.json $O/tilt_pooled12.json
  run own_trash_economics_2__kvox_tilt12 trash_nx/planta_-1_economics_2 $O/sub_kvox.json $O/tilt_pooled12.json
  run own_trash_economics_3__kvox_tilt12 trash_nx/planta_-1_economics_3 $O/sub_kvox.json $O/tilt_pooled12.json
  run own_mapping_zona_05__kvox_tilt12 mapping_zona_05 $O/sub_kvox.json $O/tilt_pooled12.json
fi
echo "$(date -Is) queue21$1 done" >> $log
