#!/bin/bash
# How aggressive can GLIM's statistical outlier removal be on eco01 before it eats real structure? Overlays stack on sub_kvox. Two streams (DDS 11 / 13), 6 cores each, 2 runs each.
# pre_o20_03: k=20, 0.3 sigma; pre_o30_02: k=30, 0.2; pre_o50_01: k=50, 0.1; pre_o50_005: k=50, 0.05 (already done: k=10/1.0 pre_outlier, k=20/0.5 pre_outlier_strict).
# usage: glim_queue24.sh A|B
cd ~
log=~/glim_eval/queue.log
O=/tools/glim_eval
dom=$([ "$1" = A ] && echo 11 || echo 13)
run() {  # variant
  name=eco01_$1__sub_kvox
  [ -d ~/glim_eval/$name ] && { echo "$(date -Is) skip $name (exists)" >> $log; return 0; }
  echo "$(date -Is) start $name" >> $log
  docker run --rm --name eval_$name --cpus 6 --network host -e ROS_DOMAIN_ID=$dom \
    -v ~/rosbags:/bags:ro -v ~/glim_eval:/eval -v ~/glim_eval_tools:/tools:ro socialtech:robot \
    /tools/glim_eval.sh /bags/mapping_eco_-1_01 /eval/$name 2 $O/base.json $O/sub_kvox.json $O/$1.json >> $log 2>&1
  echo "$(date -Is) end $name rc=$?" >> $log
}
if [ "$1" = A ]; then run pre_o20_03; run pre_o50_01; else run pre_o30_02; run pre_o50_005; fi
echo "$(date -Is) queue24$1 done" >> $log
