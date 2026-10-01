#!/bin/bash
# Do GLIM's own preprocessing filters reduce the isolated spurious pixels of the eco01 map? Overlays stack on sub_kvox. Two streams (DDS 11 / 13), 6 cores each, 2 runs each.
# pre_outlier: statistical outlier removal on (k=10, 1 sigma); pre_outlier_strict: k=20, 0.5 sigma; pre_near1/2: drop points closer than 1 / 2 m (default 0.5); sub_ds02: saved submap points 0.2 m (default 0.1).
# usage: glim_queue23.sh A|B
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
if [ "$1" = A ]; then run pre_outlier; run pre_near1; run sub_ds02; else run pre_near2; run pre_outlier_strict; fi
echo "$(date -Is) queue23$1 done" >> $log
