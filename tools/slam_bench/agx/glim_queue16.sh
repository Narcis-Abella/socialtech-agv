#!/bin/bash
# Ablation of the legacy config by section on top of `base`: which section lowers the vertical/tilt drift seen on eco_-1_01?
# Then the two most relevant on the degenerate M3DGR sequences. Waits for queue15. DDS domain 11, 6 cores.
cd ~
log=~/glim_eval/queue.log
until grep -q "queue15 done" $log; do sleep 15; done
O=/tools/glim_eval
run() {  # name bag topics-overlay overlay
  name=$1; bag=$2; topics=$3; ov=$4
  [ -d ~/glim_eval/$name ] && { echo "$(date -Is) skip $name (exists)" >> $log; return; }
  echo "$(date -Is) start $name" >> $log
  docker run --rm --name eval_$name --cpus 6 --network host -e ROS_DOMAIN_ID=11 \
    -v ~/rosbags:/bags:ro -v ~/glim_eval:/eval -v ~/glim_eval_tools:/tools:ro socialtech:robot \
    /tools/glim_eval.sh /bags/$bag /eval/$name 3 $O/base.json $topics $O/$ov.json >> $log 2>&1
  echo "$(date -Is) end $name rc=$?" >> $log
}
for cfg in leg_gyro leg_pre leg_noise leg_acc leg_gm; do run eco01_$cfg mapping_eco_-1_01 $O/base.json $cfg; done
for cfg in leg_gyro leg_pre; do
  run m3dgr_Corridor01__$cfg benchmarks/m3dgr_Corridor01 $O/m3dgr_topics.json $cfg
  run m3dgr_Elevator01__$cfg benchmarks/m3dgr_Elevator01 $O/m3dgr_topics.json $cfg
done
echo "$(date -Is) queue16 done" >> $log
