#!/bin/bash
# GLIM base and voxel 0.1 on our own bags (no ground truth). Waits for queue14. DDS domain 11, 6 cores.
cd ~
log=~/glim_eval/queue.log
until grep -q "queue14 done" $log; do sleep 15; done
O=/tools/glim_eval
run() {  # name bag overlay...
  name=$1; bag=$2; shift 2
  [ -d ~/glim_eval/$name ] && { echo "$(date -Is) skip $name (exists)" >> $log; return; }
  echo "$(date -Is) start $name" >> $log
  docker run --rm --name eval_$name --cpus 6 --network host -e ROS_DOMAIN_ID=11 \
    -v ~/rosbags:/bags:ro -v ~/glim_eval:/eval -v ~/glim_eval_tools:/tools:ro socialtech:robot \
    /tools/glim_eval.sh /bags/$bag /eval/$name 3 $O/base.json "$@" >> $log 2>&1
  echo "$(date -Is) end $name rc=$?" >> $log
}
for bag in mapping_eco_-1_03 hospital_02 mapping_real_01; do
  run own_$bag $bag
  run own_${bag}__sub_kvox $bag $O/sub_kvox.json
done
echo "$(date -Is) queue15 done" >> $log
