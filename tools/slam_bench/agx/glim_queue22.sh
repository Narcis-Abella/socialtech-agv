#!/bin/bash
# Do the spurious corridor pixels of eco01 come from Livox noise points? GLIM on eco_-1_01 with the points carrying Livox tag bits removed
# (bags made by tag_filter_bag.py into ~/bags_tag): ctrl = copy unchanged (control), g12 = tag bits 7-4, g3 = bits 3-2 (dust), g4 = bits 1-0 (thread-like), all = any bit.
# Two streams on separate DDS domains (A=11, B=13), 6 cores each, 2 runs each. usage: glim_queue22.sh A|B
cd ~
log=~/glim_eval/queue.log
O=/tools/glim_eval
dom=$([ "$1" = A ] && echo 11 || echo 13)
run() {  # name bag
  name=$1; bag=$2
  [ -d ~/glim_eval/$name ] && { echo "$(date -Is) skip $name (exists)" >> $log; return 0; }
  echo "$(date -Is) start $name" >> $log
  docker run --rm --name eval_$name --cpus 6 --network host -e ROS_DOMAIN_ID=$dom \
    -v ~/bags_tag:/bags:ro -v ~/glim_eval:/eval -v ~/glim_eval_tools:/tools:ro socialtech:robot \
    /tools/glim_eval.sh /bags/$bag /eval/$name 2 $O/base.json $O/sub_kvox.json >> $log 2>&1
  echo "$(date -Is) end $name rc=$?" >> $log
}
if [ "$1" = A ]; then
  run eco01_tag_ctrl__sub_kvox eco01_ctrl
  run eco01_tag_g12__sub_kvox eco01_g12
  run eco01_tag_g3__sub_kvox eco01_g3
else
  run eco01_tag_g4__sub_kvox eco01_g4
  run eco01_tag_all__sub_kvox eco01_all
fi
echo "$(date -Is) queue22$1 done" >> $log
