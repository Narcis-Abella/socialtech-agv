#!/bin/bash
# Redo GLIM runs that were contaminated by cross-talk with the FAST-LIO2 player (same DDS domain and topic names).
# Old dirs are moved to ~/glim_eval/contaminated/. GLIM runs in ROS_DOMAIN_ID=11, competitors in 77.
cd ~
log=~/glim_eval/queue.log
until grep -q "queue11 done" $log; do sleep 10; done
mkdir -p ~/glim_eval/contaminated
O=/tools/glim_eval
for spec in tiers_IndoorOffice2:tiers_topics m3dgr_Corridor01:m3dgr_topics m3dgr_Dynamic01:m3dgr_topics m3dgr_Sha-turn01:m3dgr_topics m3dgr_Elevator01:m3dgr_topics; do
  seq=${spec%%:*}; topics=${spec##*:}
  for cfg in sub_kvox sub_ovl_0p9; do
    name=${seq}__$cfg
    [ -d ~/glim_eval/$name ] && mv ~/glim_eval/$name ~/glim_eval/contaminated/$name
    echo "$(date -Is) start $name (redo)" >> $log
    docker run --rm --name eval_$name --network host -e ROS_DOMAIN_ID=11 \
      -v ~/rosbags:/bags:ro -v ~/glim_eval:/eval -v ~/glim_eval_tools:/tools:ro socialtech:robot \
      /tools/glim_eval.sh /bags/benchmarks/$seq /eval/$name 3 $O/base.json $O/$topics.json $O/$cfg.json >> $log 2>&1
    echo "$(date -Is) end $name rc=$?" >> $log
  done
done
echo "$(date -Is) queue12 done" >> $log
