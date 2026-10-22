#!/bin/bash
# Redo the GLIM runs contaminated by cross-talk. Old dirs are root-owned (created by docker): they cannot be moved to another parent,
# but they can be renamed inside ~/glim_eval, so they become <name>_contaminated. GLIM: DDS domain 11, 6 cores (competitors run at 0.5x on the rest).
cd ~
log=~/glim_eval/queue.log
O=/tools/glim_eval
for spec in tiers_IndoorOffice2:tiers_topics m3dgr_Corridor01:m3dgr_topics m3dgr_Dynamic01:m3dgr_topics m3dgr_Sha-turn01:m3dgr_topics m3dgr_Elevator01:m3dgr_topics; do
  seq=${spec%%:*}; topics=${spec##*:}
  for cfg in sub_kvox sub_ovl_0p9; do
    name=${seq}__$cfg
    if [ -d ~/glim_eval/$name ] && [ ! -e ~/glim_eval/${name}_contaminated ]; then mv ~/glim_eval/$name ~/glim_eval/${name}_contaminated || { echo "$(date -Is) rename failed $name" >> $log; continue; }; fi
    echo "$(date -Is) start $name (redo2)" >> $log
    docker run --rm --name eval_$name --cpus 6 --network host -e ROS_DOMAIN_ID=11 \
      -v ~/rosbags:/bags:ro -v ~/glim_eval:/eval -v ~/glim_eval_tools:/tools:ro socialtech:robot \
      /tools/glim_eval.sh /bags/benchmarks/$seq /eval/$name 3 $O/base.json $O/$topics.json $O/$cfg.json >> $log 2>&1
    echo "$(date -Is) end $name rc=$?" >> $log
  done
done
echo "$(date -Is) queue13 done" >> $log
