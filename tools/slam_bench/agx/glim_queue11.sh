#!/bin/bash
# Tuned GLIM configs on every benchmark sequence (GT available), 3 runs each. Waits for queue10.
cd ~
log=~/glim_eval/queue.log
until grep -q "queue10 done" $log; do sleep 10; done
O=/tools/glim_eval
for spec in tiers_IndoorOffice1:tiers_topics tiers_IndoorOffice2:tiers_topics m3dgr_Corridor01:m3dgr_topics m3dgr_Dynamic01:m3dgr_topics m3dgr_Sha-turn01:m3dgr_topics m3dgr_Elevator01:m3dgr_topics; do
  seq=${spec%%:*}; topics=${spec##*:}
  for cfg in sub_kvox sub_ovl_0p9; do
    name=${seq}__$cfg
    echo "$(date -Is) start $name" >> $log
    docker run --rm --name eval_$name --network host \
      -v ~/rosbags:/bags:ro -v ~/glim_eval:/eval -v ~/glim_eval_tools:/tools:ro socialtech:robot \
      /tools/glim_eval.sh /bags/benchmarks/$seq /eval/$name 3 $O/base.json $O/$topics.json $O/$cfg.json >> $log 2>&1
    echo "$(date -Is) end $name rc=$?" >> $log
  done
done
echo "$(date -Is) queue11 done" >> $log
