#!/bin/bash
# glim_ext modules on top of sub_kvox (dense submaps: floor_tilt.py needs many submaps per dump to measure drift). 3 runs each. DDS domain 11, 6 cores.
cd ~
log=~/glim_eval/queue.log
O=/tools/glim_eval
EXT=~/glim_ext_build5
run() {  # name bag overlay...
  name=$1; bag=$2; shift 2
  [ -d ~/glim_eval/$name ] && { echo "$(date -Is) skip $name (exists)" >> $log; return 0; }
  echo "$(date -Is) start $name" >> $log
  docker run --rm --name eval_$name --cpus 6 --network host -e ROS_DOMAIN_ID=11 \
    -e GLIM_DEFAULTS=/ext_config -e LD_LIBRARY_PATH=/ext:/opt/socialtech/lib:/opt/ros/jazzy/lib/aarch64-linux-gnu:/opt/ros/jazzy/lib \
    -v $EXT/lib:/ext:ro -v $EXT/config:/ext_config:ro \
    -v ~/rosbags:/bags:ro -v ~/glim_eval:/eval -v ~/glim_eval_tools:/tools:ro socialtech:robot \
    /tools/glim_eval.sh /bags/$bag /eval/$name 3 $O/base.json "$@" >> $log 2>&1
  echo "$(date -Is) end $name rc=$?" >> $log
}
for ext in ext_grav ext_flat; do
  run eco01_kvox_$ext mapping_eco_-1_01 $O/sub_kvox.json $O/$ext.json
  run own_hospital_02__kvox_$ext hospital_02 $O/sub_kvox.json $O/$ext.json
  run own_mapping_eco_-1_03__kvox_$ext mapping_eco_-1_03 $O/sub_kvox.json $O/$ext.json
done
echo "$(date -Is) queue18 done" >> $log
