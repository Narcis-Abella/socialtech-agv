#!/bin/bash
# GLIM glim_ext modules, one at a time, on our own bags (floor is flat in all of them, so z range / roll / pitch are drift):
#   ext_grav = libgravity_estimator.so, ext_flat = libflat_earther.so (min_travel_distance 20, max_neighbor_distance 1e4: "whole ground is flat").
# A 1-run smoke test per module first (module must log its startup line, GLIM must exit 0); the queue stops if it does not.
# Needs ~/glim_ext_build5 (tools/glim_eval/build_glim_ext.sh). DDS domain 11, 6 cores.
cd ~
log=~/glim_eval/queue.log
O=/tools/glim_eval
EXT=~/glim_ext_build5
run() {  # name bag runs overlay
  name=$1; bag=$2; runs=$3; ov=$4
  [ -d ~/glim_eval/$name ] && { echo "$(date -Is) skip $name (exists)" >> $log; return 0; }
  echo "$(date -Is) start $name" >> $log
  docker run --rm --name eval_$name --cpus 6 --network host -e ROS_DOMAIN_ID=11 \
    -e GLIM_DEFAULTS=/ext_config -e LD_LIBRARY_PATH=/ext:/opt/socialtech/lib:/opt/ros/jazzy/lib/aarch64-linux-gnu:/opt/ros/jazzy/lib \
    -v $EXT/lib:/ext:ro -v $EXT/config:/ext_config:ro \
    -v ~/rosbags:/bags:ro -v ~/glim_eval:/eval -v ~/glim_eval_tools:/tools:ro socialtech:robot \
    /tools/glim_eval.sh /bags/$bag /eval/$name $runs $O/base.json $O/$ov.json >> $log 2>&1
  rc=$?
  echo "$(date -Is) end $name rc=$rc" >> $log
  return $rc
}
for ext in ext_grav ext_flat; do
  pat=$([ $ext = ext_grav ] && echo "gravity estimator module" || echo "flat earther module")
  run smoke_$ext mapping_eco_-1_03 1 $ext
  if ! grep -qi "$pat" ~/glim_eval/smoke_$ext/run1.log || [ "$(awk 'NR==2{print $2}' ~/glim_eval/smoke_$ext/runs.tsv)" != 0 ]; then
    echo "$(date -Is) smoke $ext FAILED (module not loaded or GLIM exit != 0); queue17 aborted" >> $log; exit 1
  fi
  echo "$(date -Is) smoke $ext ok" >> $log
done
for ext in ext_grav ext_flat; do
  run eco01_$ext mapping_eco_-1_01 3 $ext
  run own_hospital_02__$ext hospital_02 3 $ext
  run own_mapping_eco_-1_03__$ext mapping_eco_-1_03 3 $ext
done
echo "$(date -Is) queue17 done" >> $log
