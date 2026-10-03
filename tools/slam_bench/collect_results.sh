#!/bin/bash
# Copy every trajectory from the AGX into <dest>/<method>/<sequence>/run<N>.tum (layout of eval_all.py).
# usage: collect_results.sh <dest>      (AGX = ssh host, default alias agx-cable: a Host entry in ~/.ssh/config for the direct-cable link)
set -eo pipefail
dest=${1:?usage: collect_results.sh <dest>}
AGX=${AGX:-agx-cable}
ssh_() { ssh "$AGX" "$@" </dev/null; }
seqname() {  # dataset dir name -> short name used by eval_all.py
  case $1 in tiers_IndoorOffice1*|tiers_office1*) echo office1;; tiers_IndoorOffice2*|tiers_office2*) echo office2;; m3dgr_*) echo "${1#m3dgr_}";; own_mapping_*) echo "${1#own_mapping_}";; own_*) echo "${1#own_}";; esac
}
# GLIM: ~/glim_eval/<seq>[__<cfg>]/run<N>/traj_lidar.txt (skips *_contaminated)
for d in $(ssh_ 'cd ~/glim_eval; ls -d tiers_* m3dgr_* own_* 2>/dev/null | grep -v _contaminated'); do
  cfg=base; name=$d; [[ $d == *__* ]] && { cfg=${d##*__}; name=${d%%__*}; }
  seq=$(seqname "${name%%__*}"); [ -n "$seq" ] || continue
  for r in 1 2 3; do
    mkdir -p "$dest/glim_$cfg/$seq"
    ssh_ "cat ~/glim_eval/$d/run$r/traj_lidar.txt" > "$dest/glim_$cfg/$seq/run$r.tum" 2>/dev/null || rm -f "$dest/glim_$cfg/$seq/run$r.tum"
  done
done
# eco_-1_01, the bag GLIM was first tuned on: ~/glim_eval/eco01_<config>/run<N>/traj_lidar.txt
for d in $(ssh_ 'cd ~/glim_eval; ls -d eco01_* 2>/dev/null'); do
  for r in 1 2 3; do
    mkdir -p "$dest/glim_${d#eco01_}/eco_-1_01"
    ssh_ "cat ~/glim_eval/$d/run$r/traj_lidar.txt" > "$dest/glim_${d#eco01_}/eco_-1_01/run$r.tum" 2>/dev/null || rm -f "$dest/glim_${d#eco01_}/eco_-1_01/run$r.tum"
  done
done
# competitors at 0.5x: ~/bench_out/<method>_r05_<seq>/odom.tum (voxelslam: vs/*/alidarState.txt, one file per session after a reset, merged by time, first 8 columns)
for d in $(ssh_ 'cd ~/bench_out; ls -d fastlio_r05_* iglio_r05_* voxelslam_* 2>/dev/null | grep -v "\.log" | grep -v nofinish | grep -v _custom'); do
  method=${d%%_*}; rest=${d#*_}; rest=${rest#r05_}
  run=1; [[ $rest =~ ^rep([0-9]+)_ ]] && { run=${BASH_REMATCH[1]}; rest=${rest#rep*_}; }   # repeats: voxelslam_rep<N>_<seq>
  seq=$(seqname "${rest%_custom}"); [ -n "$seq" ] || continue
  mkdir -p "$dest/${method}05/$seq"
  if [ "$method" = voxelslam ]; then
    ssh_ "cat ~/bench_out/$d/vs/*/alidarState.txt | sort -n -k1,1 | cut -d' ' -f1-8" > "$dest/${method}05/$seq/run$run.tum" 2>/dev/null || rm -f "$dest/${method}05/$seq/run$run.tum"
  else
    ssh_ "cat ~/bench_out/$d/odom.tum" > "$dest/${method}05/$seq/run$run.tum" 2>/dev/null || rm -f "$dest/${method}05/$seq/run$run.tum"
  fi
done
find "$dest" -type d -empty -delete 2>/dev/null || true
