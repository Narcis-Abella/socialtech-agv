#!/bin/bash
# Records the fixed inputs of the AMCL-only replays (/scan, /tf, /Odometry of FAST-LIO2) for further real datasets, with the stack of run_bench_a.sh at 1x. Inside the sim image; mounts: tools at /tools,
# bags at /bags (CustomMsg bags in /bags/custom), nav_data at /data, inputs written to /inputs/<dataset>/inputs, logs to the out dir.
# usage: record_real_inputs.sh <out dir>      env: DATASETS ("hospital02 zona01 seat01 zona05 zona06")
set -eo pipefail
out=${1:?out dir}
nb=/tools/nav_bench
declare -A BAG=([hospital02]=hospital_02_custom [zona01]=mapping_zona_01_custom [seat01]=mapping_seat_01_custom [zona05]=mapping_zona_05_custom [zona06]=mapping_zona_06_custom)
declare -A SET=([hospital02]=live_hospital_02 [zona01]=live_zona_01 [seat01]=live_seat_01 [zona05]=live_zona_05 [zona06]=live_zona_06)
mkdir -p "$out"
for k in ${DATASETS:-hospital02 zona01 seat01 zona05 zona06}; do
  echo "== $(date +%H:%M:%S) $k" | tee -a "$out/progress.log"
  RECORD=1 PLAY_RATE=1.0 DIST=0 DYAW=0 INIT_STD=0.1 INIT_YAW_STD=5 \
    "$nb/run_bench_a.sh" "/bags/custom/${BAG[$k]}" "/data/${SET[$k]}/final.yaml" "/data/${SET[$k]}/alidarState.txt" 0.53 "/inputs/$k" > "$out/$k.log" 2>&1 \
    || echo "record $k failed" | tee -a "$out/progress.log"
done
echo done
