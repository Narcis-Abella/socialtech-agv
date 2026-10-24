#!/bin/bash
# Records the fixed inputs of the AMCL-only replays (/scan, /tf, /Odometry of FAST-LIO2) for further real datasets, with the stack of run_bench_a.sh at 1x. Inside the sim image; mounts: tools at /tools,
# bags at /bags (CustomMsg bags in /bags/custom), nav_data at /data, inputs written to /inputs/<dataset>_v2/inputs (SUFFIX), logs to the out dir.
# usage: record_real_inputs.sh <out dir>      env: DATASETS ("hospital02 zona01 seat01 zona05 zona06"; also eco01 eco03 corridor01 corridor02 dynamic01), SUFFIX (_v2)
set -eo pipefail
out=${1:?out dir}
nb=/tools/nav_bench
declare -A BAG=([hospital02]=custom/hospital_02_custom [zona01]=custom/mapping_zona_01_custom [seat01]=custom/mapping_seat_01_custom [zona05]=custom/mapping_zona_05_custom [zona06]=custom/mapping_zona_06_custom
                [eco01]=custom/mapping_eco_-1_01_custom [eco03]=custom/mapping_eco_-1_03_custom [corridor01]=benchmarks/m3dgr_Corridor01_custom [corridor02]=benchmarks/m3dgr_Corridor02_custom [dynamic01]=benchmarks/m3dgr_Dynamic01_custom)
# map, reference trajectory, sensor height (m) per dataset; M3DGR bags use /livox/mid360/* and need their own height (Corridor01 0.773, Dynamic01 0.782, Corridor02 0.831: floor plane of ply_to_map's trajectory check)
declare -A MAPF=([hospital02]=/data/live_hospital_02/final.yaml [zona01]=/data/live_zona_01/final.yaml [seat01]=/data/live_seat_01/final.yaml [zona05]=/data/live_zona_05/final.yaml [zona06]=/data/live_zona_06/final.yaml
                 [eco01]=/data/eco_-1_01/map.yaml [eco03]=/data/eco_-1_03/final.yaml [corridor01]=/data/res_test/corridor01_r0.05.yaml [corridor02]=/data/res_test/corridor02_r0.05.yaml [dynamic01]=/data/res_test/dynamic01_r0.05.yaml)
declare -A REFF=([hospital02]=/data/live_hospital_02/alidarState.txt [zona01]=/data/live_zona_01/alidarState.txt [seat01]=/data/live_seat_01/alidarState.txt [zona05]=/data/live_zona_05/alidarState.txt [zona06]=/data/live_zona_06/alidarState.txt
                 [eco01]=/data/eco_-1_01/alidarState.txt [eco03]=/data/eco_-1_03/alidarState.txt [corridor01]=/data/res_test/corridor01_alidarState.txt [corridor02]=/data/res_test/corridor02_alidarState.txt [dynamic01]=/data/res_test/dynamic01_alidarState.txt)
declare -A HGT=([corridor01]=0.773 [dynamic01]=0.782 [corridor02]=0.831)
mkdir -p "$out"
i=0
for k in ${DATASETS:-hospital02 zona01 seat01 zona05 zona06}; do
  i=$((i + 1))   # one ROS domain per dataset: a stray node of the previous run (planar_odom survives the launch kill) would republish the TF and /scan of the next one
  echo "== $(date +%H:%M:%S) $k" | tee -a "$out/progress.log"
  # ros2 bag record refuses to write into an existing bag directory (and run_bench_a.sh then goes on without recording): one fresh directory per recording
  [ -e "/inputs/${k}${SUFFIX:-_v2}" ] && { echo "/inputs/${k}${SUFFIX:-_v2} exists: set SUFFIX" | tee -a "$out/progress.log"; continue; }
  m3=; case "$k" in corridor0*|dynamic01) m3=1;; esac   # M3DGR: other topics
  env ROS_DOMAIN_ID=$((80 + i)) RECORD=1 PLAY_RATE=1.0 DIST=0 DYAW=0 INIT_STD=0.1 INIT_YAW_STD=5 ${m3:+LIDAR=/livox/mid360/lidar IMU=/livox/mid360/imu} \
    "$nb/run_bench_a.sh" "/bags/${BAG[$k]}" "${MAPF[$k]}" "${REFF[$k]}" "${HGT[$k]:-0.53}" "/inputs/${k}${SUFFIX:-_v2}" > "$out/$k.log" 2>&1 \
    || echo "record $k failed" | tee -a "$out/progress.log"
  [ -s "/inputs/${k}${SUFFIX:-_v2}/inputs/inputs_0.mcap" ] || echo "record $k: NO bag written" | tee -a "$out/progress.log"
done
echo done
