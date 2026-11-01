#!/usr/bin/env bash
# NX host: software-restart campaign. Each case is a fresh container, a fresh node and a cold FAST-LIO2 process on a warm board
# (page cache, clocks and thermal state carry over): a SOFTWARE restart, not a physical cold boot.
# usage: R=<run root> batch_restarts.sh CASES.csv TAG [first_index last_index]     CASES from gen_restart_cases.py
# RUN_ENV="KEEP_PLAYING=1 PLAY_S=120 STOP_AT=100": keep each case running after LOCALIZED (supervision on other maps, note 22) and
# score the permit exposure of every run.
set -u
R=${R:?run root}; export R
B=/home/jetson/work/global-reloc-3d-20261009; C=$B/claude-v1
E01=$B/isolated-v16/map/map.xyz; H=$B/point7-v1/hospital-lio-s2-w01/map.xyz
RAW=/home/jetson/rosbags/custom
cases=$1; T=$2; lo=${3:-0}; hi=${4:-999999}; i=-1
while IFS=, read -r id seed ds win off jit pre lead dom; do
  [ "$id" = id ] && continue
  i=$((i+1)); { [ $i -lt $lo ] || [ $i -gt $hi ]; } && continue
  case $ds in
    eco01) bag=$RAW/mapping_eco_-1_01_custom; yaml=/home/jetson/map_out/eco01_1x/map.yaml; xyz=$E01; ref=/home/jetson/map_out/eco01_1x/vs/run/alidarState.txt;;
    eco03) bag=$RAW/mapping_eco_-1_03_custom; yaml=/home/jetson/map_out/eco01_1x/map.yaml; xyz=$E01; ref=$C/handoff/eco03_ref_in_eco01_world.txt;;
    hospital) bag=$RAW/hospital_02_custom; yaml=/home/jetson/map_out/hospital_02_r05b/map.yaml; xyz=$H; ref=/home/jetson/map_out/hospital_02_r05b/vs/run/alidarState.txt;;
  esac
  name=$id-$ds-$win; d=$R/runs/$name-$T
  offset=$(python3 -c "print($off+$jit)")
  mem0=$(awk '/MemAvailable/{print $2}' /proc/meminfo)
  env ${RUN_ENV:-} NAME=$name BAG=$bag MAPYAML=$yaml MAPXYZ=$xyz OFFSET=$offset TAG=$T MODE=s2s3 PRE=$pre LEAD=$lead DOMAIN=$dom \
    LAUNCH_EXTRA="s2_exposure_s:=3.0 s3_auto_authorize:=true" bash $R/tools/global_reloc_3d/run_online.sh > /dev/null 2>&1
  echo "mem_available_kb_before: $mem0 after: $(awk '/MemAvailable/{print $2}' /proc/meminfo)  scenario: $id,$seed,$ds,$win,$off,$jit,$pre,$lead,$dom  kind: software_restart" >> $d/host.txt
  docker run --rm --network none -v $R/tools/global_reloc_3d:/t:ro -v $R/runs:/runs:ro -v "$(dirname $ref)":/refdir:ro -v "$(dirname $yaml)":/m:ro socialtech:mapper-verify \
    python3 -B /t/evaluate_online.py --run-dir /runs/$name-$T --reference /refdir/$(basename $ref) --map-yaml /m/$(basename $yaml) > $d/evaluation.json 2> $d/evaluation.err
  python3 -B $R/tools/global_reloc_3d/f2_check.py "$d" '.*' $R/table-$T.csv
  [ -n "${RUN_ENV:-}" ] && docker run --rm --network none -v $R/tools/global_reloc_3d:/t:ro -v $R/runs:/runs:ro -v "$(dirname $ref)":/refdir:ro -v "$(dirname $yaml)":/m:ro socialtech:mapper-verify \
    python3 -B /t/permit_exposure.py --run-dir /runs/$name-$T --reference /refdir/$(basename $ref) --map-yaml /m/$(basename $yaml) >> $R/exposure-$T.jsonl 2>> $R/exposure-$T.err
done < $cases
echo BATCH_DONE
