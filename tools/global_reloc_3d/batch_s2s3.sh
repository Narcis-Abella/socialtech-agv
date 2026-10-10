#!/usr/bin/env bash
# NX host: end-to-end S2 -> supervised-S3 cases, sequential, each scored by the evaluator container (reference only there).
# usage: R=<run root> batch_s2s3.sh CASES.csv BATCH_TAG
# CASES lines: ds,window,variant,exposure_s,auth[,mode]   ds: eco01|eco03|hospital   variant: own|cross|hole
#   auth: auto (replay stand-in for the supervisor's grant) | none (nobody authorizes S3)   mode: s2s3 (default) | s3 | s2 (note-15 baselines)
set -u
R=${R:?run root}; export R
B=/home/jetson/work/global-reloc-3d-20261009; C=$B/claude-v1
E01=$B/isolated-v16/map/map.xyz; H=$B/point7-v1/hospital-lio-s2-w01/map.xyz
RAW=/home/jetson/rosbags/custom
cases=$1; batch=$2
while IFS=, read -r ds win variant exp auth mode; do
  [ "$ds" = ds ] && continue
  mode=${mode:-s2s3}
  case $ds in
    eco01) bag=$RAW/mapping_eco_-1_01_custom; yaml=/home/jetson/map_out/eco01_1x/map.yaml; xyz=$E01; ref=/home/jetson/map_out/eco01_1x/vs/run/alidarState.txt; hole=eco01_1x;;
    eco03) bag=$RAW/mapping_eco_-1_03_custom; yaml=/home/jetson/map_out/eco01_1x/map.yaml; xyz=$E01; ref=$C/handoff/eco03_ref_in_eco01_world.txt; hole=eco03;;
    hospital) bag=$RAW/hospital_02_custom; yaml=/home/jetson/map_out/hospital_02_r05b/map.yaml; xyz=$H; ref=/home/jetson/map_out/hospital_02_r05b/vs/run/alidarState.txt; hole=hospital_02_r05b;;
  esac
  line=$(cat $C/windows/$ds-cert.csv $C/windows/$ds-s2.csv $R/windows/$ds-stat.csv 2>/dev/null | grep "^$win," | head -1); offset=$(echo "$line" | cut -d, -f3)
  [ -n "$offset" ] || { echo "$ds-$win NO_WINDOW"; continue; }
  neg=""
  case $variant in
    cross) [ "$ds" = hospital ] && xyz=$E01 || xyz=$H; neg=--negative;;
    hole) xyz=$C/holes-v2/$hole-$win.xyz; [ -s "$xyz" ] || xyz=$C/holes-v2r/$hole-$win.xyz; neg=--negative;;
  esac
  token=$mode; extra=""
  if [ "$mode" = s2s3 ]; then
    token="s2s3x${exp%.*}"; [ "$auth" = none ] && token="${token}na"
    extra="s2_exposure_s:=$exp s3_auto_authorize:=$([ "$auth" = auto ] && echo true || echo false)"
  fi
  name=$ds-$win-$token-$variant
  dur=$(grep -m1 nanoseconds $bag/metadata.yaml | tr -dc 0-9); remain=$(python3 -c "print($dur/1e9-($offset-2.5))")
  if python3 -c "import sys; sys.exit(0 if $remain < 75 else 1)"; then echo "$name-$batch SKIPPED bag_remaining=${remain}s"; continue; fi
  NAME=$name BAG=$bag MAPYAML=$yaml MAPXYZ=$xyz OFFSET=$offset TAG=$batch MODE=$mode LAUNCH_EXTRA="$extra ${EXTRA:-}" bash $R/tools/global_reloc_3d/run_online.sh > /dev/null 2>&1
  run=$R/runs/$name-$batch
  docker run --rm --network none -v $R/tools/global_reloc_3d:/t:ro -v $R/runs:/runs:ro -v "$(dirname $ref)":/refdir:ro -v "$(dirname $yaml)":/m:ro socialtech:mapper-verify \
    python3 -B /t/evaluate_online.py --run-dir /runs/$name-$batch --reference /refdir/$(basename $ref) --map-yaml /m/$(basename $yaml) $neg > $run/evaluation.json 2> $run/evaluation.err
  echo "$name-$batch $(python3 -c "
import json,glob
d=json.load(open('$run/evaluation.json')); m=json.load(open(sorted(glob.glob('$run/work/attempt*/metrics.json'))[-1]))
s2=(m.get('stages') or {}).get('s2') or {}
print(d['class'],d['state'],d.get('reason'),'stage=',m.get('accepted_stage'),'s2=',s2.get('decision'),'t_acc=',m['total_s']['protocol_accept_from_first_input'],'t_end=',round(m['total_s']['attempt_end_from_first_input'],1))" 2>&1 | tail -1)"
done < $cases
echo BATCH_DONE
