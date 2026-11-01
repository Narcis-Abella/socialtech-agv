#!/usr/bin/env bash
# NX host: (re)run the evaluator over existing run dirs of a tag (outputs untouched except evaluation.json). usage: reevaluate.sh TAG
set -u
R=/home/jetson/work/reloc-online-20261009; B=/home/jetson/work/global-reloc-3d-20261009; C=$B/claude-v1; tag=$1
for run in $R/runs/*-$tag; do
  n=$(basename $run); IFS=- read -r ds win mode variant _ <<< "$n"
  case $ds in
    eco01|f06_short_acquisition|f13_deadline40|r01_second_attempt) ref=/home/jetson/map_out/eco01_1x/vs/run/alidarState.txt; yaml=/home/jetson/map_out/eco01_1x/map.yaml;;
    eco03) ref=$C/handoff/eco03_ref_in_eco01_world.txt; yaml=/home/jetson/map_out/eco01_1x/map.yaml;;
    hospital) ref=/home/jetson/map_out/hospital_02_r05b/vs/run/alidarState.txt; yaml=/home/jetson/map_out/hospital_02_r05b/map.yaml;;
    *) ref=/home/jetson/map_out/eco01_1x/vs/run/alidarState.txt; yaml=/home/jetson/map_out/eco01_1x/map.yaml;;
  esac
  neg=""; [ "$variant" = cross ] || [ "$variant" = hole ] && neg=--negative
  docker run --rm --network none -v $R/tools/global_reloc_3d:/t:ro -v $R/runs:/runs:ro -v "$(dirname $ref)":/refdir:ro -v "$(dirname $yaml)":/m:ro socialtech:mapper-verify \
    python3 -B /t/evaluate_online.py --run-dir /runs/$n --reference /refdir/$(basename $ref) --map-yaml /m/$(basename $yaml) $neg > $run/evaluation.json 2> $run/evaluation.err
done
echo REEVAL_DONE
