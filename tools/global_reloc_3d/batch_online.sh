#!/usr/bin/env bash
# NX host: run a list of end-to-end online cases sequentially, evaluate each (evaluator container, reference only there), append summary rows.
# usage: batch_online.sh CASES.csv BATCH_TAG      CASES lines: ds,window,mode,variant   (ds: eco01|eco03|hospital; mode: s3|s2; variant: own|cross|hole)
set -u
R=/home/jetson/work/reloc-online-20261009
B=/home/jetson/work/global-reloc-3d-20261009; C=$B/claude-v1
E01=$B/isolated-v16/map/map.xyz; H=$B/point7-v1/hospital-lio-s2-w01/map.xyz
RAW=/home/jetson/rosbags/custom
cases=$1; batch=$2
while IFS=, read -r ds win mode variant; do
  [ "$ds" = ds ] && continue
  case $ds in
    eco01) bag=$RAW/mapping_eco_-1_01_custom; yaml=/home/jetson/map_out/eco01_1x/map.yaml; xyz=$E01; ref=/home/jetson/map_out/eco01_1x/vs/run/alidarState.txt; csv=eco01-cert.csv; hole=eco01_1x;;
    eco03) bag=$RAW/mapping_eco_-1_03_custom; yaml=/home/jetson/map_out/eco01_1x/map.yaml; xyz=$E01; ref=$C/handoff/eco03_ref_in_eco01_world.txt; csv=eco03-cert.csv; hole=eco03;;
    hospital) bag=$RAW/hospital_02_custom; yaml=/home/jetson/map_out/hospital_02_r05b/map.yaml; xyz=$H; ref=/home/jetson/map_out/hospital_02_r05b/vs/run/alidarState.txt; csv=hospital-cert.csv; hole=hospital_02_r05b;;
  esac
  [ "$mode" = s2 ] && csv=${ds}-s2.csv
  line=$(grep "^$win," $C/windows/$csv | head -1); offset=$(echo "$line" | cut -d, -f3)
  neg=""
  case $variant in
    cross) [ "$ds" = hospital ] && xyz=$E01 || xyz=$H; yaml_neg=1; neg=--negative;;
    hole) xyz=$C/holes-v2/$hole-$win.xyz; [ -s "$xyz" ] || xyz=$C/holes-v2r/$hole-$win.xyz; neg=--negative;;
  esac
  name=$ds-$win-$mode-$variant; tag=$batch
  dur=$(grep -m1 nanoseconds $bag/metadata.yaml | tr -dc 0-9); remain=$(python3 -c "print($dur/1e9-($offset-2.5))")
  if python3 -c "import sys; sys.exit(0 if $remain < 75 else 1)"; then echo "$name-$tag SKIPPED bag_remaining=${remain}s (<75 s: bag ends before the decision deadline plus AMCL check)"; continue; fi
  NAME=$name BAG=$bag MAPYAML=$yaml MAPXYZ=$xyz OFFSET=$offset TAG=$tag MODE=$mode bash $R/tools/global_reloc_3d/run_online.sh > /dev/null 2>&1
  run=$R/runs/$name-$tag
  docker run --rm --network none -v $R/tools/global_reloc_3d:/t:ro -v $R/runs:/runs:ro -v "$(dirname $ref)":/refdir:ro -v "$(dirname $yaml)":/m:ro socialtech:mapper-verify \
    python3 -B /t/evaluate_online.py --run-dir /runs/$name-$tag --reference /refdir/$(basename $ref) --map-yaml /m/$(basename $yaml) $neg > $run/evaluation.json 2> $run/evaluation.err
  echo "$name-$tag $(python3 -c "import json;d=json.load(open('$run/evaluation.json'));print(d['class'],d['state'],d.get('reason'))" 2>&1 | tail -1)"
done < $cases
echo BATCH_DONE
