#!/usr/bin/env bash
# Freeze every method output first, then evaluate own/hole runs in a separate
# container that is the only one mounting the references.
set -u
C=/home/jetson/work/global-reloc-3d-20261009/claude-v1
V=${V:?set a version tag}
M=/home/jetson/map_out
sha256sum $C/search/*-$V/search.json > $C/search/frozen-batch-$V.sha256
for s in $C/search/*-{own,hole}-$V; do
  [ -f $s/search.json ] || continue
  n=$(basename $s); ds=${n%%-w*}
  docker run --rm --network none -v $C/inputs/$n:/input:ro -v $s:/result -v $C/code:/evaluator:ro \
    -v $M/$ds/vs/run/alidarState.txt:/reference/alidarState.txt:ro -v $M/$ds/map.yaml:/reference/map.yaml:ro \
    socialtech:mapper-verify python3 /evaluator/evaluate.py --run /input --search /result \
    --reference /reference/alidarState.txt --map-yaml /reference/map.yaml --out /result/evaluation.json > /dev/null 2>&1 \
    || echo "evaluation failed: $n"
done
