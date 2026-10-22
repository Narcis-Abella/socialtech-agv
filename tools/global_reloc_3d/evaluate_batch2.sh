#!/usr/bin/env bash
# Freeze, then evaluate positive and hole runs of one dataset. Env: V DS REF YAML [LINK]
set -u
C=/home/jetson/work/global-reloc-3d-20261009/claude-v1
: "${V:?}" "${DS:?}" "${REF:?}" "${YAML:?}"
sha256sum $C/search/$DS-*-$V/search.json > $C/search/frozen-$DS-$V.sha256
linkmount=""; linkarg=""
[ -n "${LINK:-}" ] && linkmount="-v $LINK:/reference/link.json:ro" && linkarg="--link /reference/link.json"
for s in $C/search/$DS-*-$V; do
  [ -f $s/search.json ] || continue
  case $s in *-cross-$V) continue;; esac
  n=$(basename $s)
  docker run --rm --network none -v $C/inputs/$n:/input:ro -v $s:/result -v $C/code:/evaluator:ro \
    -v $REF:/reference/alidarState.txt:ro -v $YAML:/reference/map.yaml:ro $linkmount socialtech:mapper-verify \
    python3 /evaluator/evaluate.py --run /input --search /result --reference /reference/alidarState.txt \
    --map-yaml /reference/map.yaml $linkarg --out /result/evaluation.json > /dev/null 2>&1 || echo "evaluation failed: $n"
done
