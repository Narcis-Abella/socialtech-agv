#!/usr/bin/env bash
# NX host: note-14 band support, DIAGNOSTIC ONLY, computed after the fact on frozen run outputs. It never feeds the decision.
# usage: R=<run root> band_diag_batch.sh TAG[ TAG...]   -> $R/band/<run>__<stage>.json
set -u
R=${R:?run root}; mkdir -p $R/band
for tag in "$@"; do for d in $R/runs/*-$tag; do for s in $d/work/attempt1/s2 $d/work/attempt1/s3 $d/work/attempt1; do
  [ -s $s/search/search.json ] || continue
  o=$R/band/$(basename $d)__$(basename $s).json; [ -s $o ] && continue
  docker run --rm --network none --user $(id -u):$(id -g) -v $R/tools/global_reloc_3d:/t:ro -v $s:/s:ro -v $R/band:/o socialtech:mapper-verify \
    python3 -B /t/support_breakdown.py --run /s/input --search /s/search --out /o/$(basename $o) 2>> $R/band/errors.log
done; done; done
echo BAND_DONE
