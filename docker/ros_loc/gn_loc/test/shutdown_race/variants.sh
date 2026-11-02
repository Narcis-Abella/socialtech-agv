#!/bin/sh
# usage (host): variants.sh INSTALL   — the squeezed shutdown test for the variants in $VARIANTS (default A C B), 20 stops each; results in <this dir>/<INSTALL>_<variant>
#   A: bond only  C: bond + synthetic scans and TF  B: scans and TF, no bond.   IMAGE: an image with Nav2 and the gn_loc build under <this dir>/<INSTALL> (default socialtech:robot-navtest)
cd "$(dirname "$0")" || exit 1
IMAGE=${IMAGE:-socialtech:robot-navtest}
for v in ${VARIANTS:-A C B}; do
  case $v in A) synth=0; bond=1 ;; B) synth=1; bond=0 ;; C) synth=1; bond=1 ;; esac
  docker run --rm --cpuset-cpus 0-1 -e HOG=3 -e SYNTH=$synth -e BOND=$bond --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$PWD:/g" "$IMAGE" bash /g/harness.sh "$1" 20 "$1_$v" > "$1_$v.run" 2>&1
done
echo done > variants.done
