#!/bin/sh
# usage (host): CELLS="name:install:scan_hz:mode:delay_ms ..." variants.sh   — the squeezed shutdown test, N stops per cell (default 30); results in <this dir>/out_<name>
#   install: <this dir>/install_<install> (a gn_loc build); mode: sigint | lcshutdown (ACTIVE -> FINALIZED first); delay_ms: GN_SCAN_DELAY_MS, instrumented builds only (0 otherwise)
#   e.g. CELLS="fix10:fix:10:sigint:0 fix50:fix:50:sigint:0 lc_fix:fix:10:lcshutdown:0". IMAGE: an image with Nav2 (default socialtech:robot-navtest)
cd "$(dirname "$0")" || exit 1
IMAGE=${IMAGE:-socialtech:robot-navtest}
for c in $CELLS; do
  IFS=: read name inst hz mode delay <<E
$c
E
  docker run --rm --cpuset-cpus 0-1 -e HOG=3 -e BOND=1 -e SCAN_HZ=$hz -e MODE=$mode -e GN_SCAN_DELAY_MS=$delay --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$PWD:/g" "$IMAGE" bash /g/harness.sh install_$inst ${N:-30} out_$name > out_$name.run 2>&1
done
echo done > variants.done
