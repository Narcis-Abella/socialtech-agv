#!/bin/bash
# Bag -> map, inside the `mapper` image: Voxel-SLAM replay (voxelslam_run.sh), then make_map.py does the rest in one process: levels the floor once (refusing a map whose floor does
# not fit the trajectory), removes the ghosts the beams go through, writes the clouds, builds the 2D map from the cleaned cloud and writes report.json.
# usage: make_map.sh [--reuse-slam] <bag_dir> <out_dir>
# outputs in <out_dir>: map.pgm + map.yaml (2D, for Nav2 and gn_loc), map.ply (the raw Voxel-SLAM cloud), map_clean.ply (ghost-free, whole height, floor z = 0: the relocalizer prior
#   is built from it), report.json (what the run and the map say about themselves; `pipeline: 2`, the commit and the parameters of the run).
# --reuse-slam: do not replay the bag; redo everything after the SLAM from <out_dir>/vs/run (and map.ply when it exists, else the frames of vs/run). Regenerates a map in seconds.
# env:   LIDAR (/livox/lidar)  IMU (/livox/imu)  PLAY_RATE (2: the replay speed, poses were identical to 1x on the 366 s bag; if the run warns that scans were lost, relaunch with 1)
#        CEIL (1.80, metres above the floor; 30 for outdoor maps)  RES (0.025, m: cell size of the map and, unless VOXEL is set, the thinning of the cloud)  VOXEL (= RES)  DEGRADE_BOUND (100000)
#        VOXELSLAM_RUN ENGINE_CHECK MAKE_MAP_PY: the command each step runs (default: the script next to this one; the tests swap them). MAKE_MAP_COMMIT: the commit recorded in report.json.
# On any failure it exits non-zero naming the step. A fresh run leaves no map.pgm / map.yaml in <out_dir>; a failed --reuse-slam leaves the ones that were there.
set -uo pipefail
reuse=
[ "${1:-}" = "--reuse-slam" ] && { reuse=1; shift; }
[ $# -eq 2 ] || { echo "usage: make_map.sh [--reuse-slam] <bag_dir> <out_dir>" >&2; exit 2; }
bag=$1; out=$2
here=$(dirname "$(readlink -f "$0")")
: "${LIDAR:=/livox/lidar}" "${IMU:=/livox/imu}" "${PLAY_RATE:=2}" "${CEIL:=1.80}" "${RES:=0.025}" "${VOXEL:=$RES}" "${DEGRADE_BOUND:=100000}"
: "${VOXELSLAM_RUN:=$here/voxelslam_run.sh}" "${ENGINE_CHECK:=python3 $here/ghosts.py --check-engine}" "${MAKE_MAP_PY:=python3 $here/make_map.py}"
export PLAY_RATE
fail() { echo "make_map: FAILED at step '$1': $2" >&2; exit 1; }

mkdir -p "$out"
stage=$out/.build      # make_map.py writes here and the files move into <out_dir> only when everything worked: a failed --reuse-slam must not cost the map that was there
poses=$out/vs/run/alidarState.txt
if [ -n "$reuse" ]; then
  [ -s "$poses" ] || fail reuse "no $poses to reuse: run without --reuse-slam first"
  [ -s "$out/map.ply" ] || [ -e "$out/vs/run/0.pcd" ] || fail reuse "neither $out/map.ply nor the frames of $out/vs/run to rebuild the cloud from"
else
  rm -f "$out/map.pgm" "$out/map.yaml" "$out/map_clean.ply" "$out/report.json" "$out/map.ply" "$out/node.log" "$out/vs/edge.txt" "$out/play_rate"    # a map of an earlier run must not pass for this one
  [ -e "$out/vs/run" ] && { rm -r "$out/vs/run" || fail cleanup "cannot remove $out/vs/run of an earlier run (made by another user?)"; }   # its poses and scans go too: Voxel-SLAM exits if its save directory exists
fi
[ -e "$stage" ] && { rm -r "$stage" || fail cleanup "cannot remove $stage of an earlier run"; }

# asked before the long Voxel-SLAM stage, not after it: a host without the C++ library fails in a second
$ENGINE_CHECK 2> "$out/preflight.log" || { cat "$out/preflight.log" >&2; fail preflight "the C++ beam engine is not available (see above)"; }

if [ -z "$reuse" ]; then
  printf 's/^\\( *\\)degrade_bound:.*/\\1degrade_bound: %s/\n' "$DEGRADE_BOUND" > "$out/voxelslam.sed"
  VS_SED="$out/voxelslam.sed" $VOXELSLAM_RUN "$bag" "$LIDAR" "$IMU" "$out" > "$out/voxelslam_run.log" 2>&1 \
    || fail voxelslam "see $out/voxelslam_run.log and $out/node.log"
  [ -s "$poses" ] || fail voxelslam "no poses were written (see $out/node.log)"
  echo "$PLAY_RATE" > "$out/play_rate"      # --reuse-slam reports the rate this replay used, not the default of a later run
fi

if [ -n "$reuse" ] && [ -s "$out/map.ply" ]; then src=(--cloud "$out/map.ply" --poses "$poses"); else src=(--run "$out/vs/run"); fi
rate=(); [ -s "$out/play_rate" ] && rate=(--play-rate "$(cat "$out/play_rate")")      # an old folder has no record of it: the report says so
mkdir -p "$stage"
$MAKE_MAP_PY "${src[@]}" --bag "$bag" --out "$stage" --node-log "$out/node.log" --res "$RES" --voxel "$VOXEL" --ceil "$CEIL" --topic "$LIDAR" "${rate[@]}" > "$out/make_map.log" 2>&1 \
  || { msg=$(tail -n 1 "$out/make_map.log"); rm -r "$stage"; fail make_map "$msg"; }
for f in map.ply map_clean.ply report.json map.pgm map.yaml; do mv -f "$stage/$f" "$out/$f" || fail install "cannot move $stage/$f"; done      # the map itself last
rm -r "$stage"
grep '^WARNING' "$out/make_map.log" >&2 || true
echo "make_map: OK -> $out/map.yaml ($(python3 -c "import json,sys; r=json.load(open(sys.argv[1])); print('resets', r.get('resets'), '| end-to-start', r.get('end_to_start_m'), 'm | sensor height', r.get('sensor_height_m'), 'm')" "$out/report.json" 2>/dev/null))"
