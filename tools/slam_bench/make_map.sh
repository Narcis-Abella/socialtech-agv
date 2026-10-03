#!/bin/bash
# Bag -> 2D map, inside the `mapper` image: Voxel-SLAM -> world PLY -> ply_to_map (floor, levelling, wall alignment; refuses a map whose floor does not fit the trajectory)
# -> map_visibility (cleans the map with the raw scans) -> map.pgm + map.yaml (with T_map_world) + map_clean.ply (the cloud the PGM was made from) + report.json.
# world.ply is the raw Voxel-SLAM cloud (final poses, 5 cm thinning): no levelling, no wall alignment, no floor/ceiling removal, no cleaning.
# usage: make_map.sh <bag_dir> <out_dir>
# env:   LIDAR (/livox/lidar)  IMU (/livox/imu)  PLAY_RATE (1.0)  CEIL (1.80, metres above the floor; 30 for outdoor maps)  VOXEL (0.05)  DEGRADE_BOUND (100000)
#        VOXELSLAM_RUN VS_TO_PLY PLY_TO_MAP MAP_VISIBILITY MAP_REPORT: the tool each step runs (default: the one next to this script; the tests swap them)
# On any failure it exits non-zero naming the step, and leaves no map.pgm / map.yaml in <out_dir>.
set -uo pipefail
[ $# -eq 2 ] || { echo "usage: make_map.sh <bag_dir> <out_dir>" >&2; exit 2; }
bag=$1; out=$2
here=$(dirname "$(readlink -f "$0")")
: "${LIDAR:=/livox/lidar}" "${IMU:=/livox/imu}" "${PLAY_RATE:=1.0}" "${CEIL:=1.80}" "${VOXEL:=0.05}" "${DEGRADE_BOUND:=100000}"
: "${VOXELSLAM_RUN:=$here/voxelslam_run.sh}" "${VS_TO_PLY:=python3 $here/vs_to_ply.py}" "${PLY_TO_MAP:=python3 $here/ply_to_map.py}"
: "${MAP_VISIBILITY:=python3 $here/map_visibility.py}" "${MAP_REPORT:=python3 $here/map_report.py}"
fail() { echo "make_map: FAILED at step '$1': $2" >&2; exit 1; }

mkdir -p "$out" "$out/.build"
rm -f "$out/map.pgm" "$out/map.yaml" "$out/map_clean.ply" "$out/report.json" "$out/.build/map.pgm" "$out/.build/map.yaml" "$out/.build/map_clean.ply"   # a map of an earlier run must not pass for this one

printf 's/^\\( *\\)degrade_bound:.*/\\1degrade_bound: %s/\n' "$DEGRADE_BOUND" > "$out/voxelslam.sed"
VS_SED="$out/voxelslam.sed" PLAY_RATE="$PLAY_RATE" $VOXELSLAM_RUN "$bag" "$LIDAR" "$IMU" "$out" > "$out/voxelslam_run.log" 2>&1 \
  || fail voxelslam "see $out/voxelslam_run.log and $out/node.log"
[ -s "$out/vs/run/alidarState.txt" ] || fail voxelslam "no poses were written (see $out/node.log)"
$VS_TO_PLY "$out/vs/run" "$out/world.ply" "$VOXEL" > "$out/vs_to_ply.log" 2>&1 || fail vs_to_ply "see $out/vs_to_ply.log"
$PLY_TO_MAP "$out/world.ply" "$out/base" --ceil "$CEIL" --traj "$out/vs/run/alidarState.txt" > "$out/ply_to_map.log" 2>&1 \
  || fail ply_to_map "$(tail -n 1 "$out/ply_to_map.log")"
$MAP_VISIBILITY "$bag" "$out/vs/run/alidarState.txt" "$out/world.ply" "$out/.build/map" --topic "$LIDAR" --ply-out "$out/.build/map_clean.ply" > "$out/map_visibility.log" 2>&1 \
  || fail map_visibility "see $out/map_visibility.log"
mv "$out/.build/map.pgm" "$out/map.pgm" && mv "$out/.build/map.yaml" "$out/map.yaml" && mv "$out/.build/map_clean.ply" "$out/map_clean.ply" \
  || fail map_visibility "it wrote no map.pgm / map.yaml / map_clean.ply"
$MAP_REPORT "$out" > /dev/null || fail report "see the message above"
echo "make_map: OK -> $out/map.yaml ($(python3 -c "import json,sys; r=json.load(open(sys.argv[1])); print('resets', r['resets'], '| end-to-start', r['end_to_start_m'], 'm | sensor height', r['sensor_height_m'], 'm')" "$out/report.json" 2>/dev/null))"
