#!/usr/bin/env bash
# Generic development batch (NX layout): one query bag, a window CSV, a known map,
# a negative map and an evaluator-cut hole of the known map. Method containers never
# mount a reference. Env: V DS RAW WINCSV MAP_KNOWN MAP_NEG [VARIANTS="x01 cross hole"] [POS=x01]
set -u
B=/home/jetson/work/global-reloc-3d-20261009
C=$B/claude-v1
: "${V:?}" "${DS:?}" "${RAW:?}" "${WINCSV:?}" "${MAP_KNOWN:?}" "${MAP_NEG:?}"
POS=${POS:-x01}
VARIANTS=${VARIANTS:-"$POS cross hole"}
LIB="-v $B/small_gicp-build/libsmall_gicp.so:/opt/libsmall_gicp.so:ro"
mkdir -p $C/caps $C/inputs $C/search $C/holes-$V
tail -n +2 $WINCSV | tr -d "\r" | while IFS=, read w start offset dur ex ey cap rest; do
  id=$DS-$w
  capdir=$C/caps/$id
  if [ ! -d $capdir ]; then
    docker run --rm --network host -e ROS_DOMAIN_ID=$((150+RANDOM%40)) -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
      -v $RAW:/bag:ro -v $C/code:/tool:ro -v $C/caps:/out socialtech:robot-nav \
      bash /tool/capture_fastlio.sh /bag $offset ${cap:-11} /out/$id > /dev/null 2>&1
  fi
  for variant in $VARIANTS; do
    case $variant in
      cross) map=$MAP_NEG;;
      hole) map=$C/holes-$V/$id.xyz
            python3 $C/code/make_hole_map.py --map $MAP_KNOWN --x=$ex --y=$ey --radius 8 --out $map > $map.log;;
      *) map=$MAP_KNOWN;;
    esac
    in=$id-$variant-$V
    docker run --rm --network none -v $capdir/lio_bag:/capture:ro -v $RAW:/raw:ro -v $map:/map/map.xyz:ro \
      -v $C/code:/tool:ro -v $C/inputs:/out socialtech:mapper-verify python3 /tool/lio_submap.py --bag /capture \
      --map-xyz /map/map.xyz --start $start --duration $dur --imu-bag /raw --max-radius 2.8 --max-path 6 \
      --out /out/$in > $C/inputs/$in.log 2>&1
    docker run --rm --network none -v $C/inputs/$in:/input:ro -v $C/code:/tool:ro $LIB -v $C/search:/out \
      socialtech:mapper-verify bash -lc "export LD_LIBRARY_PATH=/opt:\$LD_LIBRARY_PATH; python3 /tool/search_v2.py --run /input --out /out/$in --binary /tool/register_v2" \
      > $C/search/$in.stdout 2>&1
    echo "$in $(grep -o '"decision": "[a-z_]*"' $C/search/$in/search.json 2>/dev/null)"
  done
done
