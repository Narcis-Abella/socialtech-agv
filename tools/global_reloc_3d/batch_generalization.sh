#!/usr/bin/env bash
# Development batch on the NX (paths are the NX layout). Method containers never mount
# a reference; holes are cut by the evaluator-side helper before the method runs.
set -u
B=/home/jetson/work/global-reloc-3d-20261009
C=$B/claude-v1
V=${V:?set a version tag}
M=/home/jetson/map_out
R=/home/jetson/rosbags/custom
LIB="-v $B/small_gicp-build/libsmall_gicp.so:/opt/libsmall_gicp.so:ro"
declare -A RAW=([eco01_1x]=mapping_eco_-1_01_custom [hospital_02_r05b]=hospital_02_custom)
declare -A MAP=([eco01_1x]=$B/isolated-v16/map/map.xyz [hospital_02_r05b]=$B/point7-v1/hospital-lio-s2-w01/map.xyz)
declare -A OTHER=([eco01_1x]=hospital_02_r05b [hospital_02_r05b]=eco01_1x)
mkdir -p $C/caps $C/inputs $C/search $C/holes-$V
for ds in ${DATASETS:-eco01_1x hospital_02_r05b}; do
  tail -n +2 $C/windows/$ds-s5.csv | tr -d "\r" | while IFS=, read w start offset dur ex ey; do
    id=$ds-$w
    cap=$C/caps/$id
    if [ ! -d $cap ]; then
      docker run --rm --network host -e ROS_DOMAIN_ID=$((190+10#${w#w}+${#ds})) -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
        -v $R/${RAW[$ds]}:/bag:ro -v $C/code:/tool:ro -v $C/caps:/out socialtech:robot-nav \
        bash /tool/capture_fastlio.sh /bag $offset 11 /out/$id > /dev/null 2>&1
    fi
    python3 $C/code/make_hole_map.py --map ${MAP[$ds]} --x=$ex --y=$ey --radius 8 --out $C/holes-$V/$id.xyz > $C/holes-$V/$id.log
    for variant in ${VARIANTS:-own cross hole}; do
      case $variant in own) map=${MAP[$ds]};; cross) map=${MAP[${OTHER[$ds]}]};; hole) map=$C/holes-$V/$id.xyz;; esac
      in=$id-$variant-$V
      docker run --rm --network none -v $cap/lio_bag:/capture:ro -v $R/${RAW[$ds]}:/raw:ro -v $map:/map/map.xyz:ro \
        -v $C/code:/tool:ro -v $C/inputs:/out socialtech:mapper-verify python3 /tool/lio_submap.py --bag /capture \
        --map-xyz /map/map.xyz --start $start --duration $dur --imu-bag /raw --max-radius 2.8 --max-path 6 \
        --out /out/$in > $C/inputs/$in.log 2>&1
      docker run --rm --network none -v $C/inputs/$in:/input:ro -v $C/code:/tool:ro $LIB -v $C/search:/out \
        socialtech:mapper-verify bash -lc "export LD_LIBRARY_PATH=/opt:\$LD_LIBRARY_PATH; python3 /tool/search_v2.py --run /input --out /out/$in --binary /tool/register_v2" \
        > $C/search/$in.stdout 2>&1
      echo "$in $(grep -o '"decision": "[a-z_]*"' $C/search/$in/search.json 2>/dev/null)"
    done
  done
done
