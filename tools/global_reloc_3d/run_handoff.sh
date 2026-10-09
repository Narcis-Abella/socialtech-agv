#!/usr/bin/env bash
# Isolated handoff test (NX layout): accepted global pose -> /initialpose with covariance -> AMCL on replay.
# The handoff (pose + levelling) is computed truth-free; the reference reaches only the bench's final report.
# Env: [IMAGE=socialtech:robot-verify, needs the planar_odom package] NAME (search/input run) CAP (capture dir) RAW (bag dir) OFFSET (s, the capture offset) MAPYAML (PGM map) REF (reference
#      in that map's world) [PLAY_S=120] [INIT_STD=0.15] [INIT_YAW_STD=5] [H=0.53] [TAG=v1]
set -u
B=/home/jetson/work/global-reloc-3d-20261009; C=$B/claude-v1
: "${NAME:?}" "${CAP:?}" "${RAW:?}" "${OFFSET:?}" "${MAPYAML:?}" "${REF:?}"
out=$C/handoff/$NAME-${TAG:-v1}; mkdir -p $C/handoff
docker run --rm --network none -v $C/inputs/$NAME:/input:ro -v $C/search/$NAME:/search:ro -v $CAP/lio_bag:/capture:ro \
  -v $C/code:/tool:ro -v $C/handoff:/out socialtech:mapper-verify python3 /tool/handoff_pose.py --run /input --search /search \
  --capture /capture --out /out/$NAME-${TAG:-v1}.handoff.json > /dev/null 2>&1 || { echo "handoff failed: $NAME"; exit 1; }
read -r init level < <(python3 -c "import json,sys; d=json.load(open(sys.argv[1])); print(d['INIT_XYYAW'].replace(' ',','), d['LEVEL_Q'].replace(' ',','))" $C/handoff/$NAME-${TAG:-v1}.handoff.json)
mapdir=$(dirname $MAPYAML)
docker run --rm --cpus 4 --network host -e ROS_DOMAIN_ID=$((120+RANDOM%20)) -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e INIT_XYYAW="${init//,/ }" -e LEVEL_Q="${level//,/ }" -e INIT_STD=${INIT_STD:-0.15} -e INIT_YAW_STD=${INIT_YAW_STD:-5} \
  -e PLAY_ARGS="--start-offset $OFFSET --playback-duration ${PLAY_S:-120}" \
  -v $RAW:/bag:ro -v $mapdir:/map:ro -v $REF:/ref/alidarState.txt:ro -v $C/nav_bench:/nb:ro -v $C/handoff:/out \
  ${IMAGE:-socialtech:robot-verify} /nb/run_bench_a.sh /bag /map/$(basename $MAPYAML) /ref/alidarState.txt ${H:-0.53} /out/$NAME-${TAG:-v1} \
  > $C/handoff/$NAME-${TAG:-v1}.log 2>&1
tail -25 $C/handoff/$NAME-${TAG:-v1}.log
