#!/usr/bin/env bash
# Host side (NX): one containerised end-to-end run. Env: NAME BAG MAPYAML MAPXYZ OFFSET TAG [PLAY_S MODE H MAX_ATTEMPTS KEEP_PLAYING LEAD PRE]
# Output: $R/runs/$NAME-$TAG (new each time: docker writes as root). Also samples GPU/power with host tegrastats.
# DOCKER_CPUS (default 8): CPU quota of the container, for the contention runs of phase 3 (note 23).
set -u
R=${R:-/home/jetson/work/reloc-online-20261009}
B=/home/jetson/work/global-reloc-3d-20261009
: "${NAME:?}" "${BAG:?}" "${MAPYAML:?}" "${MAPXYZ:?}" "${OFFSET:?}" "${TAG:?}"
out=$R/runs/$NAME-$TAG
[ -e "$out" ] && { echo "exists: $out"; exit 1; }
mkdir -p "$out"
# Mount the map YAML and its image only. The map directory also holds the Voxel-SLAM reference (vs/run/alidarState.txt), which must
# never be visible to the method container.
img=$(dirname "$MAPYAML")/$(sed -n 's/^image: *//p' "$MAPYAML" | tr -d "\"'")
[ -f "$img" ] || { echo "map image not found: $img"; exit 1; }
echo "jetson_clocks: $(systemctl is-active jetson-clocks)  load: $(cut -d' ' -f1-3 /proc/loadavg)  start: $(date -Is)" > "$out/host.txt"
tegrastats --interval 500 > "$out/tegrastats.log" 2>/dev/null & teg=$!
dom=${DOMAIN:-$((100+RANDOM%100))}; echo "ros_domain_id: $dom" >> "$out/host.txt"
F2ENV=""; for v in WITH_GATE STOP_AT NO_PLAY KILL_PLAY_AT KILL_PLAY_SIG CONT_PLAY_AT KILL_AMCL_AT KILL_AMCL_SIG SIG_NODE_AT SIG_NODE_SIG CONT_NODE_AT KILL_GATE_AT CLOCK_JUMP_AT CLOCK_JUMP_SEC ROGUE_POSE_AT ROGUE_X ROGUE_Y ROGUE_PERMIT_AT AFTER_LOST_S AUTH_TIMES KEEP_FAILED_CHECKS; do
  [ -n "${!v:-}" ] && F2ENV="$F2ENV -e $v=${!v}"; done
docker run --rm --network host --cpus ${DOCKER_CPUS:-8} -e ROS_DOMAIN_ID=$dom $F2ENV \
  -e BAG=/bag -e MAPYAML=/map/$(basename "$MAPYAML") -e MAPXYZ=/prior/map.xyz -e OUT=/out -e OFFSET=$OFFSET \
  -e PLAY_S=${PLAY_S:-100} -e MODE=${MODE:-s3} -e H=${H:-0.53} -e MAX_ATTEMPTS=${MAX_ATTEMPTS:-1} -e KEEP_PLAYING=${KEEP_PLAYING:-0} -e LEAD=${LEAD:-12} -e PRE=${PRE:-2.5} -e REGISTER_BIN=${REGISTER_BIN:-} -e LAUNCH_EXTRA="${LAUNCH_EXTRA:-}" -e RELAY_SPEC="${RELAY_SPEC:-}" -e PLAY_ARGS_EXTRA="${PLAY_ARGS_EXTRA:-}" -e KILL_NODE_AT=${KILL_NODE_AT:-} -e KILL_LIO_AT=${KILL_LIO_AT:-} -e KILL_LIO_SIG=${KILL_LIO_SIG:-} -e AUTH_AT=${AUTH_AT:-} -e NODE2_ARGS="${NODE2_ARGS:-}" \
  -v "$BAG":/bag:ro -v "$MAPYAML":/map/$(basename "$MAPYAML"):ro -v "$img":/map/$(basename "$img"):ro -v "$MAPXYZ":/prior/map.xyz:ro -v $R/tools:/tools:ro \
  -v $B/small_gicp-build/libsmall_gicp.so:/opt/libsmall_gicp.so:ro -v "$out":/out socialtech:reloc-online bash /tools/global_reloc_3d/online_inner.sh > "$out/inner.log" 2>&1
echo "exit=$?" >> "$out/host.txt"
kill $teg 2>/dev/null; wait $teg 2>/dev/null
tail -3 "$out/inner.log"
