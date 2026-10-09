#!/usr/bin/env bash
# Host side (NX): one containerised end-to-end run. Env: NAME BAG MAPYAML MAPXYZ OFFSET TAG [PLAY_S MODE H MAX_ATTEMPTS KEEP_PLAYING LEAD PRE]
# Output: $R/runs/$NAME-$TAG (new each time: docker writes as root). Also samples GPU/power with host tegrastats.
set -u
R=${R:-/home/jetson/work/reloc-online-20261009}
B=/home/jetson/work/global-reloc-3d-20261009
: "${NAME:?}" "${BAG:?}" "${MAPYAML:?}" "${MAPXYZ:?}" "${OFFSET:?}" "${TAG:?}"
out=$R/runs/$NAME-$TAG
[ -e "$out" ] && { echo "exists: $out"; exit 1; }
mkdir -p "$out"
echo "jetson_clocks: $(systemctl is-active jetson-clocks)  load: $(cut -d' ' -f1-3 /proc/loadavg)  start: $(date -Is)" > "$out/host.txt"
tegrastats --interval 500 > "$out/tegrastats.log" 2>/dev/null & teg=$!
docker run --rm --network host --cpus 8 -e ROS_DOMAIN_ID=$((100+RANDOM%100)) \
  -e BAG=/bag -e MAPYAML=/map/$(basename "$MAPYAML") -e MAPXYZ=/prior/map.xyz -e OUT=/out -e OFFSET=$OFFSET \
  -e PLAY_S=${PLAY_S:-100} -e MODE=${MODE:-s3} -e H=${H:-0.53} -e MAX_ATTEMPTS=${MAX_ATTEMPTS:-1} -e KEEP_PLAYING=${KEEP_PLAYING:-0} -e LEAD=${LEAD:-12} -e PRE=${PRE:-2.5} \
  -v "$BAG":/bag:ro -v "$(dirname "$MAPYAML")":/map:ro -v "$MAPXYZ":/prior/map.xyz:ro -v $R/tools:/tools:ro \
  -v $B/small_gicp-build/libsmall_gicp.so:/opt/libsmall_gicp.so:ro -v "$out":/out socialtech:reloc-online bash /tools/global_reloc_3d/online_inner.sh > "$out/inner.log" 2>&1
echo "exit=$?" >> "$out/host.txt"
kill $teg 2>/dev/null; wait $teg 2>/dev/null
tail -3 "$out/inner.log"
