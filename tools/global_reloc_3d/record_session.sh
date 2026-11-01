#!/usr/bin/env bash
# Record one query/mapping session for the independent evaluation (see SESSION_JSON for the plan).
# NOT YET RUN ON HARDWARE: the driver must already be publishing /livox/lidar and /livox/imu. This script starts no driver and no motion.
# usage: record_session.sh SESSION_ID OUT_DIR SESSION_JSON
#   SESSION_JSON: metadata prepared beforehand; the recording refuses to start if a required field is missing or empty.
# While recording: type a stop ID + Enter at each stop (after braking); "q" + Enter ends the session.
set -euo pipefail
id=${1:?session id}; out=${2:?output dir}; meta=${3:?session.json}
[ -e "$out" ] && { echo "exists: $out (never overwrite a session)"; exit 1; }
python3 - "$meta" <<'PY'
import json, sys
m = json.load(open(sys.argv[1]))
need = ['session_id', 'date', 'operator', 'scored_against_map', 'mount_orientation', 'sensor_height_m', 'base_from_body_xyyaw',
        'mount_measured_by', 'mount_measured_date', 'mount_photos', 'driver_commit', 'image_digest', 'payload', 'floor_plan_revision', 'planned_stops']
missing = [k for k in need if m.get(k) in (None, '', [])]
if missing:
    sys.exit('session.json incomplete: ' + ', '.join(missing))
PY
for t in /livox/lidar /livox/imu; do
  timeout 5 ros2 topic echo --once "$t" --field header.stamp > /dev/null || { echo "no data on $t"; exit 1; }
done
mkdir -p "$out"; cp "$meta" "$out/session.json"
echo "stop_id,wall_time_s" > "$out/stops.csv"
ros2 bag record -s mcap -o "$out/bag" /livox/lidar /livox/imu > "$out/record.log" 2>&1 &
rec=$!
trap 'kill -INT $rec 2>/dev/null; wait $rec 2>/dev/null' EXIT
echo "recording $id -> $out/bag   (stop ID + Enter at each stop, q + Enter to end)"
while read -r stop; do
  [ "$stop" = q ] && break
  [ -n "$stop" ] && echo "$stop,$(date +%s.%N)" >> "$out/stops.csv" && echo "  marked $stop"
done
kill -INT $rec; wait $rec 2>/dev/null || true; trap - EXIT
python3 -B "$(dirname "$0")/bag_stream_integrity.py" "$out/bag" > "$out/integrity.json" && python3 - "$out/integrity.json" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
for k in ('/livox/imu', '/livox/lidar'):
    v = d[k]; print(k, 'n', v['n'], 'rate', round(v['rate_hz'], 1), 'missing %', round(100 * v['missing_fraction'], 2), 'max gap s', round(v['max_gap_s'], 3))
PY
echo "stops marked: $(($(wc -l < "$out/stops.csv") - 1))"
