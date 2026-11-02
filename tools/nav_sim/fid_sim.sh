#!/bin/bash
# Fidelity of the simulated chain to the real one, robot still, inside the `sim` image (same mounts/env as lio_sim.sh). usage: fid_sim.sh <out dir> [seconds=60]
#   1. /scan (Mid-360 -> FAST-LIO2 -> scan_median, the real chain) against the real calibration: scan_selfcheck.py (per range bin: core sigma, outlier rate, dropout; exit 1 if off)
#   2. the parked drift of FAST-LIO2 against the truth (lio_eval.py): what GN and the supervisor's parked-drift check see when nothing moves
set -eo pipefail
out=${1:?out dir}; secs=${2:-60}
source "${ROS_ENV:-/ros_env.sh}"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-56}
source /tools/nav_sim/sim_up.sh
lio_nodes
python3 /tools/nav_sim/lio_eval.py record "$out/parked.csv" "$secs" > "$out/rec.log" 2>&1 &
rec=$!
rc=0; python3 /tools/sim_realism/scan_selfcheck.py "$secs" /tools/sim_realism/calibration.json | tee "$out/scan_selfcheck.txt" || rc=$?
wait $rec || true
python3 /tools/nav_sim/lio_eval.py report "$out/parked.csv" | tee "$out/parked_report.txt"
exit $rc
