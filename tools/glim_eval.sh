#!/bin/bash
# Run glim_rosbag N times on one bag with one config (installed defaults + overlays), inside the robot image.
# GLIM is not deterministic (GPU, threads, workload throttling), so one run proves little.
#
# Usage: glim_eval.sh <bag_dir> <out_dir> <runs> <overlay.json>...
# Output: <out_dir>/config (the config used), overlays/, run<i>/ (dumps), run<i>.log, runs.tsv
# Then, outside the container: glim_metrics.py <out_dir>/run*/
set -euo pipefail

if (($# < 4)); then
  echo "usage: glim_eval.sh <bag_dir> <out_dir> <runs> <overlay.json>..." >&2
  exit 2
fi
if [[ ! -d $1 ]]; then
  echo "error: $1 is not a bag directory" >&2
  exit 1
fi
bag=$(realpath "$1")
out=$2
runs=$3
shift 3
defaults=${GLIM_DEFAULTS:-/opt/ros/jazzy/share/glim/config}

if [[ -e $out ]]; then
  echo "error: $out exists; use a new directory per config" >&2
  exit 1
fi
mkdir -p "$out/overlays"
out=$(realpath "$out")  # glim_rosbag resolves a relative config_path against the glim package
cp "$@" "$out/overlays/"
python3 "$(dirname "$0")/glim_config.py" "$defaults" "$out/config" "$@"

failed=0
printf 'run\texit\tseconds\n' >"$out/runs.tsv"
for ((i = 1; i <= runs; i++)); do
  start=$SECONDS
  rc=0
  ros2 run glim_ros glim_rosbag "$bag" --ros-args \
    -p config_path:="$out/config" -p dump_path:="$out/run$i" -p auto_quit:=true \
    >"$out/run$i.log" 2>&1 || rc=$?
  printf '%d\t%d\t%d\n' "$i" "$rc" $((SECONDS - start)) | tee -a "$out/runs.tsv"
  ((rc == 0)) || failed=1
done
exit $failed
