#!/bin/bash
# Several AMCL configurations over the same recorded inputs, PAR at a time (own ROS_DOMAIN_ID each), then one summary table. Run inside the robot image.
# usage: sweep.sh <inputs_bag> <map.yaml> <alidarState.txt> <out_root> <PAR> "name|key=value key=value" ...     (a config with nothing after | is the baseline)
# env:   RATE (5)  DIST (1.0)  DYAW (20)
set -eo pipefail
bag=$1; map=$2; ref=$3; root=$4; par=$5; shift 5
tools=$(dirname "$(readlink -f "$0")")
mkdir -p "$root"
i=0
for cfg in "$@"; do
  name=${cfg%%|*}; kv=${cfg#*|}
  i=$((i+1))
  # shellcheck disable=SC2086
  ROS_DOMAIN_ID=$((100 + i)) "$tools/run_amcl_only.sh" "$bag" "$map" "$ref" "$root/$name" $kv > "$root/$name.stdout" 2>&1 &
  [ $(jobs -rp | wc -l) -ge "$par" ] && wait -n
done
wait
python3 "$tools/summarize.py" "$root" | tee "$root/summary.txt"
