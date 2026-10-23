#!/bin/bash
# Factorial on the recorded real inputs: which AMCL term makes the default lose the pose (same inputs as real_sweeps.sh, condition track only).
# real_sweeps.sh header: AMCL variants on the recorded real inputs (FAST-LIO2 odometry + scan of the eco_-1_01 and eco_-1_03 bags, on their own maps, scored against the Voxel-SLAM trajectory of the same bag),
# the same variants as the simulator study, under four initial-pose conditions. Inside the sim image; mounts: tools at /tools, ~/nav_out at /navout (recorded inputs), ~/nav_data at /data, output at /out.
# usage: real_factorial.sh <out dir>      env: PAR (4)  RATE (5)  BAGS ("eco01 eco03")
set -eo pipefail
out=${1:?out dir}
nb=/tools/nav_bench
declare -A INPUTS=([eco01]=/navout/b_eco01_rec/inputs [eco03]=/navout/b_eco03_rec/inputs [eco01syn]=/navout/b_eco01_syn/inputs [eco03syn]=/navout/b_eco03_syn/inputs [hospital02]=/inputs/hospital02/inputs [zona01]=/inputs/zona01_v2/inputs [seat01]=/inputs/seat01_v2/inputs [zona05]=/inputs/zona05_v2/inputs [zona06]=/inputs/zona06_v2/inputs)   # *syn: /scan replaced by noise-free rays on the map (synth_scan.py)
declare -A MAPS=([eco01]=/data/eco_-1_01/map.yaml [eco03]=/data/eco_-1_03/final.yaml [eco01syn]=/data/eco_-1_01/map.yaml [eco03syn]=/data/eco_-1_03/final.yaml [hospital02]=/data/live_hospital_02/final.yaml [zona01]=/data/live_zona_01/final.yaml [seat01]=/data/live_seat_01/final.yaml [zona05]=/data/live_zona_05/final.yaml [zona06]=/data/live_zona_06/final.yaml)
declare -A REFS=([eco01]=/data/eco_-1_01/alidarState.txt [eco03]=/data/eco_-1_03/alidarState.txt [eco01syn]=/data/eco_-1_01/alidarState.txt [eco03syn]=/data/eco_-1_03/alidarState.txt [hospital02]=/data/live_hospital_02/alidarState.txt [zona01]=/data/live_zona_01/alidarState.txt [seat01]=/data/live_seat_01/alidarState.txt [zona05]=/data/live_zona_05/alidarState.txt [zona06]=/data/live_zona_06/alidarState.txt)
mkdir -p "$out"
# variant name -> overrides over the Nav2 defaults (no key twice in one config); "bench" is config/amcl.yaml as it is
CFG=$(python3 - <<'PY'
# one factor at a time over the Nav2 defaults (the "default" of real_sweeps.sh): which alpha term and which range limit make the default lose the pose
D = dict(alpha1=0.2, alpha2=0.2, alpha3=0.2, alpha4=0.2, update_min_d=0.25, update_min_a=0.2, z_hit=0.5, z_rand=0.5, max_beams=120, sigma_hit=0.2, min_particles=500, max_particles=2000)
V = {"default": {}, "default_r2": {}, "rot": dict(alpha1=0.01, alpha4=0.01), "trans": dict(alpha2=0.01, alpha3=0.01), "a1": dict(alpha1=0.01), "a4": dict(alpha4=0.01),
     "all01": dict(alpha1=0.01, alpha2=0.01, alpha3=0.01, alpha4=0.01), "rng12": dict(laser_max_range=12.0), "rng6": dict(laser_max_range=6.0)}
for name, v in V.items():
    print(name + "|" + " ".join(f"{k}={x}" for k, x in {**D, **v}.items()))
PY
)

mapfile -t CONFIGS <<< "$CFG"
# condition: name DIST DYAW INIT_STD INIT_YAW_STD
CONDS=("track 0 0 0.1 5")
k=0
for bag in ${BAGS:-eco01 eco03}; do
  for c in "${CONDS[@]}"; do
    read -r cname dist dyaw std ystd <<< "$c"
    k=$((k + 1))
    echo "== $bag $cname (DIST $dist DYAW $dyaw INIT_STD $std INIT_YAW_STD $ystd)" | tee -a "$out/progress.log"
    RATE=${RATE:-5} DIST=$dist DYAW=$dyaw INIT_STD=$std INIT_YAW_STD=$ystd DOMAIN_BASE=$((100 + 12 * k)) \
      "$nb/sweep.sh" "${INPUTS[$bag]}" "${MAPS[$bag]}" "${REFS[$bag]}" "$out/${bag}_$cname" "${PAR:-4}" "${CONFIGS[@]}" > "$out/${bag}_$cname.summary" 2>&1 || echo "sweep $bag $cname failed" | tee -a "$out/progress.log"
  done
done
echo done
