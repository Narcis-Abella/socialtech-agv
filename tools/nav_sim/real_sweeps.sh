#!/bin/bash
# AMCL variants on the recorded real inputs (FAST-LIO2 odometry + scan of the eco_-1_01 and eco_-1_03 bags, on their own maps, scored against the Voxel-SLAM trajectory of the same bag),
# the same variants as the simulator study, under four initial-pose conditions. Inside the sim image; mounts: tools at /tools, ~/nav_out at /navout (recorded inputs), ~/nav_data at /data, output at /out.
# usage: real_sweeps.sh <out dir>      env: PAR (4)  RATE (5)  BAGS ("eco01 eco03")
set -eo pipefail
out=${1:?out dir}
nb=/tools/nav_bench
declare -A INPUTS=([eco01]=/navout/b_eco01_rec/inputs [eco03]=/navout/b_eco03_rec/inputs)
declare -A MAPS=([eco01]=/data/eco_-1_01/map.yaml [eco03]=/data/eco_-1_03/final.yaml)
declare -A REFS=([eco01]=/data/eco_-1_01/alidarState.txt [eco03]=/data/eco_-1_03/alidarState.txt)
mkdir -p "$out"
# variant name -> overrides over the Nav2 defaults (no key twice in one config); "bench" is config/amcl.yaml as it is
CFG=$(python3 - <<'PY'
D = dict(alpha1=0.2, alpha2=0.2, alpha3=0.2, alpha4=0.2, update_min_d=0.25, update_min_a=0.2, z_hit=0.5, z_rand=0.5, max_beams=120, sigma_hit=0.2, min_particles=500, max_particles=2000)
A = lambda a: dict(alpha1=a, alpha2=a, alpha3=a, alpha4=a)
V = {"default": {}, "alpha01": A(0.01), "alpha05": {**A(0.05), "update_min_d": 0.1, "update_min_a": 0.1}, "update": dict(update_min_d=0.05, update_min_a=0.05),
     "sensor": dict(z_hit=0.95, z_rand=0.05, max_beams=180, sigma_hit=0.1), "particles": dict(min_particles=2000, max_particles=2000),
     "motion": {**A(0.01), "update_min_d": 0.05, "update_min_a": 0.05}}
for name, v in V.items():
    print(name + "|" + " ".join(f"{k}={x}" for k, x in {**D, **v}.items()))
print("bench|")
PY
)
mapfile -t CONFIGS <<< "$CFG"
# condition: name DIST DYAW INIT_STD INIT_YAW_STD
CONDS=("track 0 0 0.1 5" "off1 1 20 0.5 15" "off2 2 40 1.0 30" "off1tight 1 20 0.1 6")
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
