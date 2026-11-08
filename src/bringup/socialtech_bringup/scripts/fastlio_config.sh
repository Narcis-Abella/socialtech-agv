# Source it, then `fastlio_yaml OUT`: the robot's FAST-LIO2 parameters = the upstream mid360.yaml with these edits (run_live on the robot, tools/nav_sim/sim_up.sh in the simulation: one copy).
#   env: GYR_COV (0.01)  FILTER_SIZE_MAP (0.25)  POINT_FILTER_NUM (2); empty = the default. Returns 1, with the reason on stderr, when a value is invalid or the edit did not apply.
# shellcheck source=fastlio_check.sh
source "$(dirname "${BASH_SOURCE[0]}")/fastlio_check.sh"
fastlio_yaml() {
  local out=$1 fsm=${FILTER_SIZE_MAP:-0.25} pfn=${POINT_FILTER_NUM:-2} gyr=${GYR_COV:-0.01}   # env, like GYR_COV: FILTER_SIZE_MAP=0.5 POINT_FILTER_NUM=3 are the upstream values
  [[ $fsm =~ ^[0-9]+(\.[0-9]+)?$ && $fsm != 0 && $pfn =~ ^[1-9][0-9]*$ ]] || { echo "FAIL: FILTER_SIZE_MAP='$fsm' must be a positive number and POINT_FILTER_NUM='$pfn' a positive integer" >&2; return 1; }
  sed -e "s|pcd_save_en:.*|pcd_save_en: false|" -e "s|map_en:.*|map_en: false|" -e "s|path_en:.*|path_en: false|" \
      -e "s|^\( *\)gyr_cov:.*|\1gyr_cov: $gyr|" -e "s|^\( *\)acc_cov:.*|\1acc_cov: 1.0|" -e "s|^\( *\)blind:.*|\1blind: 0.1|" \
      -e "s|^\( *\)filter_size_map:.*|\1filter_size_map: $fsm|" -e "s|^\( *\)point_filter_num:.*|\1point_filter_num: $pfn|" \
      "$(ros2 pkg prefix fast_lio)/share/fast_lio/config/mid360.yaml" > "$out"   # gyr_cov (env GYR_COV, default 0.01: parked yaw drift -25 %, live 2026-10-31; 1.0 = the bench value) / acc_cov 1.0 (upstream 0.1 degrades in featureless corridors)
  # filter_size_map 0.25 (upstream 0.5) + point_filter_num 2 (upstream 3), env FILTER_SIZE_MAP / POINT_FILTER_NUM (0.5 / 3 go back). Evidence: replays of 3 live passes of
  # one house (238 s each, deterministic, one replay per condition), not a run on the robot. Hypothesis: the ICP was weakly constrained in pitch/roll. In them the pitch
  # wobble while driving fell 1.5-2.1 -> 0.2-0.3 deg, isolated /scan flashes -44..-79 %, FAST-LIO CPU 17.5 -> 20.3 s per 238 s replay ON THE AGX ONLY: not measured on
  # the NX or the Nano, where a denser map and 1 of 2 points may not keep up with the 10 Hz of the lidar (SocialTech notes/ruido_lidar_2026-10-31.md).
  fastlio_check "$out" filter_size_map="$fsm" point_filter_num="$pfn" gyr_cov="$gyr" acc_cov=1.0 blind=0.1 || { echo "FAIL: the FAST-LIO config edit did not apply" >&2; return 1; }
}
