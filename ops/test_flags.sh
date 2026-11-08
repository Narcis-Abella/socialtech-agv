#!/bin/bash
# Run: bash ops/test_flags.sh
set -u
here=$(cd "$(dirname "$0")" && pwd)
source "$here/flags.sh"
fail=0
check() {   # check VALUE EXPECTED: the state of a flag after normalize_flags
  local got
  got=$(FLAG="$1" bash -c 'source "$0/flags.sh"; normalize_flags FLAG; [ -z "${FLAG:-}" ] && echo off || echo "on=$FLAG"' "$here")
  [[ $got == "$2" ]] || { echo "FAIL: FLAG='$1' -> $got, expected $2"; fail=1; }
}
check "" off; check 0 off; check false off; check False off; check NO off; check no off; check off off; check OFF off
check 1 on=1; check yes on=1; check true on=1; check on on=1; check anything on=1
unset FLAG; got=$(bash -c 'source "$0/flags.sh"; normalize_flags FLAG; [ -z "${FLAG:-}" ] && echo off || echo on' "$here")
[[ $got == off ]] || { echo "FAIL: unset FLAG -> $got"; fail=1; }
# HEAVY_DDS (a loopback-only URI for the driver) is untested with the supervisor, which reads the raw lidar and IMU on the default URI: refused together
grep -q '^normalize_flags .*HEAVY_DDS' "$here/nav_live.sh" && grep -qF '[[ -z ${HEAVY_DDS:-} ]] || die "HEAVY_DDS=1' "$here/nav_live.sh" || { echo "FAIL: nav_live.sh does not refuse HEAVY_DDS together with SUPERVISOR"; fail=1; }
# nav_live.sh must run SUPERVISOR through the same normalisation as NAV and BASE: SUPERVISOR=0 used to start the supervisor on a real robot
grep -q '^normalize_flags NAV BASE SUPERVISOR' "$here/nav_live.sh" || { echo "FAIL: nav_live.sh does not normalise SUPERVISOR"; fail=1; }
# PROD=1 (no recorder, no rviz aid): normalised like the other flags, forwarded into the container; viz:=false, the recorder guard and the plain exec hang on it (exact lines: a loose grep let
# an inverted or unconditional viz:=false and a neutralised recorder guard through)
nl="$here/nav_live.sh"
bash -n "$nl" || { echo "FAIL: nav_live.sh has a syntax error"; fail=1; }
grep -q '^normalize_flags NAV BASE SUPERVISOR PROD' "$nl" || { echo "FAIL: nav_live.sh does not normalise PROD"; fail=1; }
grep -qE '^exec docker run .* -e PROD="\$\{PROD:-\}"' "$nl" || { echo "FAIL: the docker run that starts the container does not forward PROD"; fail=1; }
grep -qxF '  [[ -z ${PROD:-} ]] || extra+=(viz:=false)' "$nl" || { echo "FAIL: viz:=false is not exactly 'PROD set -> viz:=false'"; fail=1; }
# viz:=false must come after the last assignment of extra (an earlier one is wiped by `extra=(...)`)
awk '/(^|;) *extra=\(/ {n = NR} /extra\+=\(viz:=false\)/ {v = NR} END {exit !(v > n)}' "$nl" || { echo "FAIL: viz:=false is added before extra is (re)assigned"; fail=1; }
# the recorder and its `rec=$!` sit together under the PROD guard, up to its closing fi
awk '/^    if \[\[ -z \$\{PROD:-\} \]\]; then   # PROD=1: no recording$/ {g = 1} g && /ros2 bag record/ {r = 1} g && /^      rec=\$!$/ {s = 1} g && /^    fi$/ {exit} END {exit !(r && s)}' "$nl" || { echo "FAIL: the bag recorder and rec=\$! are not both under the PROD guard"; fail=1; }
grep -qxF '  [[ -n ${NAV:-} && -z ${PROD:-} ]] || exec ros2 launch "${args[@]}"   # no recorder (PROD, or no NAV): nothing to stop after the launch' "$nl" || { echo "FAIL: the exec shortcut does not cover PROD"; fail=1; }
# LEVEL_CORRECTOR=1: normalised, forwarded into the container, level_corrector:=true added after extra is (re)assigned, and the image checked for the package before anything starts
grep -q '^normalize_flags NAV BASE SUPERVISOR PROD LEVEL_CORRECTOR' "$nl" || { echo "FAIL: nav_live.sh does not normalise LEVEL_CORRECTOR"; fail=1; }
grep -qE '^exec docker run .* -e LEVEL_CORRECTOR="\$\{LEVEL_CORRECTOR:-\}"' "$nl" || { echo "FAIL: the docker run does not forward LEVEL_CORRECTOR"; fail=1; }
grep -qxF '  [[ -z ${LEVEL_CORRECTOR:-} ]] || extra+=(level_corrector:=true)' "$nl" || { echo "FAIL: level_corrector:=true is not exactly 'LEVEL_CORRECTOR set -> level_corrector:=true'"; fail=1; }
awk '/(^|;) *extra=\(/ {n = NR} /extra\+=\(level_corrector:=true\)/ {v = NR} END {exit !(v > n)}' "$nl" || { echo "FAIL: level_corrector:=true is added before extra is (re)assigned"; fail=1; }
grep -q 'ros2 pkg prefix level_corrector' "$nl" || { echo "FAIL: nav_live.sh does not check the image for level_corrector"; fail=1; }
# inside the container the planar_odom that would run (an image older than the checkout has one that ignores it) must know correction_topic, or the corrector would run and change nothing
grep -qF 'grep -q correction_topic "$(ros2 pkg prefix planar_odom)/lib/planar_odom/planar_odom"' "$nl" || { echo "FAIL: nav_live.sh does not check that the effective planar_odom has correction_topic"; fail=1; }
# LEVEL_CORRECTOR is ON unless set to something off (the corrector is the robot's default since 2026-11-07); LEVEL_CORRECTOR=0 turns it off
grep -qxF 'LEVEL_CORRECTOR=${LEVEL_CORRECTOR-1}   # on by default: only an explicit LEVEL_CORRECTOR=0 (or off/false/no) turns it off' "$nl" || { echo "FAIL: LEVEL_CORRECTOR is not on by default"; fail=1; }
awk '/^LEVEL_CORRECTOR=\$\{LEVEL_CORRECTOR-1\}/ {d = NR} /^normalize_flags NAV BASE SUPERVISOR PROD LEVEL_CORRECTOR/ {n = NR} END {exit !(d && n && d < n)}' "$nl" || { echo "FAIL: the LEVEL_CORRECTOR default must be set before normalize_flags"; fail=1; }
for v in "" 1 0 off; do
  got=$(bash -c 'source "$0/flags.sh"; if [ "$1" = unset ]; then unset LEVEL_CORRECTOR; else LEVEL_CORRECTOR=$1; fi; LEVEL_CORRECTOR=${LEVEL_CORRECTOR-1}; normalize_flags LEVEL_CORRECTOR; [ -z "${LEVEL_CORRECTOR:-}" ] && echo off || echo on' "$here" "${v:-unset}")
  case "$v:$got" in ":on"|"1:on"|"0:off"|"off:off") ;; *) echo "FAIL: LEVEL_CORRECTOR='${v:-<unset>}' gave $got"; fail=1 ;; esac
done
# the corrector needs the 3D map: refuse before starting, not by dying silently at the node's construction
grep -qF '[[ -f $MAP/map.ply ]]' "$nl" && grep -qF '$MAP/map.yaml"; } || die "LEVEL_CORRECTOR needs' "$nl" || { echo "FAIL: nav_live.sh does not check map.ply and T_map_world of the map for LEVEL_CORRECTOR"; fail=1; }
# the full-chain sim runs the robot's configuration by default and refuses a planar_odom that would ignore the corrector
grep -qF 'LEVEL_CORRECTOR=${LEVEL_CORRECTOR-1}   # on by default, as ops/nav_live.sh' "$here/../tools/nav_sim/nav_full_sim.sh" || { echo "FAIL: nav_full_sim.sh does not default LEVEL_CORRECTOR on"; fail=1; }
grep -qF 'grep -q correction_topic "$(ros2 pkg prefix planar_odom)/lib/planar_odom/planar_odom"' "$here/../tools/nav_sim/nav_full_sim.sh" || { echo "FAIL: nav_full_sim.sh does not check that planar_odom has correction_topic"; fail=1; }
# the DEV bag keeps what is needed to diagnose the corrector offline
grep -qF 'lc_topics=(); [[ -z ${LEVEL_CORRECTOR:-} ]] || lc_topics=(/level_corrector/status /level_corrector/correction)' "$nl" && grep -qF '"${lc_topics[@]}"' "$nl" || { echo "FAIL: nav_live.sh does not record the level_corrector topics"; fail=1; }
# the sim reads LEVEL_CORRECTOR / SUPERVISOR like the robot (off/false/no/0 are off) and does not hide a python failure behind export
sim="$here/../tools/nav_sim/nav_full_sim.sh"
grep -qF 'normalize_flags LEVEL_CORRECTOR SUPERVISOR' "$sim" || { echo "FAIL: nav_full_sim.sh does not normalise LEVEL_CORRECTOR and SUPERVISOR"; fail=1; }
grep -qF 'if [ -n "${LEVEL_CORRECTOR:-}" ]; then' "$sim" || { echo "FAIL: nav_full_sim.sh tests the corrector flag as a string against 0"; fail=1; }
grep -q '^export SPAWN=\$(' "$sim" && { echo "FAIL: nav_full_sim.sh exports SPAWN=\$(python3 ...): a failing python is hidden by export"; fail=1; }
# the corrector's map is validated with the node's own loader before anything starts
grep -qF 'level_corrector_node --ros-args -p map_dir:=/map -p check_only:=true' "$nl" || { echo "FAIL: nav_live.sh does not dry-run the corrector's map loader"; fail=1; }
# the provenance block of nav_live.sh runs under set -e/pipefail on checkouts that are not git repositories (the Nano's is a plain copy): it must not end the script
d=$(mktemp -d /tmp/nogit.XXXXXX)
sed -n '/^{   # what ran:/,/^} > "\$RUN\/provenance.txt"/p' "$nl" >"$d/block.sh"
[[ -s $d/block.sh ]] || { echo "FAIL: the provenance block of nav_live.sh was not found"; fail=1; }
RUN=$d REPO=$d IMAGE=none SUPERVISOR= bash -euo pipefail -c "source $d/block.sh" || { echo "FAIL: the provenance block ends the script outside a git repository"; fail=1; }
grep -q '^checkout: unknown' "$d/provenance.txt" || { echo "FAIL: provenance.txt does not say unknown for a checkout without git"; fail=1; }
rm -r "$d"
((fail == 0)) && echo "ok"
exit $fail
