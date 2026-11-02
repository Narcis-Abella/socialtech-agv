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
((fail == 0)) && echo "ok"
exit $fail
