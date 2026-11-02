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
# PROD=1 (no recorder, no rviz aid): normalised like the other flags, forwarded into the container, and the bag recorder and viz:=false hang on it
grep -q '^normalize_flags NAV BASE SUPERVISOR PROD' "$here/nav_live.sh" || { echo "FAIL: nav_live.sh does not normalise PROD"; fail=1; }
grep -q -- '-e PROD="${PROD:-}"' "$here/nav_live.sh" || { echo "FAIL: nav_live.sh does not forward PROD into the container"; fail=1; }
grep -q 'viz:=false' "$here/nav_live.sh" || { echo "FAIL: nav_live.sh never passes viz:=false"; fail=1; }
grep -B4 'ros2 bag record' "$here/nav_live.sh" | grep -q 'z ${PROD:-}' || { echo "FAIL: the bag recorder is not guarded by PROD"; fail=1; }
((fail == 0)) && echo "ok"
exit $fail
