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
((fail == 0)) && echo "ok"
exit $fail
