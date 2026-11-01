#!/bin/bash
# Source it, then `fastlio_check FILE key=value ...`: 0 when every key is on exactly one line of FILE and has exactly that value. nav_live.sh edits the upstream
# FAST-LIO config with sed anchored on the key name only; this says so when the edit did not apply (a renamed or moved key), hit two lines, or left another value
# (a substring check passes for 0.255 when 0.25 was asked).
fastlio_check() {
  local f=$1 kv k v n
  shift
  for kv in "$@"; do
    k=${kv%%=*}; v=${kv#*=}
    n=$(grep -cE "^[[:space:]]*${k}:" "$f" || true)
    [[ $n == 1 ]] || { echo "FAST-LIO config: '$k' is on $n lines of $f (expected 1)" >&2; return 1; }
    grep -qE "^[[:space:]]*${k}:[[:space:]]*${v//./\\.}[[:space:]]*$" "$f" || { echo "FAST-LIO config: '$k' is not exactly $v in $f" >&2; return 1; }
  done
}
