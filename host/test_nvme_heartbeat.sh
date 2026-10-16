#!/usr/bin/env bash
# Checks nvme-heartbeat.c and the heartbeat parts of setup_host.sh without a device, root or systemd.
# Run: bash host/test_nvme_heartbeat.sh   (needs gcc; ~15 s)
set -euo pipefail
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
fail() { echo "FAIL: $*" >&2; exit 1; }
tmp=$(mktemp -d -p "${TMPDIR:-/var/tmp}"); trap 'rm -r "$tmp"' EXIT

# ── the daemon's log, against a simulated device ──────────────────────────────────────────────────────────────────
# A read stuck for X ms (a lost MSI) must leave an event=slow_read line whatever else was logged meanwhile: it is the only
# thing that tells the operator the heartbeat did not heal the stall. After 1.6 s (32 reads of 50 ms) the queue is full
# and event=queue_full is logged every second, which must not hide it.
gcc -O2 -Wall -Wextra -Werror -pthread -Dsyscall=fake_syscall -o "$tmp/hb" "$here/nvme-heartbeat.c" "$here/test_nvme_heartbeat_aio.c"
head -c 131072 /dev/urandom >"$tmp/dev.bin"
if ! python3 -c 'import os,sys; os.close(os.open(sys.argv[1], os.O_RDONLY | os.O_DIRECT))' "$tmp/dev.bin" 2>/dev/null; then
  echo "SKIP daemon log tests: no O_DIRECT in ${TMPDIR:-/var/tmp}"
else
  stuck() {   # stuck <heal_ms>: log of one CPU whose reads are stuck until heal_ms
    HEAL_MS=$1 "$tmp/hb" "$tmp/dev.bin" 50 $(( $1 / 1000 + 2 )) 0 2>&1 || true
  }
  log=$(stuck 400);  [[ $log == *'event=slow_read cpu=0 ms=4'* ]]  || fail "400 ms stall not logged: $log"
  log=$(stuck 3300); [[ $log == *'event=queue_full'* ]]            || fail "test setup: the queue never filled: $log"
  [[ $log == *'event=slow_read cpu=0 ms=3'* ]]                     || fail "a 3.3 s stall was hidden by queue_full: $log"
  echo "daemon log ok"
fi

# ── setup_host.sh pieces, extracted from the real script ──────────────────────────────────────────────────────────
sh_fn() { sed -n "/^$1() {/,/^}/p" "$here/setup_host.sh"; }
# shellcheck disable=SC1090
source <(sh_fn nvme_hb_bad_lines; sh_fn nvme_hb_healing_ok)
systemctl() { return 0; }   # the service is active
check_log() {   # check_log <expect pass|fail> <description> <journal text>; journalctl_rc defaults to 0
  local want=$1 desc=$2 got=fail
  jtext=$3
  journalctl() { printf '%s' "$jtext"; return "${journalctl_rc:-0}"; }
  if nvme_hb_healing_ok; then got=pass; fi
  [[ $got == "$want" ]] || fail "health check: $desc: expected $want, got $got"
}
check_log pass 'start line only'          $'event=start file=/dev/nvme0n1 period_ms=50 cpus=0-11\n'
check_log pass 'a 150 ms heal'            $'event=slow_read cpu=3 ms=150.2\n'
check_log fail 'a 1744 ms read'           $'event=slow_read cpu=3 ms=1744.0\n'
check_log fail 'read_error'               $'event=read_error cpu=3 res=-5\n'
check_log fail 'submit_error'             $'event=submit_error cpu=3 errno=11\n'
check_log fail 'getevents_error'          $'event=getevents_error cpu=3 errno=22\n'
check_log fail 'queue_full (reads stuck for 1.6 s)' $'event=queue_full cpu=3 inflight=32\n'
journalctl_rc=1 check_log fail 'journalctl failing' ''
systemctl() { return 1; }
check_log fail 'service inactive'         ''
echo "health check ok"

# ── unit and step order ───────────────────────────────────────────────────────────────────────────────────────────
grep -q '^Conflicts=shutdown.target' "$here/setup_host.sh" && grep -q '^Before=shutdown.target' "$here/setup_host.sh" ||
  fail "the unit has DefaultDependencies=no: it must name Conflicts= and Before=shutdown.target itself"
apply=$(sed -n '/^if \[\[ \$MODE == apply/,/^fi/p' "$here/setup_host.sh")
[[ $(grep -n step_nvme_heartbeat <<<"$apply" | cut -d: -f1) -gt $(grep -n step_tailscale <<<"$apply" | cut -d: -f1) ]] ||
  fail "step_nvme_heartbeat dies on failure: it must run last so that it cannot skip the other steps (sudo hardening, tailscale)"
grep -q '0 = stop and disable' "$here/robot.conf.example" || fail "robot.conf.example: 0 only stops and disables, it does not remove"
echo ok
