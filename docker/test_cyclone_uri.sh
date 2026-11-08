#!/usr/bin/env bash
# Checks cyclone_uri.sh without ROS or a container. Run: bash docker/test_cyclone_uri.sh
set -euo pipefail
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=cyclone_uri.sh
source "$here/cyclone_uri.sh"
fail() { echo "FAIL: $*" >&2; exit 1; }
wellformed() { python3 -c 'import sys, xml.etree.ElementTree as E; E.fromstring(sys.stdin.read())'; }

unset DDS_IFACE CYCLONEDDS_URI
uri=$(cyclone_uri)
wellformed <<<"$uri" || fail "not well-formed XML without DDS_IFACE: $uri"
[[ $uri == *"SocketReceiveBufferSize min=\"${DDS_RCVBUF_MIN}B\""* ]] || fail "no receive buffer request: $uri"
[[ $uri != *NetworkInterface* ]] || fail "interface pinned without DDS_IFACE: $uri"

uri=$(DDS_IFACE=wlP1p1s0 cyclone_uri)
wellformed <<<"$uri" || fail "not well-formed XML with DDS_IFACE: $uri"
[[ $uri == *'<NetworkInterface name="wlP1p1s0"/>'* && $uri == *"SocketReceiveBufferSize min=\"${DDS_RCVBUF_MIN}B\""* ]] || fail "interface and buffer expected together: $uri"

unset CYCLONEDDS_URI
set_cyclone_uri
[[ ${CYCLONEDDS_URI:-} == *SocketReceiveBufferSize* ]] || fail "set_cyclone_uri did not set the URI"
export CYCLONEDDS_URI='<CycloneDDS>mine</CycloneDDS>'
set_cyclone_uri
[[ $CYCLONEDDS_URI == *'<CycloneDDS>mine</CycloneDDS>' ]] || fail "the caller's URI must stay, and last (Cyclone lets the last fragment win): $CYCLONEDDS_URI"
[[ $CYCLONEDDS_URI == *SocketReceiveBufferSize*,'<CycloneDDS>mine'* ]] || fail "an explicit URI must still get the buffer request, before it: $CYCLONEDDS_URI"
before=$CYCLONEDDS_URI; set_cyclone_uri
[[ $CYCLONEDDS_URI == "$before" ]] || fail "set_cyclone_uri must be idempotent (interactive shells source it twice): $CYCLONEDDS_URI"
export CYCLONEDDS_URI='<CycloneDDS><Domain><Internal><SocketReceiveBufferSize min="1MB"/></Internal></Domain></CycloneDDS>'
before=$CYCLONEDDS_URI; set_cyclone_uri
[[ $CYCLONEDDS_URI == "$before" ]] || fail "a caller that sets its own buffer must be left alone: $CYCLONEDDS_URI"

# rmem_max preflight: Cyclone needs the kernel to grant the request, and the grant is 2x min(request, rmem_max)
unset CYCLONEDDS_URI; set_cyclone_uri   # the URI the entrypoint sees when it runs the check
f=$(mktemp); trap 'rm -f "$f"' EXIT
echo 212992 >"$f"
msg=$(RMEM_MAX_FILE=$f rmem_max_warning 2>&1) && fail "rmem_max 212992 must warn"
[[ $msg == *rmem_max* && $msg == *setup_host.sh* ]] || fail "the warning must name rmem_max and setup_host.sh: $msg"
echo 67108864 >"$f"
msg=$(RMEM_MAX_FILE=$f rmem_max_warning 2>&1) || fail "rmem_max 64 MB must pass: $msg"
[[ -z $msg ]] || fail "no output expected when rmem_max is enough: $msg"
RMEM_MAX_FILE=/nonexistent rmem_max_warning >/dev/null || fail "an unreadable rmem_max must not warn (non-Linux hosts, odd containers)"
echo abc >"$f"
RMEM_MAX_FILE=$f rmem_max_warning >/dev/null 2>&1 || fail "a non-numeric rmem_max must not warn"
echo $(( (DDS_RCVBUF_MIN + 1) / 2 )) >"$f"   # the grant is 2x the limit: half the request is exactly enough
RMEM_MAX_FILE=$f rmem_max_warning >/dev/null 2>&1 || fail "rmem_max at half the request must pass"
echo $(( (DDS_RCVBUF_MIN + 1) / 2 - 1 )) >"$f"
! RMEM_MAX_FILE=$f rmem_max_warning >/dev/null 2>&1 || fail "rmem_max one below half the request must warn"
# a caller that sets its own buffer is not bound to our request: Cyclone only aborts on the min it was given
echo 212992 >"$f"
export CYCLONEDDS_URI='<CycloneDDS><Domain><Internal><SocketReceiveBufferSize min="1MB"/></Internal></Domain></CycloneDDS>'
set_cyclone_uri
RMEM_MAX_FILE=$f rmem_max_warning >/dev/null 2>&1 || fail "a caller's own buffer min must not trigger our warning: $CYCLONEDDS_URI"
export CYCLONEDDS_URI='<CycloneDDS><Domain><Internal><SocketReceiveBufferSize max="1MB"/></Internal></Domain></CycloneDDS>'
set_cyclone_uri
RMEM_MAX_FILE=$f rmem_max_warning >/dev/null 2>&1 || fail "a caller's buffer without min must not trigger our warning: $CYCLONEDDS_URI"
export CYCLONEDDS_URI='<CycloneDDS>mine</CycloneDDS>'
set_cyclone_uri
! RMEM_MAX_FILE=$f rmem_max_warning >/dev/null 2>&1 || fail "an explicit URI without a buffer still gets our request, so it must warn: $CYCLONEDDS_URI"

export CYCLONEDDS_URI='file:///etc/cyclone.xml,<CycloneDDS>mine</CycloneDDS>'
set_cyclone_uri
[[ $CYCLONEDDS_URI == *SocketReceiveBufferSize*,'file:///etc/cyclone.xml,<CycloneDDS>mine</CycloneDDS>' ]] || fail "a multi-fragment or file:// URI must be kept whole, after the buffer fragment: $CYCLONEDDS_URI"
# every script that exports its own URI inside the container, after /ros_env.sh, replaces the one that carries the buffer request: it must ask again right after
# (nav_live.sh --inside did not until 2026-11-08: the whole live stack ran without the 10 MB buffer). `docker exec -e CYCLONEDDS_URI` (nav_exec.sh) is fine: ros_env.sh runs after it.
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
for script in "$repo"/ops/nav_live.sh "$repo"/src/bringup/socialtech_bringup/scripts/run_live "$repo"/tools/nav_sim/*.sh; do
  grep -q 'export.*CYCLONEDDS_URI=' "$script" || continue
  # every export of the URI is followed (comments and blank lines aside; consecutive exports count as one) by set_cyclone_uri before any other statement
  awk -v f="$script" '/^[[:space:]]*#/ || /^[[:space:]]*$/ {next} /export.*CYCLONEDDS_URI=/ {pending = 1; next} /^[[:space:]]*set_cyclone_uri/ {pending = 0; next} pending {print f ": an export of CYCLONEDDS_URI is followed by: " $0; bad = 1; pending = 0} END {if (pending) bad = 1; exit bad}' "$script" || fail "$script replaces the URI without asking for the receive buffer again"
done
# stack_cyclone_uri: lo first, DDS_IFACE and the unicast peer when given, localhost always, the interface once
u=$(stack_cyclone_uri lo); [[ $u == '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General><Discovery><Peers><Peer address="localhost"/></Peers><MaxAutoParticipantIndex>30</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>' ]] || fail "loopback-only URI: $u"
[[ $(stack_cyclone_uri "" "") == "$u" ]] || fail "no interface and no peer must be loopback only"
u=$(stack_cyclone_uri wlan0 192.168.1.53); [[ $u == *'<NetworkInterface name="lo"/><NetworkInterface name="wlan0"/>'* && $u == *'<Peer address="192.168.1.53"/><Peer address="localhost"/>'* ]] || fail "interface and peer: $u"
[[ $(stack_cyclone_uri lo 10.0.0.2) == *'<Peer address="10.0.0.2"/><Peer address="localhost"/>'* && $(stack_cyclone_uri lo 10.0.0.2) != *'name="lo"/><NetworkInterface name="lo"'* ]] || fail "lo twice or peer lost"
[[ $(stack_cyclone_uri lo "" 40) == *'<MaxAutoParticipantIndex>40<'* ]] || fail "the participant index must be settable"
unset CYCLONEDDS_URI
echo ok
