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
unset CYCLONEDDS_URI
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

export CYCLONEDDS_URI='file:///etc/cyclone.xml,<CycloneDDS>mine</CycloneDDS>'
set_cyclone_uri
[[ $CYCLONEDDS_URI == *SocketReceiveBufferSize*,'file:///etc/cyclone.xml,<CycloneDDS>mine</CycloneDDS>' ]] || fail "a multi-fragment or file:// URI must be kept whole, after the buffer fragment: $CYCLONEDDS_URI"
unset CYCLONEDDS_URI
echo ok
