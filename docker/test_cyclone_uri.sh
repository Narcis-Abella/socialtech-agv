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
[[ $uri == *'SocketReceiveBufferSize min="10MB"'* ]] || fail "no receive buffer request: $uri"
[[ $uri != *NetworkInterface* ]] || fail "interface pinned without DDS_IFACE: $uri"

DDS_IFACE=wlP1p1s0 uri=$(DDS_IFACE=wlP1p1s0 cyclone_uri)
wellformed <<<"$uri" || fail "not well-formed XML with DDS_IFACE: $uri"
[[ $uri == *'<NetworkInterface name="wlP1p1s0"/>'* && $uri == *'SocketReceiveBufferSize min="10MB"'* ]] || fail "interface and buffer expected together: $uri"

unset CYCLONEDDS_URI
set_cyclone_uri
[[ ${CYCLONEDDS_URI:-} == *SocketReceiveBufferSize* ]] || fail "set_cyclone_uri did not set the URI"
export CYCLONEDDS_URI='<CycloneDDS>mine</CycloneDDS>'
set_cyclone_uri
[[ $CYCLONEDDS_URI == '<CycloneDDS>mine</CycloneDDS>' ]] || fail "an explicit CYCLONEDDS_URI must win"
echo ok
