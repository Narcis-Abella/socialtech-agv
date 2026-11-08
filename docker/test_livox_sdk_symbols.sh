#!/bin/bash
# The Livox SDK must neither export nor import a spdlog/fmt symbol (docker/livox_sdk_hide_spdlog.map says why). Run by the image build right after the SDK install; by hand:
#   docker run --rm --entrypoint bash -v "$PWD/docker:/t" socialtech:robot /t/test_livox_sdk_symbols.sh [LIB]
set -euo pipefail
cache=$(ldconfig -p)           # a variable, not a pipe: with pipefail an early-exiting awk/grep -q would fail the pipeline with SIGPIPE
lib=${1:-$(sed -n 's/.*liblivox_lidar_sdk_shared\.so .*=> //p' <<<"$cache" | head -n1 || true)}
[[ -n $lib && -f $lib ]] || { echo "FAIL: liblivox_lidar_sdk_shared.so not found (ldconfig -p)"; exit 1; }
n_copies=$(grep -c 'liblivox_lidar_sdk_shared\.so ' <<<"$cache" || true)
[[ $n_copies == 1 ]] || { echo "FAIL: $n_copies copies of the Livox SDK visible to the dynamic linker"; exit 1; }
syms=$(nm -D "$lib")
bad=$(grep -c '6spdlog\|3fmt' <<<"$syms" || true)      # the criterion of the version script (mangled, infix)
((bad == 0)) || { echo "FAIL: $lib exports or imports $bad spdlog/fmt symbols (they bind to libspdlog.so at run time)"; exit 1; }
grep -q 'LivoxLidarSdkInit' <<<"$syms" || { echo "FAIL: the Livox API is no longer exported from $lib"; exit 1; }
echo "PASS: $lib has no spdlog/fmt in its dynamic symbols"
