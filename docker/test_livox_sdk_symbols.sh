#!/bin/bash
# The Livox SDK must not export or import any spdlog/fmt symbol (docker/livox_sdk_hide_spdlog.map): its bundled spdlog 1.3 and the system libspdlog 1.12 mixed their vtables at exit
# and livox_ros_driver2_node segfaulted in about 1 of 10 shutdowns. Run by the image build right after the SDK install; also by hand:
#   docker run --rm --entrypoint bash -v "$PWD/docker:/t" socialtech:robot /t/test_livox_sdk_symbols.sh [LIB]
set -euo pipefail
lib=${1:-$(ldconfig -p | awk '/liblivox_lidar_sdk_shared\.so /{print $NF; exit}')}
[[ -f $lib ]] || { echo "FAIL: liblivox_lidar_sdk_shared.so not found (ldconfig -p)"; exit 1; }
n_copies=$(ldconfig -p | grep -c 'liblivox_lidar_sdk_shared\.so ' || true)
[[ $n_copies == 1 ]] || { echo "FAIL: $n_copies copies of the Livox SDK visible to the dynamic linker"; exit 1; }
syms=$(nm -D "$lib" | c++filt)
bad=$(grep -c 'spdlog::\|fmt::' <<<"$syms" || true)
((bad == 0)) || { echo "FAIL: $lib exports or imports $bad spdlog/fmt symbols (they bind to libspdlog.so at run time)"; exit 1; }
grep -q 'LivoxLidarSdkInit' <<<"$syms" || { echo "FAIL: the Livox API is no longer exported from $lib"; exit 1; }
echo "PASS: $lib has no spdlog/fmt in its dynamic symbols"
