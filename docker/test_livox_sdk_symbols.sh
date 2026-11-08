#!/bin/bash
# The Livox SDK must neither export nor import a spdlog/fmt symbol (docker/livox_sdk_hide_spdlog.map says why). Run by the image build right after the SDK install; by hand:
#   docker run --rm --entrypoint bash -v "$PWD/docker:/t" socialtech:robot /t/test_livox_sdk_symbols.sh [LIB]
set -euo pipefail
cache=$(ldconfig -p)           # a variable, not a pipe: with pipefail an early-exiting awk/grep -q would fail the pipeline with SIGPIPE
lib=${1:-$(sed -n 's/.*liblivox_lidar_sdk_shared\.so .*=> //p' <<<"$cache" | head -n1 || true)}
[[ -n $lib && -f $lib ]] || { echo "FAIL: liblivox_lidar_sdk_shared.so not found (ldconfig -p)"; exit 1; }
if [[ -z ${1:-} ]]; then      # the library the linker would load must be the only one; with an explicit path the caller tests a library that is not installed
  n_copies=$(grep -c 'liblivox_lidar_sdk_shared\.so ' <<<"$cache" || true)
  [[ $n_copies == 1 ]] || { echo "FAIL: $n_copies copies of the Livox SDK visible to the dynamic linker"; exit 1; }
fi
syms=$(nm -D "$lib")
# both criteria: the version script's (mangled, infix) and the readable one, so a symbol the globs miss still fails
mangled=$(grep -c '6spdlog\|3fmt' <<<"$syms" || true)
readable=$(c++filt <<<"$syms" | grep -c 'spdlog::\|fmt::' || true)
((mangled == 0 && readable == 0)) || { echo "FAIL: $lib exports or imports spdlog/fmt symbols ($mangled by mangled name, $readable by readable name): they bind to libspdlog.so at run time"; exit 1; }
grep -q 'LivoxLidarSdkInit' <<<"$syms" || { echo "FAIL: the Livox API is no longer exported from $lib"; exit 1; }
echo "PASS: $lib has no spdlog/fmt in its dynamic symbols"
