#!/bin/bash
# Build only the glim_ext modules under test (flat_earther, gravity_estimator) against the GLIM installed in the robot image.
# Runs on the Jetson host; the image is left untouched (it ships CUDA runtime only, so the host toolkit with nvcc is mounted read-only over the same path), the .so files land in <out_dir>/lib (mount it and list the libs in extension_modules).
# Also writes <out_dir>/config: the installed GLIM defaults plus glim_ext's config_flat_earther.json, registered under `global` (GLIM 1.2.2's config.json lacks it).
# Use with glim_eval.sh: GLIM_DEFAULTS=/ext_config, -v <out_dir>/lib:/ext:ro -v <out_dir>/config:/ext_config:ro, LD_LIBRARY_PATH starting with /ext.
# usage: build_glim_ext.sh <out_dir> [git_ref]      (out_dir is created by Docker as root: use a fresh name per attempt)
set -eo pipefail
out=${1:?usage: build_glim_ext.sh <out_dir> [git_ref]}; ref=${2:-master}
mkdir -p "$out"
docker run --rm --network host -v "$out":/out -v /usr/local/cuda-13.2:/usr/local/cuda-13.2:ro socialtech:robot bash -c "
  set -eo pipefail
  source /opt/ros/jazzy/setup.bash
  export PATH=/usr/local/cuda/bin:\$PATH
  ln -sf libmetis.so.5 /usr/lib/aarch64-linux-gnu/libmetis.so   # gtsam's CMake config links the dev symlink, the image only has the runtime lib
  git clone -q https://github.com/koide3/glim_ext /tmp/src && git -C /tmp/src checkout -q $ref
  cmake -S /tmp/src -B /tmp/build -DCMAKE_BUILD_TYPE=Release -DENABLE_VELSUPP=OFF -DENABLE_IMUVAL=OFF -DENABLE_IMUPRED=OFF \
    -DENABLE_DESKEWING=OFF -DENABLE_GNSS=OFF -DENABLE_GRAVITY=ON -DENABLE_FLATEARTHER=ON
  cmake --build /tmp/build -j6
  mkdir -p /out/lib && find /tmp/build -name '*.so' -exec cp -v {} /out/lib/ \;
  git -C /tmp/src rev-parse HEAD > /out/glim_ext_commit.txt
  cp -r /opt/ros/jazzy/share/glim/config /out/config && cp /tmp/src/config/config_flat_earther.json /out/config/
  sed -i 's|\"config_global_mapping\": \"config_global_mapping_gpu.json\"|&,\n    \"config_flat_earther\": \"config_flat_earther.json\"|' /out/config/config.json
  grep -c config_flat_earther /out/config/config.json
"
