# syntax=docker/dockerfile:1
# SocialTech robot images for Jetson Orin (JetPack 7.2.1 / L4T r39.2, arm64).
# One file, layered stages; build one with: docker build --target <stage> .
#   base   ROS 2 Jazzy + CycloneDDS + CUDA runtime matching the host driver
#   robot  sensor/base drivers + FAST-LIO2 + rosbag2 (what runs on every robot)
# Run with network_mode: host (DDS discovery, Livox raw sockets) and the GPU requested
# via NVIDIA_VISIBLE_DEVICES (set below) or --device nvidia.com/gpu=all.

# Pinned by digest (the tag moves with every ROS sync); the one built and tested on the Orin NX.
FROM ros:jazzy-ros-base@sha256:c3706ef0a0aa45413c07803cf433602f543b22e45b4855f6fca955c2d8ecc4e8 AS base

# The host (JetPack) provides only the driver via CDI; the CUDA runtime lives in the image and
# must not be newer than the host driver. Bump together with the L4T version on the Jetsons.
ARG L4T_RELEASE=r39.2
ARG CUDA_VERSION=13.2.86-1

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Keep downloaded .debs in the BuildKit cache mount instead of deleting them.
RUN rm -f /etc/apt/apt.conf.d/docker-clean

ADD --chmod=644 https://repo.download.nvidia.com/jetson/jetson-ota-public.asc /etc/apt/trusted.gpg.d/jetson-ota-public.asc
RUN echo "deb https://repo.download.nvidia.com/jetson/common ${L4T_RELEASE} main" \
      > /etc/apt/sources.list.d/nvidia-l4t.list

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      cuda-cudart-13-2="${CUDA_VERSION}" \
      ros-jazzy-rmw-cyclonedds-cpp

ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility \
    PATH=/usr/local/cuda-13.2/bin:${PATH} \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ROS environment shared by the entrypoint and interactive shells (`docker exec -it ... bash`).
COPY ros_env.sh /ros_env.sh
RUN echo 'source /ros_env.sh' >> /etc/bash.bashrc
COPY --chmod=755 entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# ──────────────────────────────────────────────────────────────────────────────
FROM base AS robot

# Livox-SDK2: plain CMake library (not a ROS package), needed by livox_ros_driver2.
ARG LIVOX_SDK2_COMMIT=c0796f04c143143899c87a773d9f6b7136453c0b
RUN --mount=type=tmpfs,target=/tmp/src \
    git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/src/livox-sdk2 \
 && git -C /tmp/src/livox-sdk2 checkout "${LIVOX_SDK2_COMMIT}" \
 && cmake -S /tmp/src/livox-sdk2 -B /tmp/src/livox-sdk2/build -DCMAKE_BUILD_TYPE=Release \
 && cmake --build /tmp/src/livox-sdk2/build -j"$(nproc)" \
 && cmake --install /tmp/src/livox-sdk2/build \
 && ldconfig

WORKDIR /ws
COPY robot.repos fetch_repos.py ./
COPY patches/ patches/
RUN python3 fetch_repos.py robot.repos src \
 # livox_ros_driver2 ships ROS1/ROS2 variants; its build.sh swaps these in before building.
 && cp src/livox_ros_driver2/package_ROS2.xml src/livox_ros_driver2/package.xml \
 && cp -r src/livox_ros_driver2/launch_ROS2 src/livox_ros_driver2/launch \
 # Fixes to third-party code: patches/<repo name in robot.repos>/*.patch, applied in order.
 && shopt -s nullglob \
 && for p in patches/*/*.patch; do \
      repo=$(basename "$(dirname "$p")"); echo "applying $p"; \
      git -C "src/$repo" apply "/ws/$p" || exit 1; \
    done

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update \
 && rosdep update --rosdistro jazzy \
 # ugv_sdk declares catkin (ROS 1) but is plain CMake that only uses catkin if found.
 && DEBIAN_FRONTEND=noninteractive rosdep install -y --from-paths src --ignore-src --rosdistro jazzy \
      --skip-keys catkin \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-jazzy-rosbag2-storage-mcap

# build/ and log/ live in cache mounts so they never land in the image. (A refetch gives sources
# new mtimes, so it still rebuilds everything.) Few parallel packages: each already compiles with
# -j$(nproc), and PCL-heavy packages side by side can exhaust RAM on an 8 GB Orin Nano.
ARG COLCON_WORKERS=2
RUN --mount=type=cache,target=/ws/build \
    --mount=type=cache,target=/ws/log \
    . /opt/ros/jazzy/setup.sh \
 && colcon build --merge-install --install-base /opt/socialtech \
      --parallel-workers "${COLCON_WORKERS}" \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy

# Navigation bench (tools/nav_bench): localization and scan conversion. After the colcon build so a change here does not rebuild the workspace.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-jazzy-nav2-amcl ros-jazzy-nav2-map-server ros-jazzy-nav2-lifecycle-manager ros-jazzy-pointcloud-to-laserscan
