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
    done \
 # Submodule fixes (git apply cannot reach into a submodule from the parent): patches/<repo>/<submodule path>/*.patch.
 && for p in patches/*/*/*/*.patch; do \
      rel=${p#patches/}; echo "applying $p"; \
      git -C "src/$(dirname "$rel")" apply "/ws/$p" || exit 1; \
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

# Our own packages (docker/ros/<pkg>): built with the third-party ones.
COPY ros/ src/

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
      ros-jazzy-nav2-amcl ros-jazzy-nav2-map-server ros-jazzy-nav2-lifecycle-manager ros-jazzy-pointcloud-to-laserscan \
      ros-jazzy-bondcpp

# docker/ros_loc: gn_loc (Gauss-Newton scan-to-map localizer; ops/nav_live.sh selects it with LOCALIZER=gn, AMCL with LOCALIZER=amcl) and scan_median (the /scan
# binner, quantile 0 = pointcloud_to_laserscan). Their own colcon build: gn_loc needs the Nav2 packages installed just above, which the first build does not
# have. Tests and the gn_offline study tool are not built in the image (BUILD_TESTING=OFF, GN_LOC_OFFLINE=OFF); see docker/ros_loc/gn_loc/README.md.
COPY ros_loc/ /ws/src_loc/
RUN --mount=type=cache,target=/tmp/build_loc \
    --mount=type=cache,target=/tmp/log_loc \
    . /opt/ros/jazzy/setup.sh \
 && cd /ws && colcon --log-base /tmp/log_loc build --base-paths src_loc --merge-install --install-base /opt/socialtech_loc --build-base /tmp/build_loc \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DGN_LOC_OFFLINE=OFF

# Nav2 on the real robot (ops/nav_live.sh NAV=1) and in the simulator, whose stage adds Gazebo on top. docker/ros_nav: socialtech_nav (our params, launch and map) and
# socialtech_costmap (the decay obstacle layer). The simulation-only packages of ros_nav are built in `sim`. The regulated pure pursuit controller is built from the
# pinned Nav2 source with nav2/*.patch (rotate_to_heading_dist_tol, which ops/nav2_robot_overrides.yaml sets) and installed in /opt/socialtech_nav, which ros_env.sh
# sources after /opt/ros: it shadows the apt controller that nav2_bringup pulls in. Not in patches/: that directory is applied to the robot.repos checkouts.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-jazzy-nav2-bringup ros-jazzy-nav2-regulated-pure-pursuit-controller ros-jazzy-nav2-smac-planner
COPY nav2/ /ws/nav2/
RUN python3 fetch_repos.py nav2/nav2.repos src_nav2 \
 && for p in nav2/*.patch; do echo "applying $p"; git -C src_nav2/navigation2 apply "/ws/$p" || exit 1; done
COPY ros_nav/ /ws/src_nav/
RUN . /opt/ros/jazzy/setup.sh \
 && colcon build --base-paths src_nav src_nav2/navigation2/nav2_regulated_pure_pursuit_controller \
      --packages-select socialtech_nav socialtech_costmap nav2_regulated_pure_pursuit_controller \
      --merge-install --install-base /opt/socialtech_nav --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# ROS environment shared by the entrypoint and interactive shells (`docker exec -it ... bash`).
# Last on purpose: editing these scripts must not invalidate the SDK and colcon layers above.
COPY cyclone_uri.sh /cyclone_uri.sh
COPY ros_env.sh /ros_env.sh
RUN echo 'source /ros_env.sh' >> /etc/bash.bashrc
COPY --chmod=755 entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# ──────────────────────────────────────────────────────────────────────────────
# Simulator: Gazebo Harmonic on top of `robot`, which already carries Nav2 and socialtech_nav. Adds the simulation-only packages of ros_nav to the same prefix.
FROM robot AS sim
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz ros-jazzy-xacro libembree-dev

# /opt/socialtech too: socialtech_livox_sim needs livox_ros_driver2, which `robot` installed there.
WORKDIR /ws
RUN . /opt/ros/jazzy/setup.sh && . /opt/socialtech/setup.sh && . /opt/socialtech_nav/setup.sh \
 && colcon build --base-paths src_nav --packages-select socialtech_livox_sim socialtech_simreal --merge-install --install-base /opt/socialtech_nav

# Offline mapping (`docker build --target mapper`): Voxel-SLAM + GTSAM + the tools/slam_bench pipeline (make_map.sh). A separate stage: the robot image
# runs the navigation stack and carries neither GTSAM (with Boost) nor Voxel-SLAM. Without --target, the last stage is the one that gets built.
FROM robot AS mapper
# GTSAM 4.3.0 from koide3's PPA, the build Voxel-SLAM was validated against. The PPA republishes debs under the same version, so apt reads a fixed commit of its
# `ppa` branch (signed InRelease -> every deb's SHA256); to bump, move the commit. The repo and its key are removed after the install so later apt runs cannot move it.
ARG KOIDE3_PPA_COMMIT=a21da6b120456c1b9a1ce15480b622c8ccbaa3ff
ADD --chmod=644 --checksum=sha256:08c6f6bff0bf9b991398c693d44a16871932bc7de969848413e3066dc1acc77e \
    https://raw.githubusercontent.com/koide3/ppa/${KOIDE3_PPA_COMMIT}/ubuntu2404/KEY.gpg /etc/apt/keyrings/koide3.asc
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    echo "deb [signed-by=/etc/apt/keyrings/koide3.asc]" \
         "https://raw.githubusercontent.com/koide3/ppa/${KOIDE3_PPA_COMMIT}/ubuntu2404 ./" \
      > /etc/apt/sources.list.d/koide3.list \
 && apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      libgtsam-notbb-dev=4.3.0 libmetis-dev python3-numpy python3-scipy python3-pip \
 && rm /etc/apt/sources.list.d/koide3.list /etc/apt/keyrings/koide3.asc \
 && ldconfig
# map_visibility.py reads the bag with rosbags (mcap and sqlite3); the version it was written against.
RUN pip3 install --no-cache-dir --break-system-packages rosbags==0.11.5

WORKDIR /ws
COPY mapper.repos ./
ARG COLCON_WORKERS=2
RUN python3 fetch_repos.py mapper.repos src_mapper \
 && . /opt/ros/jazzy/setup.sh && . /opt/socialtech/setup.sh \
 && colcon --log-base /tmp/log_mapper build --base-paths src_mapper --packages-select voxel_slam \
      --merge-install --install-base /opt/socialtech --build-base /tmp/build_mapper \
      --parallel-workers "${COLCON_WORKERS}" --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && ldd -r /opt/socialtech/lib/voxel_slam/voxelslam > /tmp/ldd.txt \
 && if grep -E 'not found|undefined symbol' /tmp/ldd.txt; then exit 1; fi \
 && rm /tmp/ldd.txt

# socialtech_mapping: the beam counting of map_visibility.py in C++ (libsocialtech_mapping_beams.so, loaded with --engine cpp). After Voxel-SLAM so a change here does not rebuild it.
COPY ros_mapper/ src_mapper_local/
RUN . /opt/ros/jazzy/setup.sh \
 && colcon --log-base /tmp/log_mapper_local build --base-paths src_mapper_local --merge-install --install-base /opt/socialtech \
      --build-base /tmp/build_mapper_local --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
 && rm -r /tmp/build_mapper_local /tmp/log_mapper_local \
 && ldd -r /opt/socialtech/lib/libsocialtech_mapping_beams.so > /tmp/ldd_beams.txt \
 && if grep -E 'not found|undefined symbol' /tmp/ldd_beams.txt; then exit 1; fi \
 && rm /tmp/ldd_beams.txt
