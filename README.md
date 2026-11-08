# socialtech-agv

Robot software for the IQS SocialTech Challenge team: an autonomous AGV (AgileX Tracer base) for hospital patient transport.
Runs on NVIDIA Jetson Orin (AGX Orin, Orin NX, Orin Nano) with JetPack 7.2.1, ROS 2 Jazzy, in Docker.

## Layout

```
host/    the Jetson itself (kernel modules, udev, networking, power): everything that cannot live in a container
deps/    third-party sources pinned by commit, and the fixes we apply to them
docker/  the Dockerfile and what the image copies from it
src/     our ROS 2 packages, by role: localization/ navigation/ bringup/ simulation/ mapping/
ops/     running the robot from a board (host side) and the checks done by hand
tools/   offline: the full-chain simulation, the map-building pipeline
scripts/ the one command that runs every test that needs neither ROS nor a board
```

| Path | What |
|---|---|
| `host/` | One-time Jetson host setup (`setup_host.sh`, udev rules, `nvme-heartbeat`) |
| `deps/robot/`, `deps/nav2/`, `deps/mapper/` | `robot.repos`, `nav2.repos`, `mapper.repos` (pinned third-party ROS sources) and the fixes to them: `robot/patches/<repo>/*.patch` (see its README), `nav2/patches/*.patch` (the controller built from the Nav2 source). `fetch_repos.py`: the shallow fetch all three use |
| `docker/` | `Dockerfile` (stages `base` → `robot` → `sim` or `mapper`), the entrypoint, `ros_env.sh`, `cyclone_uri.sh`, the Livox SDK symbol script and its test, `run_cpp_tests.sh` |
| `src/localization/` | `gn_loc` (scan-to-map Gauss-Newton, `map -> odom`), `gn_supervisor` (watches `gn_loc`, holds Nav2 when it cannot be trusted and relocalizes with a correlative search + GICP on the map's 3D prior `<map>/prior/map.xyz`), `level_corrector` (slow roll/pitch correction of FAST-LIO2's odom frame against the map's 3D points), `planar_odom` (FAST-LIO2 `/Odometry` to the levelled planar `base_footprint` TF) |
| `src/bringup/` | `socialtech_bringup`: the launch files, their defaults (`config/`) and `scripts/run_live`, the container half of `ops/nav_live.sh` |
| `src/navigation/` | `socialtech_nav` (Nav2 parameters, behavior trees), `socialtech_costmap` (`DecayObstacleLayer`, height-band clearing) |
| `src/simulation/` | `socialtech_livox_sim`, `socialtech_simreal`: built only into the `sim` image |
| `src/mapping/` | `socialtech_mapping`: the beam counting of `tools/slam_bench/ghosts.py` in C++, built only into the `mapper` image |
| `ops/` | `nav_live.sh` (starts the stack), `record_bag.sh`, `make_map.sh`, `rviz_live.sh` + `laptop/`, `build_image.sh` / `ship_image.sh` / `image_revision.sh`, `nav_exec.sh`, the base-layer checks (`tracer_test.sh` + `can_latency.py`: wheels in the air; `send_goal.py` (in `socialtech_bringup`, run with `ops/nav_exec.sh`): one recorded goal to Nav2) and their tests |
| `tools/nav_sim/`, `tools/sim_realism/` | Full-chain simulation (simulated Mid-360 + real FAST-LIO2 + Nav2) on the AGX and its runners |
| `tools/slam_bench/` | Offline: bag to map with Voxel-SLAM (`make_map.sh`), bag checks, Voxel-SLAM cost on the Jetsons |

Each package and tool folder has a README or a header that says what is in it.

## From a commit to the robot

The image is the robot: its code, launch files, defaults and binaries are built from this repository (`docker/Dockerfile`, build context = the repository root). A checkout on a board only holds the host-side scripts of `ops/`.

```bash
ops/build_image.sh robot                   # on a board that can build (the AGX): socialtech:robot-<7 hex of the commit>[-dirty] + the alias socialtech:robot
ops/ship_image.sh robot jetson@HOST ...    # to the boards that cannot (the 8 GB Nano): docker save | ssh | docker load, then the alias moves there
PEER=<laptop IP> ops/nav_live.sh MAP_NAME  # on the board; PEER only if rviz2 runs on a laptop
```

- **Provenance.** The image is labelled `org.opencontainers.image.revision` (the commit, `-dirty` for an edited tree); `ops/nav_live.sh` stops when the image is not the one this checkout builds (`ops/image_revision.sh`: only `src/`, `deps/`, `docker/` and the launcher count, so a docs commit leaves it current) and writes the revision to `provenance.txt` of the run. `STALE_IMAGE_OK=1` runs anyway. A checkout without `.git` (made by `git archive`) still knows its commit: `REVISION` is filled in by `git archive` (`.gitattributes`).
- **Rollback.** The newest three tags of each stage are kept: `docker tag socialtech:robot-<old sha> socialtech:robot`.
- **Developing without a rebuild.** `OVERLAY=1 ops/nav_live.sh ...` builds `socialtech_bringup` and `socialtech_nav` from the checkout's `src/` at start-up (launch files, scripts, config, the Nav2 parameters and behavior trees); C++ changes need an image. The simulation scripts do the same with the checkout mounted at `/src`.
- On an 8 GB Orin Nano, if a build runs out of RAM: `--build-arg COLCON_WORKERS=1`.

### `ops/nav_live.sh` flags
Unset = the default; `0`, `false`, `no`, `off` mean off (`src/bringup/socialtech_bringup/scripts/flags.sh`).

| Flag | Default | Does |
|---|---|---|
| `NAV=1` | off | Nav2, with the base OFF: goals plan, nothing moves |
| `BASE=1` (with `NAV`) | off | Starts the Tracer base: **a goal MOVES THE ROBOT** |
| `SUPERVISOR` | **on** | `gn_supervisor`; `SUPERVISOR=0` turns it off. The 3D prior is `<map>/prior/map.xyz` (made by `make_map.sh`; built here, in seconds, for a map that has none) |
| `LEVEL_CORRECTOR` | on | The roll/pitch corrector; `0` turns it off (the static level of the mapping run) |
| `PROD=1` | off | The robot as deployed: no bag recorder (DEV records ~14 GB/h with `NAV`), no rviz aids |
| `HEAVY_DDS=1` | off | Driver and FAST-LIO2 on loopback with 64 kB datagrams; not measured on the Nano; refused with the supervisor |
| `OVERLAY=1`, `STALE_IMAGE_OK=1` | off | See above |
| `PEER`, `DDS_IFACE`, `ROS_DOMAIN_ID` | none, route to `PEER` or `lo`, 53 | The laptop's address for rviz2 and the interface towards it; without `PEER` DDS stays on loopback |

## Images

| Stage | Contents |
|---|---|
| `base` | ROS 2 Jazzy, CycloneDDS, CUDA 13.2 runtime (must match the host L4T driver) |
| `robot` | Livox Mid-360 driver, FAST-LIO2, AgileX Tracer driver, Orbbec camera driver, rosbag2 + MCAP, `src/localization` (small_gicp twice: single-threaded for `level_corrector`, TBB for the supervisor's `register_v2`), Nav2 with the patched controller and `src/navigation`, `src/bringup`, map_server, pointcloud_to_laserscan, scipy, entrypoint and ROS environment |
| `sim` | `robot` + Gazebo and `src/simulation`: the full-chain simulation of `tools/nav_sim` |
| `mapper` | `robot` + GTSAM 4.3.0, Voxel-SLAM, `src/mapping` and `rosbags`: offline mapping (`tools/slam_bench/make_map.sh`). Without `--target` the last stage in the file is built |

```bash
docker build -f docker/Dockerfile --target robot -t socialtech:robot .     # from the repository root; the .dockerignore lets only docker/, deps/ and src/ through
```

All of ours installs into `/opt/socialtech` (one layer per role, after the third-party workspace, so an edit of ours never rebuilds FAST-LIO2 or the drivers); `ros_env.sh` sources it after `/opt/ros`, so the patched Nav2 controller shadows the apt one.

Host networking is required (DDS discovery, Livox sockets). The GPU is requested via `NVIDIA_VISIBLE_DEVICES=all`, set in the image. USB devices (the Orbbec camera): `-v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 189:* rmw'`.
Without `DDS_IFACE`, CycloneDDS picks a network interface arbitrarily and other machines may not see the robot's nodes.
The images ask CycloneDDS for a 10 MiB socket receive buffer (`docker/cyclone_uri.sh`). When the host did not raise `net.core.rmem_max` (`host/setup_host.sh` does, via `/etc/sysctl.d/99-socialtech-dds.conf`), the entrypoint prints a warning naming it and every ROS node then fails to start ("failed to create domain"), instead of silently losing the LiDAR scans that overflow a UDP buffer (measured: 24 % of the scans on an Orin Nano). An explicit `CYCLONEDDS_URI` keeps its own configuration (interface included) and still gets the buffer request, unless it sets `SocketReceiveBufferSize` itself.
Shells opened with `docker exec -it <container> bash` get the same ROS environment as the entrypoint.

## Jetson setup (once per board)

Flash JetPack 7.2.1, then on the Jetson:

```bash
cd host
cp robot.conf.example robot.conf   # edit: hostname, Livox link, hotspot SSID, ...
sudo ./setup_host.sh               # interactive: hotspot password, sudo password check, Tailscale login, MAXN reboot
sudo ./setup_host.sh --check       # verify; all checks should PASS
```

Idempotent: re-run after changing `robot.conf` (emptying a feature removes/disables it) or after an
L4T/kernel update (rebuilds the `gs_usb` CAN driver when the kernel ABI changed; `--check` flags a stale one).
It also disables the passwordless sudo that SDK Manager leaves for the default user, after you confirm its password.
It also builds and starts `nvme-heartbeat` (`host/nvme-heartbeat.c`, `NVME_HEARTBEAT=1`), a workaround for Orin NVMe interrupts that get lost and
stall a process for 30 s; `--check` fails if it is not running on every online CPU or its log shows a read blocked >= 1 s in the last hour
(`journalctl -u nvme-heartbeat`).

## Maps

The robot's SLAM is **Voxel-SLAM** (chosen 2026-10-01 after comparing it with GLIM, FAST-LIO2, iG-LIO and KISS-ICP; the comparison scripts were removed on 2026-11-08, see git history). It runs offline, in the `mapper` image, to build the maps; the `robot` image runs FAST-LIO2 and `gn_loc` on the finished map.

- `ops/record_bag.sh`, then `tools/slam_bench/bag_check.py` (run by it): gaps in the LiDAR and IMU streams, from the recording times (a scan lost while recording is gone for good and nothing warns about it).
- `ops/make_map.sh BAG_NAME [MAP_NAME]` runs `tools/slam_bench/make_map.sh` in the `mapper` image: Voxel-SLAM replay (2x), then `make_map.py` (levels the floor, removes the ghosts the beams go through) and the writing of `map.ply` (raw SLAM cloud), `map_clean.ply` (ghost-free, whole height), `map.pgm` + `map.yaml` (2D, from the band 0.10-1.80 m of the clean cloud, unknown cells the beams crossed freed), `report.json`, and `prior/map.xyz`, the relocalizer's 3D prior (`prepare_map.py` of `gn_supervisor`). `REUSE_SLAM=1` redoes everything after the SLAM in seconds. See `tools/slam_bench/README.md`.

## Tests

`scripts/test.sh` runs everything that needs neither ROS 2 nor a board (pytest, the C++ gtests of the header-only cores, the shell tests of `docker/`, `ops/` and `socialtech_bringup`); CI runs the same script on every pull request. Tests that need the robot image (rclpy, Nav2, FAST-LIO2, a running stack) are left out of the pytest collection by the `conftest.py` next to them and run inside the image, each as a script (`python3 test_x.py`: they start `rclpy` themselves, so not together in one pytest process); each package README says how. A simulation run (`tools/nav_sim/agx_full.sh`) is the end-to-end check of a change to the stack.

## Where changes go

| Change | Where |
|---|---|
| Fix so upstream code builds/runs | `deps/robot/patches/<repo>/*.patch` (Nav2: `deps/nav2/patches/`) |
| New logic in upstream code | a fork, pinned in `deps/robot/robot.repos` |
| Our configuration (params, launch args) | `src/bringup/socialtech_bringup/config/` (overrides merged over `src/navigation/socialtech_nav/config/nav2_params.yaml`), never upstream configs |
| Our own nodes | an ament package under `src/<role>/`; a Python reference of it, if it replaces one, goes in that package's `test/` |
| A new third-party dependency | `deps/robot/robot.repos` (ROS) or an `ARG` with a commit in the `Dockerfile` (plain CMake), never an unpinned `git clone` |

## License

GNU General Public License, version 2 only (`GPL-2.0-only`): see `LICENSE`. The patches in `deps/robot/patches/` modify GPL-2.0 code (FAST-LIO2, ikd-Tree) and are covered by it. The sources `robot.repos` fetches at build time keep their own licenses.

## Contributing

`main` is protected: work on a branch and open a pull request; merges need the maintainer's approval.
