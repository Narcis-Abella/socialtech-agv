# socialtech-agv

Robot software for the IQS SocialTech Challenge team: an autonomous AGV (AgileX Tracer base) for hospital patient transport.
Runs on NVIDIA Jetson Orin (AGX Orin, Orin NX, Orin Nano) with JetPack 7.2.1, ROS 2 Jazzy, in Docker.

## Layout

| Path | What |
|---|---|
| `host/` | One-time Jetson host setup: everything that cannot live in a container (kernel modules, udev, networking, power) |
| `docker/` | The `Dockerfile` (layered stages `base` → `robot` → `sim` or `mapper`), the entrypoint and the ROS/DDS environment scripts |
| `deps/` | Third-party sources, pinned by commit, and the fixes we apply to them: `robot/` (`robot.repos` + `patches/<repo>/*.patch`, see its README), `nav2/` (the Nav2 source the patched controller is built from + its patch), `mapper/` (`mapper.repos`), `fetch_repos.py` (the shallow fetch all three use) |
| `docker/run_cpp_tests.sh` | Runs the C++ gtests that need only Eigen and GoogleTest (gn_core, level_corrector core and map_io); the image builds without tests |
| `src/` | Our own ROS 2 packages, by role; the image builds them on top of the third-party workspace, one layer per role, into `/opt/socialtech` (see Images) |
| `src/localization/` | `gn_loc` (scan-to-map Gauss-Newton, `map -> odom`), `gn_supervisor` (watches `gn_loc`, holds Nav2 when it cannot be trusted and relocalizes with a correlative search + GICP on the map's 3D prior `<map>/prior/map.xyz`; on by default, `SUPERVISOR=0` turns it off; its binaries are built by CMake), `level_corrector` (slow roll/pitch correction of FAST-LIO2's odom frame against the map's 3D points; on by default in `ops/nav_live.sh`, `LEVEL_CORRECTOR=0` turns it off; small_gicp is built in the `robot` stage) and `planar_odom` (FAST-LIO2 `/Odometry` to the levelled planar `base_footprint` TF); each has its README or header |
| `src/bringup/` | `socialtech_bringup`: the launch files, their defaults (`config/`) and `run_live`, the container half of `ops/nav_live.sh`; see its README |
| `src/navigation/` | Nav2 on top of the localizer: `socialtech_nav` (parameters, behavior trees) and `socialtech_costmap` (`DecayObstacleLayer`, height-band clearing); the patched controller is built from `deps/nav2` |
| `src/simulation/` | `socialtech_livox_sim`, `socialtech_simreal`: built only into the `sim` image |
| `src/mapping/` | `socialtech_mapping`: the beam counting of `tools/slam_bench/ghosts.py` in C++, built only into the `mapper` image |
| `ops/` | Running the robot from a board: `nav_live.sh` (the host half: checks, flags `NAV`, `BASE`, `SUPERVISOR`, `PROD`, `LEVEL_CORRECTOR`, `OVERLAY`; the stack itself starts in the container), `record_bag.sh` (the operator's recorder), `rviz_live.sh` + `laptop/` (rviz on the laptop against the board), `make_map.sh` (bag to map on a board), `build_image.sh` / `ship_image.sh` (images tagged by commit), base-layer checks run by hand (`tracer_test.sh` + `can_latency.py`: wheels in the air; `send_goal.py`: one recorded goal to Nav2; `nav_exec.sh`: a command in the nav container), and their tests |
| `tools/nav_sim/`, `tools/sim_realism/` | Full-chain simulation (simulated Mid-360 + real FAST-LIO2 + Nav2) on the AGX and its runners |
| `tools/slam_bench/` | Offline: bag to map with Voxel-SLAM (`make_map.sh`), bag checks, Voxel-SLAM cost on the Jetsons (see Tools) |

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

## Images

```bash
# from the repository root: the build context is the whole repo (the .dockerignore lets only docker/ and deps/ through)
docker build -f docker/Dockerfile --target robot -t socialtech:robot .
# DDS_IFACE: the robot's interface to talk DDS on (differs per board).
# /dev/bus/usb + cgroup rule: USB devices (Orbbec camera).
docker run --rm -it --network host \
  -e DDS_IFACE=wlP1p1s0 \
  -v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 189:* rmw' \
  socialtech:robot
```

`ops/build_image.sh <stage>` builds from the repository root and tags the image `socialtech:<stage>-<7 hex of the commit>[-dirty]`, moves the alias `socialtech:<stage>` to it, labels it `org.opencontainers.image.revision` and keeps the newest three tags per stage. A board that cannot build (the 8 GB Nano) gets the image from another one: `ops/ship_image.sh robot jetson@HOST` (`docker save | ssh | docker load`, then the alias moves there). Rollback on a board: `docker tag socialtech:robot-<old sha> socialtech:robot`. A checkout without `.git` (a copy made with `git archive`) still knows its commit: `REVISION` is filled in by `git archive` (`.gitattributes`).

On an 8 GB Orin Nano, if the build runs out of RAM: `--build-arg COLCON_WORKERS=1`.

| Stage | Contents |
|---|---|
| `base` | ROS 2 Jazzy, CycloneDDS, CUDA 13.2 runtime (must match the host L4T driver) |
| `robot` | Livox Mid-360 driver, FAST-LIO2, AgileX Tracer driver, Orbbec camera driver, rosbag2 + MCAP, `gn_loc` + `level_corrector` (with small_gicp) + `planar_odom`, Nav2 with our `socialtech_nav`/`socialtech_costmap` and the patched controller, map_server, pointcloud_to_laserscan, scipy, entrypoint and ROS environment |
| `sim` | `robot` + Gazebo and the simulation-only packages (`socialtech_livox_sim`, `socialtech_simreal`): the full-chain simulation of `tools/nav_sim` |
| `mapper` | `robot` + GTSAM 4.3.0, Voxel-SLAM and `rosbags`: offline mapping (`tools/slam_bench/make_map.sh`). Build it with `--target mapper`; without `--target` the last stage in the file is built |

Host networking is required (DDS discovery, Livox sockets). The GPU is requested via `NVIDIA_VISIBLE_DEVICES=all`, set in the image.
Without `DDS_IFACE`, CycloneDDS picks a network interface arbitrarily and other machines may not see the robot's nodes.
The images ask CycloneDDS for a 10 MiB socket receive buffer (`docker/cyclone_uri.sh`). When the host did not raise `net.core.rmem_max` (`host/setup_host.sh` does, via `/etc/sysctl.d/99-socialtech-dds.conf`), the entrypoint prints a warning naming it and every ROS node then fails to start ("failed to create domain"), instead of silently losing the LiDAR scans that overflow a UDP buffer (measured: 24 % of the scans on an Orin Nano). An explicit `CYCLONEDDS_URI` keeps its own configuration (interface included) and still gets the buffer request, unless it sets `SocketReceiveBufferSize` itself.
Shells opened with `docker exec -it <container> bash` get the same ROS environment as the entrypoint.

## Tools

The robot's SLAM is **Voxel-SLAM** (chosen 2026-10-01 after comparing it with GLIM, FAST-LIO2, iG-LIO and KISS-ICP; the comparison scripts were removed on 2026-11-08, see git history). It runs offline to build the maps: it is in the `mapper` image, not in `robot`, which runs FAST-LIO2 and gn_loc on the finished map. The tools below are offline helpers; none of them is part of the `robot` image.

- `tools/slam_bench/`: map building from a bag (`make_map.sh`), Voxel-SLAM cost measurements on the Jetsons; see its `README.md`.
- `tools/slam_bench/bag_check.py`: gaps in the LiDAR and IMU streams of a bag, from the recording times; run it right after recording (a scan lost while recording is gone for good and nothing warns about it).
- `tools/slam_bench/make_map.sh`: bag to map in one command inside the `mapper` image: Voxel-SLAM replay (2x) and then `make_map.py`, one process that levels the floor, removes the ghosts the beams go through, writes `map.ply` (raw SLAM cloud), `map_clean.ply` (ghost-free, whole height), `map.pgm` + `map.yaml` (2D, from the band 0.10-1.80 m of the clean cloud, unknown cells the beams crossed freed) and `report.json`. The modules are `cloud_io.py`, `floor_level.py`, `ghosts.py` (beam counting in C++, `src/mapping/socialtech_mapping`), `grid2d.py`; the relocalizer's 3D prior `prior/map.xyz` is built from `map_clean.ply` by `prepare_map.py` of `src/localization/gn_supervisor` (the last step of `make_map.sh`). See `tools/slam_bench/README.md`.

## Tests

`scripts/test.sh` runs everything that needs neither ROS 2 nor a board (pytest, the C++ gtests of the header-only cores, the shell tests of `docker/` and `ops/`); CI runs the same script on every pull request. Tests that need the robot image (rclpy, Nav2, FAST-LIO2, a running stack) are left out of the pytest collection by the `conftest.py` next to them and run inside the image; each package README says how.

## Where changes go

| Change | Where |
|---|---|
| Fix so upstream code builds/runs | `deps/robot/patches/<repo>/*.patch` |
| Our configuration (params, launch args) | `src/bringup/socialtech_bringup/config/` (overrides merged over `src/navigation/socialtech_nav/config/nav2_params.yaml`), never upstream configs |
| Our own nodes | an ament package under `src/<role>/` (see Layout); a Python reference of it, if it replaces one, goes in that package's `test/` |
| New logic in upstream code | a fork, pinned in `deps/robot/robot.repos` |

## License

GNU General Public License, version 2 only (`GPL-2.0-only`): see `LICENSE`. The patches in `deps/robot/patches/` modify GPL-2.0 code (FAST-LIO2, ikd-Tree) and are covered by it. The sources `robot.repos` fetches at build time keep their own licenses.

## Contributing

`main` is protected: work on a branch and open a pull request; merges need the maintainer's approval.
