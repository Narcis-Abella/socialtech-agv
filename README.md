# socialtech-agv

Robot software for the IQS SocialTech Challenge team: an autonomous AGV (AgileX Tracer base) for hospital patient transport.
Runs on NVIDIA Jetson Orin (AGX Orin, Orin NX, Orin Nano) with JetPack 7.2.1, ROS 2 Jazzy, in Docker.

## Layout

| Path | What |
|---|---|
| `host/` | One-time Jetson host setup: everything that cannot live in a container (kernel modules, udev, networking, power) |
| `docker/` | Layered images built from one `Dockerfile` (`base` → `robot`) |
| `docker/robot.repos` | Third-party ROS sources, pinned by commit |
| `docker/patches/` | Build/run fixes to third-party sources (see its README) |
| `docker/ros/` | Our own ROS 2 packages, copied to `src/` and built with the third-party ones (`planar_odom`: FAST-LIO2 `/Odometry` to the levelled planar `base_footprint` TF). Under `docker/` because it is the build context |
| `docker/ros_mapper/` | Our packages built only into the `mapper` image (`socialtech_mapping`: the beam counting of `tools/slam_bench/ghosts.py` in C++) |
| `docker/ros_loc/` | The localizer and what levels it: `gn_loc` (scan-to-map Gauss-Newton, `map -> odom`) and `level_corrector` (slow roll/pitch correction of FAST-LIO2's odom frame against the map's 3D points; on by default in `ops/nav_live.sh`, `LEVEL_CORRECTOR=0` turns it off; small_gicp is built in the `robot` stage); each has its README |
| `docker/ros_nav/`, `docker/nav2/` | Nav2 on top of the localizer: `socialtech_nav` (parameters, launch), `socialtech_costmap` (`DecayObstacleLayer`, height-band clearing), the patched controller; the sim-only packages |
| `ops/` | Running the robot: `nav_live.sh` (localization + Nav2; flags `NAV`, `BASE`, `SUPERVISOR`, `PROD`, `LEVEL_CORRECTOR`), the launch files, the map and base helpers, and the tests of all that (`test_*.py`, `test_flags.sh`) |
| `tools/global_reloc_3d/` | The global relocalizer and the supervisor that calls it; **README says which files the robot runs and which are the previous pipeline** |
| `tools/nav_sim/`, `tools/sim_realism/` | Full-chain simulation (simulated Mid-360 + real FAST-LIO2 + Nav2) on the AGX and its runners |
| `tools/` | Offline research helpers: SLAM benchmark, 2D map from a PLY, GLIM dump to PLY and config evaluation (see Tools) |
| `tools/nav_bench/` | Localization bench (stage A): FAST-LIO2 + AMCL over a bag's own map, scored against Voxel-SLAM (see its README) |

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
cd docker
docker build --target robot -t socialtech:robot .
# DDS_IFACE: the robot's interface to talk DDS on (differs per board).
# /dev/bus/usb + cgroup rule: USB devices (Orbbec camera).
docker run --rm -it --network host \
  -e DDS_IFACE=wlP1p1s0 \
  -v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 189:* rmw' \
  socialtech:robot
```

On an 8 GB Orin Nano, if the build runs out of RAM: `--build-arg COLCON_WORKERS=1`.

| Stage | Contents |
|---|---|
| `base` | ROS 2 Jazzy, CycloneDDS, CUDA 13.2 runtime (must match the host L4T driver) |
| `robot` | Livox Mid-360 driver, FAST-LIO2, AgileX Tracer driver, Orbbec camera driver, rosbag2 + MCAP, gn_loc + map_server, pointcloud_to_laserscan, entrypoint and ROS environment |
| `mapper` | `robot` + GTSAM 4.3.0, Voxel-SLAM and `rosbags`: offline mapping (`tools/slam_bench/make_map.sh`). Build it with `--target mapper`; without `--target` the last stage in the file is built |

Host networking is required (DDS discovery, Livox sockets). The GPU is requested via `NVIDIA_VISIBLE_DEVICES=all`, set in the image.
Without `DDS_IFACE`, CycloneDDS picks a network interface arbitrarily and other machines may not see the robot's nodes.
The images ask CycloneDDS for a 10 MiB socket receive buffer (`docker/cyclone_uri.sh`). When the host did not raise `net.core.rmem_max` (`host/setup_host.sh` does, via `/etc/sysctl.d/99-socialtech-dds.conf`), the entrypoint prints a warning naming it and every ROS node then fails to start ("failed to create domain"), instead of silently losing the LiDAR scans that overflow a UDP buffer (measured: 24 % of the scans on an Orin Nano). An explicit `CYCLONEDDS_URI` keeps its own configuration (interface included) and still gets the buffer request, unless it sets `SocketReceiveBufferSize` itself.
Shells opened with `docker exec -it <container> bash` get the same ROS environment as the entrypoint.

## Tools

The robot's SLAM is **Voxel-SLAM** (chosen 2026-10-01 after comparing it with GLIM, FAST-LIO2, iG-LIO and KISS-ICP; the comparison scripts were removed on 2026-11-08, see git history). It runs offline to build the maps: it is in the `mapper` image, not in `robot`, which runs FAST-LIO2 and gn_loc on the finished map. The tools below are offline helpers; none of them is part of the `robot` image.

- `tools/slam_bench/`: map building from a bag (`make_map.sh`), Voxel-SLAM cost measurements on the Jetsons, and the floor diagnostics `make_map` uses (`floor_tilt.py`, `static_offset.py`); see its `README.md`.
- `tools/slam_bench/bag_check.py`: gaps in the LiDAR and IMU streams of a bag, from the recording times; run it right after recording (a scan lost while recording is gone for good and nothing warns about it).
- `tools/slam_bench/make_map.sh`: bag to map in one command inside the `mapper` image: Voxel-SLAM replay (2x) and then `make_map.py`, one process that levels the floor, removes the ghosts the beams go through, writes `map.ply` (raw SLAM cloud), `map_clean.ply` (ghost-free, whole height), `map.pgm` + `map.yaml` (2D, from the band 0.10-1.80 m of the clean cloud, unknown cells the beams crossed freed) and `report.json`. The modules are `cloud_io.py`, `floor_level.py`, `ghosts.py` (beam counting in C++, `docker/ros_mapper/socialtech_mapping`), `grid2d.py`; the relocalizer prior is built from `map_clean.ply` by `tools/global_reloc_3d/prepare_map.py`. See `tools/slam_bench/README.md`.

## Where changes go

| Change | Where |
|---|---|
| Fix so upstream code builds/runs | `docker/patches/<repo>/*.patch` |
| Our configuration (params, IPs, launch args) | our own bringup package (planned), never upstream configs |
| Our own nodes | `docker/ros/<pkg>` (ament package; a Python reference + test next to the bench if it replaces one) |
| New logic in upstream code | a fork, pinned in `docker/robot.repos` |

## License

GNU General Public License, version 2 only (`GPL-2.0-only`): see `LICENSE`. The patches in `docker/patches/` modify GPL-2.0 code (FAST-LIO2, ikd-Tree) and are covered by it. The sources `robot.repos` fetches at build time keep their own licenses.

## Contributing

`main` is protected: work on a branch and open a pull request; merges need the maintainer's approval.
