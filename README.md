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
| `robot` | Livox Mid-360 driver, FAST-LIO2, AgileX Tracer driver, Orbbec camera driver, rosbag2 + MCAP, Nav2 AMCL + map_server, pointcloud_to_laserscan |

Host networking is required (DDS discovery, Livox sockets). The GPU is requested via `NVIDIA_VISIBLE_DEVICES=all`, set in the image.
Without `DDS_IFACE`, CycloneDDS picks a network interface arbitrarily and other machines may not see the robot's nodes.
Shells opened with `docker exec -it <container> bash` get the same ROS environment as the entrypoint.

## Where changes go

| Change | Where |
|---|---|
| Fix so upstream code builds/runs | `docker/patches/<repo>/*.patch` |
| Our configuration (params, IPs, launch args) | our own bringup package (planned), never upstream configs |
| Our own nodes | `docker/ros/<pkg>` (ament package; a Python reference + test next to the bench if it replaces one) |
| New logic in upstream code | a fork, pinned in `docker/robot.repos` |

## Contributing

`main` is protected: work on a branch and open a pull request; merges need the maintainer's approval.
