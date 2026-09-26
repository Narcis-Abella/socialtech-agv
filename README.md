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

## Jetson setup (once per board)

Flash JetPack 7.2.1, then on the Jetson:

```bash
cd host
cp robot.conf.example robot.conf   # edit: hostname, Livox link, hotspot SSID, ...
sudo ./setup_host.sh               # interactive: hotspot password, Tailscale login, MAXN reboot
sudo ./setup_host.sh --check       # verify; all checks should PASS
```

Idempotent: re-run after changing `robot.conf` or after an L4T/kernel update (rebuilds the `gs_usb` CAN driver).

## Images

```bash
cd docker
docker build --target robot -t socialtech:robot .
docker run --rm -it --network host socialtech:robot
```

| Stage | Contents |
|---|---|
| `base` | ROS 2 Jazzy, CycloneDDS, CUDA 13.2 runtime (must match the host L4T driver) |
| `robot` | Livox Mid-360 driver, FAST-LIO2, AgileX Tracer driver, Orbbec camera driver, rosbag2 + MCAP |

Host networking is required (DDS discovery, Livox sockets). The GPU is requested via `NVIDIA_VISIBLE_DEVICES=all`, set in the image.

## Where changes go

| Change | Where |
|---|---|
| Fix so upstream code builds/runs | `docker/patches/<repo>/*.patch` |
| Our configuration (params, IPs, launch args) | our own bringup package (planned), never upstream configs |
| New logic in upstream code | a fork, pinned in `docker/robot.repos` |

## Contributing

`main` is protected: work on a branch and open a pull request; merges need the maintainer's approval.
