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
| `tools/` | Offline helpers: GLIM map dump to PLY without the GUI, GLIM config evaluation (see Tools) |

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
| `robot` | Livox Mid-360 driver, FAST-LIO2, AgileX Tracer driver, Orbbec camera driver, rosbag2 + MCAP |

Host networking is required (DDS discovery, Livox sockets). The GPU is requested via `NVIDIA_VISIBLE_DEVICES=all`, set in the image.
Without `DDS_IFACE`, CycloneDDS picks a network interface arbitrarily and other machines may not see the robot's nodes.
Shells opened with `docker exec -it <container> bash` get the same ROS environment as the entrypoint.

## Tools

```bash
# GLIM map dump (glim_rosbag -p dump_path:=...) -> PLY, same points as offline_viewer's Export Points.
python3 tools/glim_dump_to_ply.py <dump_dir> map.ply
python3 tools/test_glim_dump_to_ply.py   # self-check
```

Needs numpy (apt: `python3-numpy`). Differences from the GUI export are listed in the script's docstring.

### GLIM config evaluation

Compare GLIM configs on recorded bags. GLIM is not deterministic, so each config runs several times.

```bash
# In the robot image (needs GLIM): N runs of one config = installed defaults + overlays, in order.
docker run --rm --network host -v ~/rosbags:/bags -v ~/glim_eval:/eval -v "$PWD/tools:/tools" socialtech:robot \
  /tools/glim_eval.sh /bags/mapping_eco_-1_01 /eval/eco01_legacy 3 /tools/glim_eval/base.json /tools/glim_eval/legacy.json
# Anywhere with numpy + scipy: metrics per run + divergence between runs.
python3 tools/glim_metrics.py ~/glim_eval/eco01_legacy/run*/
python3 tools/test_glim_config.py && python3 tools/test_glim_metrics.py   # self-checks
```

`glim_config.py` rejects overlay parameters that the installed GLIM does not have (GLIM itself ignores them silently).
Metrics need no ground truth; definitions in `glim_metrics.py`. The main signals: `revisit_p90_m`
(doubled walls where a place is passed again), `mme` (Mean Map Entropy: blur anywhere, also within
one pass), `z_range_m` / `roll_range_deg` / `pitch_range_deg` (drift on a flat floor), run divergence (stability).

| Overlay (`tools/glim_eval/`) | On top of | What |
|---|---|---|
| `base.json` | defaults | Livox topics, Mid-360 `T_lidar_imu` (IMU pose in the LiDAR frame, Mid-360 manual), headless modules |
| `legacy.json` | `base.json` | Previous (Humble, GLIM 1.0.x) config ported to 1.2.x, including its `T_lidar_imu`, to reproduce its results |
| `loop_overlap.json` | `base.json` | `min_implicit_loop_overlap` 0.05: more loop constraints |
| `far_8m.json` | `base.json` | `distance_far_thresh` 8 m, as in the legacy config: more frequent keyframes/submaps? |
| `density.json` | `base.json` | Denser submaps: more keyframes, 5 cm voxels, no point cap |

Not portable from the legacy config: its crop box kept only z in [-0.43, 1.23] m through a `crop_bbox_invert`
parameter that upstream GLIM does not have (1.2.x only removes points inside the box). Left out: frame IDs,
QoS, image topic, `ring_field` (Livox has no ring field; ignored when absent), viewers.

## Where changes go

| Change | Where |
|---|---|
| Fix so upstream code builds/runs | `docker/patches/<repo>/*.patch` |
| Our configuration (params, IPs, launch args) | our own bringup package (planned), never upstream configs |
| New logic in upstream code | a fork, pinned in `docker/robot.repos` |

## Contributing

`main` is protected: work on a branch and open a pull request; merges need the maintainer's approval.
