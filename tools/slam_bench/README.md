# slam_bench

Map building from a bag with Voxel-SLAM, and the Voxel-SLAM cost measurements on the Jetsons. The 2026-09/10 comparison of SLAM methods (GLIM, FAST-LIO2, iG-LIO, KISS-ICP) was removed on 2026-11-08: its scripts are in the git history before that commit and its results and rationale in `notes/slam_seleccion.md` of the SocialTech repo.
Not deployment code: tooling run by hand on a laptop and a Jetson.

| File | Purpose |
|---|---|
| `m3dgr_convert.py` | M3DGR ROS 1 bag -> ROS 2 mcap with the Mid-360 as `PointCloud2` + IMU; with `pc2_to_custom.py` it makes the `benchmarks/m3dgr_*_custom` bags that `agx/jetson_cost_queue.sh` and `jetson_cost_long.sh` run on |
| `pc2_to_custom.py` | Mid-360 `PointCloud2` bag -> `livox_ros_driver2/CustomMsg` bag, for old bags recorded as `PointCloud2` (the robot records `CustomMsg` since `xfer_format 1`; FAST-LIO2 ignores per-point time in `PointCloud2`) |
| `bag_check.py` | gaps in the LiDAR and IMU streams of a bag from its recording times (run by `ops/record_bag.sh` and `ops/make_map.sh`) |
| `traj_compare.py` | run-against-run ATE of two Voxel-SLAM trajectories (the evidence that 1x and 2x replays give the same poses) |
| `cost_summary.py` | tegrastats + docker stats of a `jetson_cost.sh` run into one table |
| `voxelslam_run.sh` | run Voxel-SLAM on one bag inside an image (workspace at `/ws`, tools at `/tools`, output at `/out`) in `socialtech:mapper` |
| `make_map.sh` | Bag -> map in one command, inside the `mapper` image: Voxel-SLAM replay at 2x (`degrade_bound` 100000; the end is the file `vs/edge.txt`) and then `make_map.py`. `--reuse-slam` redoes everything after the SLAM of an earlier run in seconds. Leaves `map.pgm` + `map.yaml` (with `T_map_world`), `map.ply` (the raw Voxel-SLAM cloud), `map_clean.ply` (ghost-free, whole height), `report.json` and `vs/run/alidarState.txt` (the levelling reference of `socialtech_bringup/scripts/map_frame.py level`, read by `ops/nav_live.sh`) |
| `make_map.py` | The whole post-SLAM pipeline in one process: world cloud of the run (or `map.ply` + poses) -> level the floor once -> remove ghosts -> `map_clean.ply` -> 2D map from its band 0.10-`CEIL` -> `report.json` |
| `cloud_io.py` | PLY and pose I/O; the world cloud of a Voxel-SLAM run (one body-frame PCD per frame + `alidarState.txt`) in memory, float32 values like `map.ply` |
| `floor_level.py` | Floor of a SLAM cloud -> levelled (floor z = 0) -> walls on the axes -> cross-check with the trajectory. XYZ only, any SLAM; refuses when the floor cannot be trusted. Needs scipy. Holds `fit_plane` (RANSAC floor plane) and `rot_to_z`. Choices and evidence: `notes/ply_to_map_suelo.md` in the SocialTech repo |
| `ghosts.py` | Ghost removal: a 5 cm voxel the beams mostly pass through is not there. The beam test is `libsocialtech_mapping_beams.so` (C++, all cores) |
| `grid2d.py` | Rasterize a levelled cloud (halo, islands), free the unknown cells the beams crossed, write the PGM + `map_server` YAML |
| `map_report.py` | The `report.json` of a `make_map.sh` run (`pipeline: 2`, commit, replay rate, scans of the bag and the lost-scan warning, floor tilt, sensor height) |
| `golden/` | sha256 of the pipeline as it was before the refactor, and `check.py` that proves the new one reproduces it (`golden/README.md`) |
| `synth_room.py`, `synth_corridor.py`, `synth_bag.py`, `beams_oracle.py` | Test-only helpers: synthetic rooms and corridor, a rosbags writer, and the numpy beam counter the C++ engine is checked against |
| `voxelslam_degrade.sed` | `degrade_bound: 100000` for `voxelslam_run.sh` (`VS_SED`): the port's default 10 resets on bags with featureless stretches and never finishes |
| `agx/` | wrappers and queues as run on the AGX (`docker run` lines, DDS domain 78); see below for which still run |

## `agx/` today

The scripts run in `socialtech:mapper` (override with `IMAGE=`).

| Scripts | Method | Status |
|---|---|---|
| `bench_voxelslam.sh`, `jetson_*.sh` | Voxel-SLAM | Run in `socialtech:mapper`, the default (`IMAGE=` overrides). A port compiled in `~/bench_ws` overrides the image's. |

## Make a map

On a board, use the wrapper: `ops/make_map.sh BAG_NAME [MAP_NAME]` (bag in `~/rosbags/records`, map to `~/maps`). It runs `bag_check.py`, refuses a busy board or a low disk, runs the container as your user on a private DDS domain and records the commit in `report.json`.

The raw form it wraps (what the wrapper adds is not optional: without `MAKE_MAP_COMMIT` the report says `unknown`, without `--user` the maps belong to root, and without a private `ROS_DOMAIN_ID` the replay publishes on the LAN):

```bash
docker build -f docker/Dockerfile --target mapper -t socialtech:mapper .   # from the repository root
docker run --rm --init --network host --user "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_LOG_DIR=/tmp/roslog \
  -e ROS_DOMAIN_ID=78 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e DDS_IFACE=lo -e MAKE_MAP_COMMIT="$(git rev-parse --short HEAD)" \
  -v <bags>:/bags:ro -v <maps>:/out -v $PWD/tools/slam_bench:/tools:ro socialtech:mapper \
  /tools/make_map.sh [--reuse-slam] /bags/<bag_dir> /out/<name>
```

`make_map.sh` counts the beams with `libsocialtech_mapping_beams.so` (in the `mapper` image; else `BEAMS_LIB=<path>`) and checks it before the replay, so a host without it fails in a second. After the SLAM, `make_map.py` takes about 50 s on the AGX for a 366 s bag (assembling the cloud 19 s, the unknown-cell fill 29 s), measured 2026-11-01.

Outputs of `<name>/`: `map.pgm` + `map.yaml` (the 2D map; its `# T_map_world` comment is what `socialtech_bringup/scripts/map_frame.py` reads), `map.ply` (the raw SLAM cloud), `map_clean.ply` (the cloud without ghosts, whole height, floor z = 0, in the map frame: `prepare_map.py` of `gn_supervisor` builds the relocalizer prior `prior/map.xyz` from it, as the last step of `make_map.sh`) and `report.json`. A folder made before `pipeline: 2` has a different `map_clean.ply` (only the obstacle band over occupied cells): `prepare_map.py` refuses it; regenerate it with `--reuse-slam` (`REUSE_SLAM=1 ops/make_map.sh`).

Cleaning rule (constants of `ghosts.py` and `grid2d.py`, recorded in `report.json`): ghost test on the centroid of 5 cm voxels over the whole height (margin 3 cm, >= 10 observations, hit ratio < 0.10), density halo 0.10 m / 0.10, islands under 10 cells of 5 cm freed, no swept strip, Voxel-SLAM `blind` 0.1. Evidence: `notes/mapeo_optimizacion_2026-10-30.md` and `notes/mapeo_pipeline_*_2026-11-01.md` in the SocialTech repo. There are no options to reproduce an older map: `report.json` says which commit and rule made each one.

Running it again into the same `<name>` replaces the earlier run, `vs/run` (poses and scans) included: copy `vs/run/alidarState.txt` away first if you still need it.

`LIDAR` and `IMU` set the topics, `CEIL=30` makes an outdoor map, `PLAY_RATE` sets the replay speed (default 2). Voxel-SLAM follows the timestamps of the data, not the clock, so the speed should not change the result as long as no message is lost (the 366 s bag gave byte-identical poses at 1x and at 2x, N = 2 runs, one bag; if a run warns that scans were lost, relaunch with `PLAY_RATE=1`): keep the board otherwise idle while mapping (Elevator01 splits its sessions differently at 3x). On an idle Orin NX, eco01 at 0.5x and at 1x gave byte-identical poses and the same PGM; with several jobs on one AGX, pose counts of the same bag varied by up to 12 %, probably from lost messages. Check `report.json` (resets, `end_to_start_m`) before trusting a map. When the robot ends where it started (our bags do), check `end_to_start_m`: 1-2 m is normal.

Rules learned the hard way:
- One `ROS_DOMAIN_ID` per method (Voxel-SLAM 78); a second player with the same topic names on the same domain corrupts the input.
- Do not send SIGINT to background jobs started from a script (ignored); use a self-terminating recorder.
