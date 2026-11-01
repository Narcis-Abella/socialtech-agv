# slam_bench

Scripts used to compare SLAM methods (GLIM, FAST-LIO2, iG-LIO, Voxel-SLAM, KISS-ICP) on public datasets with ground truth
(TIERS multi-modal LiDAR, M3DGR) and on our own bags. Results and rationale: `notes/slam_seleccion.md` in the SocialTech repo.
Not deployment code: research tooling, run by hand on a laptop and a Jetson AGX Orin.

| File | Purpose |
|---|---|
| `m3dgr_convert.py` | M3DGR ROS 1 bag (Livox `CustomMsg`) -> ROS 2 mcap with Mid-360 `PointCloud2` (`x y z intensity tag line timestamp`) + IMU |
| `pc2_to_custom.py` | Mid-360 `PointCloud2` bag -> `livox_ros_driver2/CustomMsg` bag (FAST-LIO2 ignores per-point time in `PointCloud2`) |
| `gt_from_bag.py` | Mocap `PoseStamped` topic -> TUM file |
| `restamp.py` | KISS-ICP writes bag time; replace it with each scan's `header.stamp` |
| `aruco_metric.py` | M3DGR ArUco end-pose error (Corridor01, Elevator01), same formula as the dataset script; robust to blank lines |
| `odom_to_tum.py` | rclpy recorder: `nav_msgs/Odometry` topic -> TUM, exits when the topic goes silent |
| `fastlio_run.sh`, `iglio_run.sh`, `voxelslam_run.sh` | run one method on one bag inside an image (workspace at `/ws`, tools at `/tools`, output at `/out`): `socialtech:mapper` for Voxel-SLAM, `socialtech:robot` for FAST-LIO2; iG-LIO is in no image |
| `eval_all.py` | One markdown table from `<root>/<method>/<sequence>/*.tum`: APE (SE(3) alignment, offset 0 and best offset) for mocap sequences, ArUco closure for Corridor01 / Elevator01 |
| `traj_metrics.py` | Ground-truth-free table for our own bags (flat floor, return to start): path, end-start distance, z / roll / pitch range, divergence between runs |
| `collect_results.sh` | Copy all trajectories from the AGX into the layout `eval_all.py` reads (GLIM runs, competitors at 0.5x; skips contaminated runs) |
| `floor_tilt.py` | Ground-truth-free flatness of GLIM dumps: floor plane per submap, global floor tilt from x-y, floor residual (`floor_tilt.py [-v] <run_dir>...`) |
| `static_offset.py` | IMU-vs-floor/ceiling offset of the Livox from raw bags (still windows); prints a `T_lidar_imu` overlay that cancels the GLIM map tilt |
| `make_map.sh` | Bag -> map in one command, inside the `mapper` image: Voxel-SLAM replay at 2x (`degrade_bound` 100000; the end is the file `vs/edge.txt`) and then `make_map.py`. `--reuse-slam` redoes everything after the SLAM of an earlier run in seconds. Leaves `map.pgm` + `map.yaml` (with `T_map_world`), `map.ply` (the raw Voxel-SLAM cloud), `map_clean.ply` (ghost-free, whole height), `report.json` and `vs/run/alidarState.txt` (the reference for `pose_eval`) |
| `make_map.py` | The whole post-SLAM pipeline in one process: world cloud of the run (or `map.ply` + poses) -> level the floor once -> remove ghosts -> `map_clean.ply` -> 2D map from its band 0.10-`CEIL` -> `report.json` |
| `cloud_io.py` | PLY and pose I/O; the world cloud of a Voxel-SLAM run (one body-frame PCD per frame + `alidarState.txt`) in memory, float32 values like `map.ply` |
| `floor_level.py` | Floor of a SLAM cloud -> levelled (floor z = 0) -> walls on the axes -> cross-check with the trajectory. XYZ only, any SLAM; refuses when the floor cannot be trusted. Needs scipy (host/laptop, not in the robot image). Choices and evidence: `notes/ply_to_map_suelo.md` in the SocialTech repo |
| `ghosts.py` | Ghost removal: a 5 cm voxel the beams mostly pass through is not there. The beam test is `libsocialtech_mapping_beams.so` (C++, all cores) |
| `grid2d.py` | Rasterize a levelled cloud (halo, islands), free the unknown cells the beams crossed, write the PGM + `map_server` YAML |
| `map_report.py` | The `report.json` of a `make_map.sh` run (`pipeline: 2`, commit, replay rate, scans of the bag and the lost-scan warning, floor tilt, sensor height) |
| `golden/` | sha256 of the pipeline as it was before the refactor, and `check.py` that proves the new one reproduces it (`golden/README.md`) |
| `synth_room.py`, `synth_corridor.py`, `synth_bag.py`, `beams_oracle.py` | Test-only helpers: synthetic rooms and corridor, a rosbags writer, and the numpy beam counter the C++ engine is checked against |
| `voxelslam_degrade.sed` | `degrade_bound: 100000` for `voxelslam_run.sh` (`VS_SED`): the port's default 10 resets on bags with featureless stretches and never finishes |
| `agx/` | wrappers and queues as run on the AGX (`docker run` lines, DDS domain per method); see below for which still run |

## `agx/` today

The image tags in these scripts date from the benchmark runs (2026-09/10), when the AGX's `socialtech:robot` had GLIM and GTSAM installed (the GLIM queues ran in it) and `~/bench_ws` held the other methods compiled on the board. The `robot` stage no longer does.

| Scripts | Method | Status |
|---|---|---|
| `bench_voxelslam*.sh`, `vs_queue1.sh`, `jetson_*.sh`, the Voxel-SLAM calls of `bench_own_queue.sh` and `bench_reps_queue.sh` | Voxel-SLAM | Run in `socialtech:mapper`, the default (`IMAGE=` overrides). A port compiled in `~/bench_ws` overrides the image's. |
| `bench_fastlio*.sh` and the FAST-LIO calls of the queues | FAST-LIO2 | FAST-LIO2 is in the `robot` stage, but `fastlio_run.sh` sources `/ws/install` unconditionally, so it needs the `~/bench_ws` overlay too. Not rerun since the image changed. |
| `bench_iglio*.sh`, `bench_r05_queue.sh` | iG-LIO | Record only: iG-LIO is in no image (it was compiled in `~/bench_ws`). `bench_r05_queue.sh` also runs FAST-LIO. |
| `glim_queue11-24.sh` | GLIM | Record only: GLIM is in no image. The header of each says what the experiment varied; the results are in `notes/slam_seleccion.md` and `notes/glim_*.md` of the SocialTech repo. |

## Make a map

```bash
docker build --target mapper -t socialtech:mapper docker/
docker run --rm --network host -v <bags>:/bags:ro -v <maps>:/out -v $PWD/tools/slam_bench:/tools:ro socialtech:mapper \
  /tools/make_map.sh [--reuse-slam] /bags/<bag_dir> /out/<name>
```

`make_map.sh` counts the beams with `libsocialtech_mapping_beams.so` (in the `mapper` image; else `BEAMS_LIB=<path>`) and checks it before the replay, so a host without it fails in a second. After the SLAM, `make_map.py` takes about 50 s on the AGX for a 366 s bag (assembling the cloud 19 s, the unknown-cell fill 29 s), measured 2026-11-01.

Outputs of `<name>/`: `map.pgm` + `map.yaml` (the 2D map; its `# T_map_world` comment is what `pose_eval.py` reads), `map.ply` (the raw SLAM cloud), `map_clean.ply` (the cloud without ghosts, whole height, floor z = 0, in the map frame: `tools/global_reloc_3d/prepare_map.py --map-dir` builds the relocalizer prior from it) and `report.json`. A folder made before `pipeline: 2` has a different `map_clean.ply` (only the obstacle band over occupied cells): `prepare_map.py` refuses it; regenerate it with `--reuse-slam`.

Cleaning rule (constants of `ghosts.py` and `grid2d.py`, recorded in `report.json`): ghost test on the centroid of 5 cm voxels over the whole height (margin 3 cm, >= 10 observations, hit ratio < 0.10), density halo 0.10 m / 0.10, islands under 10 cells of 5 cm freed, no swept strip, Voxel-SLAM `blind` 0.1. Evidence: `notes/mapeo_optimizacion_2026-10-30.md` and `notes/mapeo_pipeline_*_2026-11-01.md` in the SocialTech repo. There are no options to reproduce an older map: `report.json` says which commit and rule made each one.

Running it again into the same `<name>` replaces the earlier run, `vs/run` (poses and scans) included: copy `vs/run/alidarState.txt` away first if you still need it.

`LIDAR` and `IMU` set the topics, `CEIL=30` makes an outdoor map, `PLAY_RATE` sets the replay speed (default 2). Voxel-SLAM follows the timestamps of the data, not the clock, so the speed should not change the result as long as no message is lost (the 366 s bag gave byte-identical poses at 1x and at 2x, N = 2 runs, one bag; if a run warns that scans were lost, relaunch with `PLAY_RATE=1`): keep the board otherwise idle while mapping (Elevator01 splits its sessions differently at 3x). On an idle Orin NX, eco01 at 0.5x and at 1x gave byte-identical poses and the same PGM; with several jobs on one AGX, pose counts of the same bag varied by up to 12 %, probably from lost messages. Check `report.json` (resets, `end_to_start_m`) before trusting a map. When the robot ends where it started (our bags do), check `end_to_start_m`: 1-2 m is normal.

Rules learned the hard way:
- One `ROS_DOMAIN_ID` per method (GLIM 11, FAST-LIO2 / iG-LIO 77, Voxel-SLAM 78). GLIM also listens to live topics while replaying a bag; a second player with the same topic names corrupts its input.
- Compare methods at their own best `evo_ape --t_offset` as well as at 0: KISS-ICP stamps the end of the scan, GLIM its own frame stamp.
- Do not send SIGINT to background jobs started from a script (ignored); use a self-terminating recorder.
