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
| `fastlio_run.sh`, `iglio_run.sh`, `voxelslam_run.sh` | run one method on one bag inside `socialtech:robot` (workspace at `/ws`, tools at `/tools`, output at `/out`) |
| `eval_all.py` | One markdown table from `<root>/<method>/<sequence>/*.tum`: APE (SE(3) alignment, offset 0 and best offset) for mocap sequences, ArUco closure for Corridor01 / Elevator01 |
| `traj_metrics.py` | Ground-truth-free table for our own bags (flat floor, return to start): path, end-start distance, z / roll / pitch range, divergence between runs |
| `collect_results.sh` | Copy all trajectories from the AGX into the layout `eval_all.py` reads (GLIM runs, competitors at 0.5x; skips contaminated runs) |
| `floor_tilt.py` | Ground-truth-free flatness of GLIM dumps: floor plane per submap, global floor tilt from x-y, floor residual (`floor_tilt.py [-v] <run_dir>...`) |
| `static_offset.py` | IMU-vs-floor/ceiling offset of the Livox from raw bags (still windows); prints a `T_lidar_imu` overlay that cancels the GLIM map tilt |
| `ply_to_map.py` | PLY map -> levelled floor -> `map_server` PGM + YAML (floor and everything above 1.80 m removed). XYZ only, any SLAM; refuses when the floor cannot be trusted. Needs scipy (host/laptop, not in the robot image). Choices and evidence: `notes/ply_to_map_suelo.md` in the SocialTech repo |
| `make_map.sh` | Bag -> 2D map in one command, inside the `mapper` image: Voxel-SLAM (`degrade_bound` 100000) -> `vs_to_ply.py` -> `ply_to_map.py` -> `map_visibility.py`. Leaves `map.pgm`, `map.yaml` (with `T_map_world`), `vs/run/alidarState.txt` (the reference for `pose_eval`), `world.ply` (the raw Voxel-SLAM cloud: final poses, 5 cm thinning), `map_clean.ply` (the levelled, cleaned cloud the PGM was made from) and `report.json` (resets, loop-closure error, sensor height, cell counts). Exits non-zero naming the step, and leaves no map, when a step fails or `ply_to_map` refuses the floor |
| `vs_to_ply.py` | Voxel-SLAM output (one body-frame PCD per frame + `alidarState.txt`) -> one world-frame PLY |
| `map_report.py` | The `report.json` of a `make_map.sh` run |
| `voxelslam_degrade.sed` | `degrade_bound: 100000` for `voxelslam_run.sh` (`VS_SED`): the port's default 10 resets on bags with featureless stretches and never finishes |
| `agx/` | wrappers and queues as run on the AGX (`docker run` lines, DDS domain per method) |

## Make a map

```bash
docker build --target mapper -t socialtech:mapper docker/
docker run --rm --network host -v <bags>:/bags:ro -v <maps>:/out -v $PWD/tools/slam_bench:/tools:ro socialtech:mapper \
  /tools/make_map.sh /bags/<bag_dir> /out/<name>
```

`LIDAR` and `IMU` set the topics, `CEIL=30` makes an outdoor map, `PLAY_RATE` sets the replay speed (1.0). Voxel-SLAM follows the timestamps of the data, not the clock, so the speed should not change the result as long as no message is lost: keep the board otherwise idle while mapping (Elevator01 splits its sessions differently at 3x). On an idle Orin NX, eco01 at 0.5x and at 1x gave byte-identical poses and the same PGM; with several jobs on one AGX, pose counts of the same bag varied by up to 12 %, probably from lost messages. Check `report.json` (resets, `end_to_start_m`) before trusting a map. When the robot ends where it started (our bags do), check `end_to_start_m`: 1-2 m is normal.

Rules learned the hard way:
- One `ROS_DOMAIN_ID` per method (GLIM 11, FAST-LIO2 / iG-LIO 77, Voxel-SLAM 78). GLIM also listens to live topics while replaying a bag; a second player with the same topic names corrupts its input.
- Compare methods at their own best `evo_ape --t_offset` as well as at 0: KISS-ICP stamps the end of the scan, GLIM its own frame stamp.
- Do not send SIGINT to background jobs started from a script (ignored); use a self-terminating recorder.
