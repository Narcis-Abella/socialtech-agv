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
| `agx/` | wrappers and queues as run on the AGX (`docker run` lines, DDS domain per method) |

Rules learned the hard way:
- One `ROS_DOMAIN_ID` per method (GLIM 11, FAST-LIO2 / iG-LIO 77, Voxel-SLAM 78). GLIM also listens to live topics while replaying a bag; a second player with the same topic names corrupts its input.
- Compare methods at their own best `evo_ape --t_offset` as well as at 0: KISS-ICP stamps the end of the scan, GLIM its own frame stamp.
- Do not send SIGINT to background jobs started from a script (ignored); use a self-terminating recorder.
