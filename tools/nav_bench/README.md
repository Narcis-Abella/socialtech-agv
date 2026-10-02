# nav_bench — localization bench, stage A

Replays one recorded bag over **its own map**: FAST-LIO2 odometry + Nav2 AMCL on the deskewed scan, then scores AMCL against the Voxel-SLAM trajectory of the same bag.
Stage A is the best case (the map holds exactly what the LiDAR sees): a failure here is a real failure, a pass only rules out base errors. The initial pose is pushed
away on purpose so convergence is measured, not just tracking.

| File | What |
|---|---|
| `run_bench_a.sh` | One run, inside the `robot` image |
| `launch/bench_a.launch.py`, `config/` | FAST-LIO2, levelled odom frame, planar `base_footprint`, `pointcloud_to_laserscan`, `map_server`, AMCL |
| `planar_odom.py` | `odom -> base_footprint` TF: x, y, yaw of the body, z = body z - sensor height, no roll/pitch |
| `record_poses.py` | `/Odometry` and `/amcl_pose` to text files |
| `pose_eval.py` | `init`, `level`, `check` and `report` (numpy only); tests: `test_pose_eval.py`, `test_planar_odom.py` |
| `scan_vs_map.py` | Share of `/scan` endpoints on occupied map cells, at the reference pose: checks the z cut, sensor height and frames |

## Inputs per bag
- `<bag>`: Livox `CustomMsg` bag (FAST-LIO2 ignores per-point time in `PointCloud2`); `/livox/lidar`, `/livox/imu`.
- `map.yaml` + PGM from `ply_to_map.py`: the YAML must carry the `# T_map_world` comment block (levelling + wall alignment applied to the SLAM PLY).
- `alidarState.txt` of the Voxel-SLAM run **that produced that map's PLY** (`pose_eval.py check`: the path must lie on free cells, not on walls).
- Sensor height above the floor: `pose_eval.py check` prints it (`sensor_height_m`).

## Run
```bash
docker run --rm --name navbench --cpus 4 --network host -e PLAY_RATE=1.0 \
  -v ~/rosbags:/bags:ro -v ~/nav_data:/data:ro -v <repo>/tools/nav_bench:/nb:ro -v ~/nav_out:/out \
  socialtech:robot /nb/run_bench_a.sh /bags/custom/<bag> /data/<bag>/map.yaml /data/<bag>/alidarState.txt <sensor_height_m> /out/<run>
```
Docker creates the output as root: one new `/out/<run>` per run. Own `ROS_DOMAIN_ID` (79) so other benches on the board never mix in; keep the board idle at 1x
(lost `best_effort` messages make the filters diverge). `/out/<run>/report.json` has the metrics; thresholds in `pose_eval.py` are provisional.

## Tests
```bash
cd tools/nav_bench && for t in test_*.py; do python3 -B $t || break; done   # needs numpy; -B: no stale .pyc
```

## Frames
FAST-LIO2's world frame is the IMU frame at start, **not** gravity-aligned. `odom` is that frame rotated by `R_L` (`pose_eval.py level`) so the PGM's floor is
z = 0 and the axes follow the walls. The z cut of `config/scan.yaml` (0.10 - 1.80 m) is the obstacle band of `ply_to_map.py` (`--floor-band`, `--ceil`): keep them in sync.

## AMCL tuning
`config/amcl.yaml` holds the tuned values; Nav2 defaults (alpha 0.2, `update_min_d` 0.25 m, 500-2000 particles) lose the pose from a 1 m / 20 deg offset. Measured on the
`eco_-1_01` and `eco_-1_03` bags, each on its own map, 3 initial offsets each (1 m/20 deg, -1/-20, 2/40), replay 1x on an AGX Orin with `jetson_clocks`:
| | Before tuning (alpha 0.02, 500-2000 particles) | Now (alpha 0.01, 2000 fixed) |
|---|---|---|
| Convergence | 3-16 s | 3-28 s |
| Yaw error p95 | 1.3-1.9 deg | 1.0-1.2 deg |
| Position error median / p95 / max | 3-5 cm / 7-10 cm / 23-30 cm | same |
| Correction applied per update, p95 | 3.3-4.3 cm | 1.8-2.6 cm |
| CPU, whole stack (FAST-LIO2 + AMCL + scan), % of ONE core | 23 (p95 32) | 31 (p95 45) |
| RSS | ~300 MB | ~300 MB |
Notes: alpha 0.005 was rejected, from large offsets it converged late (100-243 s) with 3000-5000 particles too. 5000 particles cost 35 % of a core for AMCL alone.
Position error is at the floor of the reference and the 5 cm map: more particles only improve yaw. Measured on the AGX only (NX/Nano cost not measured); stage B (a bag over another
bag's map) not run. Pose jitter (`map->odom` zig-zag, ~50 % sign reversals between consecutive corrections) is not judged offline: check it live in rviz2.
