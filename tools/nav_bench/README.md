# nav_bench

Shared pieces of the localization/navigation stack that outlived the AMCL bench (stage A: FAST-LIO2 + Nav2 AMCL replayed over a bag and scored against the Voxel-SLAM trajectory; removed on 2026-11-08, with its AMCL tuning table and scripts, in the git history before that commit).

| File | What |
|---|---|
| `map_frame.py` | `level` (the one command `ops/nav_live.sh` calls: `qx qy qz qw` of R_L, FAST-LIO2's world frame to the floor-levelled, wall-aligned frame) and the helpers `planar_odom.py` and `live/live_paths.py` import (quaternions, poses, `read_map_yaml`, `reference_map`). numpy only; test: `test_map_frame.py` |
| `planar_odom.py` | Python reference of the `odom -> base_footprint` maths (x, y, yaw of the body, z = body z - sensor height, no roll/pitch). The robot runs the C++ node `docker/ros/planar_odom`, checked against it by `test_planar_odom_node.py` (inside the image); `test_planar_odom.py` |
| `procmon.py` | the one CPU/RAM sampler of the repo (used by `tools/nav_sim` and the Voxel-SLAM cost scripts); `test_procmon.py` |
| `config/scan.yaml` | the `/scan` cut (0.10 - 1.80 m above the floor): the obstacle band of the map, in sync with `grid2d.rasterize` (floor band 0.10 m, `CEIL` of `make_map.sh`); read by `ops/nav_live.launch.py` and the Nav2 parameters |
| `live/live_paths.py`, `live/laptop_env.sh`, `live/bench_a.rviz` | the paths node `ops/nav_live.launch.py` starts (`/gn_path` etc.), and the laptop's ROS environment and rviz config used by `ops/rviz_live.sh` |

## Frames
FAST-LIO2's world frame is the IMU frame at start, **not** gravity-aligned. `odom` is that frame rotated by `R_L` (`map_frame.py level`) so the PGM's floor is z = 0 and the axes follow the walls. The map YAML must carry the `# T_map_world` comment block written by `make_map.sh`.

## Tests
```bash
cd tools/nav_bench && for t in test_*.py live/test_*.py; do python3 -B $t || break; done   # needs numpy and, for live_paths, ROS sourced
```
