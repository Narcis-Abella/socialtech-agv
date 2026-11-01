# gn_loc

Gauss-Newton scan-to-map localizer. A lifecycle node that stands where `nav2_amcl` stands: it computes `map -> odom`, publishes `amcl_pose` and takes the same `/initialpose`, so Nav2 needs no change. The core (`include/gn_loc/gn_core.hpp`) has no ROS dependency.

| | |
|---|---|
| In | `scan` (LaserScan, base frame), `map` (latched OccupancyGrid), `initialpose` (its covariance is the prior), TF `odom -> base_footprint` |
| Out | TF `map -> odom`, `amcl_pose`, `gn_loc/status` (String, 1 Hz), `gn_loc/need_reloc` (Bool, latched) |
| States | TRACKING, LOST. A scan is bad below `min_inliers` (0.25) or `min_tight` (0.50). LOST after `lost_s` (3 s) of bad scans. Recovery needs 10 scans with consistency >= `accept_tight` (0.60). `need_reloc` is set after `reloc_after_s` (15 s) in LOST, for whoever starts a global relocalizer; a new `/initialpose` ends LOST |
| Limits | A scan with fewer than 20 valid beams is not counted as bad: with a covered lidar GN stays TRACKING at 0/0 and never goes LOST. Watch `gn_loc/status` from outside (see `tools/global_reloc_3d/gn_supervisor_core.py`) |

Parameters: `ros2 param list /gn_loc` after `configure`; the defaults are declared in `src/gn_loc_node.cpp`.

## Use

```
ros2 launch gn_loc gn_loc.launch.py map:=MAP.yaml use_sim_time:=false
```

Starts `map_server`, `gn_loc` and a lifecycle manager for both. The live stack (`ops/nav_live.sh`, a separate change) selects it with `LOCALIZER=gn` and AMCL with `LOCALIZER=amcl`.

## Build and tests

The image builds it in its own colcon step after the Nav2 packages (`/opt/socialtech_loc`), without tests. The 13 gtests of the core run in a separate build:

```
colcon build --packages-select gn_loc --cmake-args -DBUILD_TESTING=ON && ./build/gn_loc/test_gn_core
```

`gn_offline` runs the core over a replay bag (study tool, not used by the node).
