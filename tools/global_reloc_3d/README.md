# global_reloc_3d

Global relocalization of the robot in the 3D prior of its map, and the supervisor that decides when to call it. Everything here runs on the robot (or is a test of it, or builds the prior). The previous pipeline of the AMCL era (boot-time node, permit/gate chain, evaluation campaigns) was removed on 2026-11-08; it is in the git history before that commit.

`gn_supervisor_node.py` (started by `ops/nav_live.launch.py` with `SUPERVISOR=1`) watches `gn_loc`; when gn_loc is lost, starved of scans, silent or drifting it holds Nav2, restarts FAST-LIO2, relocalizes and gives gn_loc a new `/initialpose`. The import closure of the node and of its search worker is exactly these modules (`test_runtime_closure.py` keeps it that way):

| module | role |
|---|---|
| `gn_supervisor_node.py`, `gn_supervisor_core.py` | the ROS node and the decision logic (no ROS in the core) |
| `reloc_live.py` | one relocalization attempt: waits for stillness, restarts FAST-LIO2, exposes, runs the search, checks nobody moved, hands the pose over |
| `imu_ready.py` | the IMU stillness tests (`window_ok`, `expected_samples`) |
| `lio_submap.py`, `livox_frames.py` | the raw Livox frames and IMU of the exposure into the submap the search fits |
| `search_v2.py`, `search_tiles.py`, `correlative_search.py`, `corr_cpp.py`, `offline_s2.py`, `verify.py`, `cpp_search/` | the worker: FFT correlative search + `register_v2` (GICP) + the decision (`verify`) |
| `reloc_core.py` | the handoff maths (`handoff_pose`, `compose_planar`) and `dump` |
| `register_v2.cpp` | built into `register_v2` (+ `libsmall_gicp.so`, `libcorr_search.so`), staged separately |

Setup helper (used by `prepare_live_prior.sh` of the team workspace, not at runtime): `prepare_map.py` (prior from `map_clean.ply`).

## Tests

`test_runtime_closure.py` is a deliberate guard: it fails when a module appears in this directory that is neither in the table above nor the documented setup helper. Put analysis scripts elsewhere (the team workspace repo), or add the module to the table and to the test.

`python3 -m pytest -q test_*.py` inside the robot image (`source /ros_env.sh` first; scipy, numpy and rclpy are needed). `test_register_synthetic.py` skips without `REGISTER_BINARY`. Expect `test_reloc_live.py` ~15 s.
