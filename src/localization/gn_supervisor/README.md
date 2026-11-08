# gn_supervisor

Global relocalization of the robot in the 3D prior of its map, and the supervisor that decides when to call it. Everything here runs on the robot (or is a test of it, or builds the prior). The previous pipeline of the AMCL era (boot-time node, permit/gate chain, evaluation campaigns) was removed on 2026-11-08; it is in the git history before that commit.

`scripts/gn_supervisor_node.py` (started by `socialtech_bringup`'s `nav_live.launch.py`; on by default, `SUPERVISOR=0` turns it off) watches `gn_loc`; when gn_loc is lost, starved of scans, silent or drifting it holds Nav2, restarts FAST-LIO2, relocalizes and gives gn_loc a new `/initialpose`. Installed in `lib/gn_supervisor` (flat: the modules import each other and the node starts `search_v2.py` as a script), with the worker's binaries. The import closure of the node and of its search worker is exactly these modules (`test/test_runtime_closure.py` keeps it that way):

| module | role |
|---|---|
| `gn_supervisor_node.py`, `gn_supervisor_core.py` | the ROS node and the decision logic (no ROS in the core) |
| `reloc_live.py` | one relocalization attempt: waits for stillness, restarts FAST-LIO2, exposes, runs the search, checks nobody moved, hands the pose over |
| `imu_ready.py` | the IMU stillness tests (`window_ok`, `expected_samples`) |
| `lio_submap.py`, `livox_frames.py` | the raw Livox frames and IMU of the exposure into the submap the search fits |
| `search_v2.py` (also the multi-hypothesis policy), `correlative_search.py`, `corr_cpp.py`, `pointcloud.py`, `verify.py` | the worker: FFT correlative search + `register_v2` (GICP) + the decision (`verify`) |
| `reloc_core.py` | the handoff maths (`handoff_pose`, `compose_planar`) and `dump` |

Setup helper, not imported by the runtime: `prepare_map.py` builds the prior (`<map>/prior/map.xyz`) from the `map_clean.ply` of a map. `ops/make_map.sh` runs it at the end of a mapping run and `ops/nav_live.sh` runs it on a map that has none.

## Binaries (built by CMake, installed next to the scripts)

| file | what |
|---|---|
| `src/register_v2.cpp` -> `register_v2` | GICP (small_gicp) from every candidate pose of `--init-list`, against the map; one `NN.json` per candidate. Linked with the TBB build of small_gicp (`GN_SUPERVISOR_SMALL_GICP_DIR`: the image keeps a second, single-threaded one for `level_corrector`) |
| `src/corr_search.cpp` -> `libcorr_search.so` | the hot part of `correlative_search.py` (MapField, correlate, `yaw_peaks`, the yaw loop with the skip bar), a C API for ctypes (`corr_cpp.py`). pocketfft (BSD-3, the FFT scipy.fft is built on) is header-only in `third_party/pocketfft`, commit in `POCKETFFT_COMMIT`. `-O3` without `-march`: one binary for every Orin |

`search_v2.py` uses the C++ search by default (`--corr-backend python`: the numpy path, for an environment without the library); a missing library is an error, never a silent fallback. Differences from the numpy path: exact EDT by lower envelope (same distances as scipy's feature transform), per-yaw ties among equal scores broken by cell index, `rot_z` product without BLAS. Measured on the AGX, 17 real inputs x 2: decisions 34/34 equal, primary poses identical, scores within 2e-7 (`CppBackendTest`, skipped without the library; point `CORR_SEARCH_LIB` at it).

`-DGN_SUPERVISOR_PYTHON_ONLY=ON` installs the Python alone (an overlay of this source over an image, as `OVERLAY=1` of `ops/nav_live.sh` and the simulation scripts do): the launch finds the binaries in the first prefix of the ament path that has them.

## Tests

`test/test_runtime_closure.py` is a deliberate guard: it fails when a module appears in `scripts/` that is neither in the table above nor the documented setup helper. Put analysis scripts elsewhere (the team workspace repo), or add the module to the table and to the test.

`python3 -m pytest -q src/localization/gn_supervisor/test` (scipy and numpy; no ROS). `test_register_synthetic.py` skips without `REGISTER_BINARY`, `CppBackendTest` without `CORR_SEARCH_LIB`. Expect `test_reloc_live.py` ~15 s.
