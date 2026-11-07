# global_reloc_3d

Global relocalization of the robot in the 3D prior of its map, and the supervisor that decides when to call it. **Two eras live in this directory**; only the first runs on the robot.

## 1. What the robot runs (the supervisor era)

`gn_supervisor_node.py` (started by `ops/nav_live.launch.py` with `SUPERVISOR=1`) watches `gn_loc`; when gn_loc is lost, starved of scans, silent or drifting it holds Nav2, restarts FAST-LIO2, relocalizes and gives gn_loc a new `/initialpose`. The import closure of the node and of its search worker is exactly these modules (`test_runtime_closure.py` keeps it that way):

| module | role |
|---|---|
| `gn_supervisor_node.py`, `gn_supervisor_core.py` | the ROS node and the decision logic (no ROS in the core) |
| `reloc_live.py` | one relocalization attempt: waits for stillness, restarts FAST-LIO2, exposes, runs the search, checks nobody moved, hands the pose over |
| `imu_ready.py` | the IMU stillness tests (`window_ok`, `expected_samples`) |
| `lio_submap.py`, `livox_frames.py` | the raw Livox frames and IMU of the exposure into the submap the search fits |
| `search_v2.py`, `search_tiles.py`, `correlative_search.py`, `corr_cpp.py`, `offline_s2.py`, `verify.py`, `cpp_search/` | the worker: FFT correlative search + `register_v2` (GICP) + the decision (`verify`) |
| `reloc_core.py` | shared helpers; **two thirds of it (368 of 552 lines) is the legacy node's** (below) |
| `register_v2.cpp` | built into `register_v2` (+ `libsmall_gicp.so`, `libcorr_search.so`), staged separately |

Setup helpers (used by `stage_supervisor.sh`/`prepare_live_prior.sh` of the team workspace, not at runtime): `prepare_map.py` (prior from `map_clean.ply`), `map_check.py`, `handoff_pose.py`.

## 2. The previous pipeline (AMCL era): not used by the robot

A boot-time relocalizer node that fed Nav2's AMCL, with its permit/gate chain and the evaluation campaigns of that phase. Kept as the evidence base of the notes, **not staged for use**, and nothing in section 1 imports it.

- Node and gate: `reloc_node.py`, `nav_gate.py`, `permit_exposure.py`, `permit_stub.py`, `can_deadman.py`, `gate_*.py`/`gate_*.sh`, `fault_relay.py`, `hang_exe.sh`, `slow_exe.sh`, `launch/reloc_online.launch.py`, `docker/Dockerfile.{online,gate-can}`.
- Campaign scripts (written for the NX layout): `batch_*.sh`, `run_online.sh`, `online_inner.sh`, `record_session.sh`, `run_handoff.sh`, `f2_check.py`, `f3_summary.py`, `restart_summary.py`, `gen_restart_cases.py`, `latency_summary.py`, `summarize_*.py`, `analyze_online.py`, `proc_cost.py`, `mem_trend.py`, `io_stall_audit.py`, `topic_loss.py`, `bag_stream_integrity.py`, `ready_windows.py`.
- In `reloc_core.py`: `Attempt`, `Acquisition`, `ImuGate`, `HandoffMonitor`, `InputIntegrity`, `OdomPlausibility`, `LocalizationMonitor`, `MapSupervisor`, `PermitGate`, `s3_viable`, `planar_delta`, `trim`.
- `register.cpp` (v1): "kept frozen for the first result", superseded by `register_v2.cpp`.

## 3. Offline evaluation of the search (reference poses only on the evaluator side)

`evaluate.py`, `evaluate_online.py`, `evaluate_batch*.sh`, `reevaluate.sh`, `diagnose_*.py`, `convert_reference.py`, `map_link.py`, `make_hole_map.py`, `perturb_input.py`, `plot_overlay.py`, `plot_band_overlay.py`, `retrieval_recall.py`, `select_windows.py`, `support_breakdown.py`, `band_diag_batch.sh`, `capture_fastlio.sh`. Useful to re-validate the search on stored sessions; they import the runtime modules, never the other way round.

## Tests

`python3 -m pytest -q test_*.py` inside the robot image (`source /ros_env.sh` first; scipy, numpy and rclpy are needed). `test_register_synthetic.py` skips without `REGISTER_BINARY`. Expect `test_reloc_live.py` ~15 s.

## Known debt

- `stage_supervisor.sh` (`notes/gn_reloc_scripts/` of the SocialTech workspace repo) copies this whole directory to the robot, section 2 and 3 included. A split into `runtime/` and the rest, with staging of the first only, is the clean fix; it moves ~65 files and rewrites the paths the notes cite, so it waits for a decision.
- `reloc_live.StillWindow` and `reloc_core.ImuGate` are two implementations of the same sliding window; the first one is the one with the fixes (`expected_samples`, clock step, noise scale).
