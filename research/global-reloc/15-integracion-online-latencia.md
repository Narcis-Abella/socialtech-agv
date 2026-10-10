# 15 — Online integration, end-to-end latency and fault behaviour

Date: 2026-10-10. Branch `feat/global-reloc`. All data DEVELOPMENT (same-run or cross-session maps; reference = Voxel-SLAM). **No reliability claim.** Labels: **V** measured, **I** inferred, **NV** not validated.
Raw data on the NX: `/home/jetson/work/reloc-online-20261009/runs/` (one dir per run: `work/attempt*/metrics.json`, `evaluation.json`, `cost.csv`, `tegrastats.log`, `launch.log`). Tables: `results/15-online/`.

## Architecture (V, runs end to end under ROS 2 Jazzy on the NX, replay 1x)
`tools/global_reloc_3d/`: `reloc_core.py` (ROS-free: state machine, deadline, IMU gate, acquisition, handoff maths, loss stats; 12 unit tests), `reloc_node.py` (rclpy), `launch/reloc_online.launch.py`, `online_inner.sh`/`run_online.sh` (runner), `batch_online.sh`, `batch_faults.sh`, `evaluate_online.py` (evaluator only), `summarize_online.py`, `analyze_online.py`, `proc_cost.py`, `fault_relay.py`. Image: `socialtech:reloc-online` = `robot-verify` + `python3-scipy` (`docker/Dockerfile.online`; `robot` stage untouched).

Flow: `/livox/imu` -> 2 s stationary gate (`imu_ready` test, unchanged) -> node spawns a cold FAST-LIO2 (own process group, dies with the node via PDEATHSIG) -> accumulate body clouds until S3 stop (radius 2.8 m / path 6 m from own odometry) or 30 s -> `lio_submap.assemble()` (same code as the offline path) -> `search_v2.py` worker subprocess (FFT search, small_gicp, frozen `verify.decide`, killed at the 60 s deadline) -> `ACCEPTED` -> `/initialpose` (std 0.15 m / 5 deg, PROVISIONAL) only from `HANDOFF` -> AMCL confirmed when `/amcl_pose` stays within 0.30 m / 8 deg of the pose propagated with FAST-LIO odometry for >=3 updates spanning >=5 s. The node also publishes `odom->camera_init` (level) and `odom->base_footprint` (same maths as `planar_odom`; sensor height 0.53 m PROVISIONAL).
States: IDLE, WAIT_SENSORS, IMU_CONDITIONING, LIO_STARTING, ACCUMULATING, SEARCHING (covers refine+verify inside the worker), ACCEPTED, HANDOFF, LOCALIZED | REJECTED | TIMEOUT | HANDOFF_FAILED | ERROR. Illegal transitions raise; only `HANDOFF` (reachable only from `ACCEPTED`, reachable only from `SEARCHING`) publishes. Thresholds are not node parameters. Retry: new attempt = new buffers, cold FAST-LIO, `min_stamp` refuses messages of the previous process.

## Clock definitions (protocol 02, not redefined)
- Protocol origin = first permitted sensor input (first IMU/LiDAR message): `total_time_protocol = accepted - first_input`; deadline 60 s from there; acquisition limit 30 s from there.
- B localization startup = node start -> first input: dominated by my runner's 12 s lead for AMCL bring-up (not a system cost). C algorithm runtime = `search_wall` (+ build). D AMCL handoff = `/initialpose` published -> confirmation.
- A cold boot: **NOT measured** (needs a physical power cycle and an autostart service; host changes forbidden). Only a component: `systemd-analyze` on the host: 6.5 s kernel + 10.7 s userspace = 17.1 s to graphical.target (**V**, read-only). Cold boot + 60 s as one requirement would need ~17 s + container/stack start + ~48 s = about 70 s (**I**, not measured): it does NOT fit one 60 s budget; the protocol 60 s starts at first sensor input.

## Results (S3 mode, replay 1x, idle board, `jetson_clocks` active, N = runs)
Positives (own map of the bag, or eco03 against the eco01 map), b1+b3 valid runs N=22: 18 correct_accept, 3 unscorable (Voxel-SLAM reference gap at the stamp), 1 `error_no_pose` (hospital c02: the IMU gate passed only after 39.7 s, the 30 s acquisition limit expired with 1 cloud; REJECTED path hit the illegal transition, fixed). Not run (bag ends < 75 s after start): eco01 c05, eco03 c03, hospital c09. **0 wrong_accept** (Wilson 95 % upper 0.17 on 18 scored + 0 errors; development windows, not a reliability rate). All 18 correct accepts reached `LOCALIZED`.
Negatives (wrong map eco<->hospital N=7, hole map N=3): 10 runs, 0 poses published (9 REJECTED `reject_geometry`; 1 ERROR: eco03 c01 cross, FAST-LIO2 produced 2 odometry messages and stalled, the `lio_stalled` watchdog was added afterwards). Wilson upper 0.28 for 0/10.

| quantity (correct accepts, N=18) | median / p95 / max |
|---|---|
| IMU gate (s) | 3.0 / 4.9 / 7.4 |
| LIO ready (s) | 0.6 / 0.7 / 0.7 |
| submap exposure (s, motion-dependent) | 18.8 / 27.2 / 27.4 |
| submap build (s, worker thread) | 3.5 / 5.6 / 5.7 |
| FFT incl. confirmation (s) | 7.1 / 9.6 / 9.7 |
| small_gicp (s) | 3.8 / 4.6 / 4.6 |
| verification/other in worker (s) | 0.9 / 1.0 / 1.0 |
| search worker wall (s) | 11.7 / 15.2 / 15.3 |
| handoff publish -> AMCL confirmed (s) | 5.3 / 8.6 / 9.8 |
| **total, protocol (first input -> accepted)** | **39.8 / 47.1 / 47.8** |
| first input -> LOCALIZED | 45.8 / 52.6 / 53.4 |
| pose error vs reference | XY median 0.03 m, max 0.08 m; yaw max 1.2 deg |
| whole stack CPU (% of one core, excl. bag player) | mean 69, p95 213 (search bursts) |
| PSS peak sum (MB) / RSS peak sum | 339 / 496 (max 418 / 594) |
| search worker maxrss (MB) | 87 / 93 |
| GPU (tegrastats GR3D) | 0 % in all runs; no CUDA process |
| IMU loss at replay (%) | 7.1 / 8.2 / 8.2; an independent witness subscriber sees 7.0 / 8.2 / 8.2 |

**60 s (V for S3 windows with motion):** accepted within 47.8 s worst of 18; 60 s met on these windows. The submap term is the lever: it equals the time the robot needs to move 2.8 m (or 30 s). **I:** a robot that stays still at boot hits the 30 s limit: ~3+0.6+30+3.5+12 = ~49 s (not measured; `f06` with a 5 s limit accepted and localized, so the time-limit path works). S2 (fixed 3 s exposure, **N=1 effective** on eco03/eco01 map): accepted 18.1 s, LOCALIZED 23.3 s, error 0.028 m; one duplicate row (`s2b1`) is not independent and is not counted.
Noise floor: repeated windows (eco01 c00/c02/c04, eco03 c00/c02, hospital c00 in b1/b3/b3a) give protocol totals within about +-2.5 s; accept decisions identical.

## Fault behaviour (V, eco01 c04 unless noted; no pose published in any failing case)
| case | outcome |
|---|---|
| LiDAR missing / IMU missing | TIMEOUT `sensors_unavailable` at 60.0 s |
| 30 % LiDAR drop / 30 % IMU drop | ERROR `lio_not_ready_5s` (7.5 s) / TIMEOUT `imu_not_stationary` (gate needs >=95 % samples) |
| IMU stamps +1000 s | ERROR `lio_not_ready_5s` |
| moving at start | TIMEOUT `imu_not_stationary` at 60 s; **second attempt** (max_attempts=2, new cold FAST-LIO, 0 stale messages) LOCALIZED, error 0.020 m |
| small_gicp binary `false` / missing | REJECTED `reject_no_hypothesis` / ERROR `search_worker_exit_1` |
| FAST-LIO2 exits / hangs | ERROR `fastlio_exited_1` (2.3 s) / `lio_not_ready_5s` |
| AMCL absent | HANDOFF_FAILED `amcl_not_available` (pose not published: no subscriber) |
| deadline 25 s / 40 s (injected) | TIMEOUT `acquisition` at 25.2 s / accepted and localized at 45.8 s after the 40 s decision |
| node SIGKILL mid-attempt | 0 orphan FAST-LIO2; fresh node starts attempt 1 with clean state (joined mid-motion: TIMEOUT `imu_not_stationary`, no pose) |
| acquisition limit 5 s | accepted + LOCALIZED, error 0.028 m (the gate, not the exposure, decides) |
| wrong map / hole | REJECTED `reject_geometry` |
Not tested: reversed-order stamps, ambiguous symmetric places, FAST-LIO2 divergence after acceptance.

## Mistakes and invalid runs (kept)
- dev1-dev13 (diagnostic): AMCL failed to confirm and 47 % of IMU messages were lost inside the node. Cause (**V**): subscribing `/livox/lidar` as a full `CustomMsg` converts ~20k points to Python objects, starving the single-threaded executor; fixed with a raw subscription (header stamp parsed from CDR) and an incremental S3 stop rule; remaining 7-8 % loss is upstream (the witness subscriber sees the same).
- dev5: `ROS_DOMAIN_ID` > 232 (Cyclone port range). b2: empty `fastlio_exe` launch argument crashed the node (all 24 runs invalid, marked `INVALID.txt`, rerun as b3). Metrics were missing for failures before any worker dir existed (fixed). `IllegalTransition ACCUMULATING->REJECTED` (fixed in `TRANSITIONS`, b3 rows kept as `error_no_pose`).
- Stall watchdog `lio_stalled` and the early-failure metrics fix were added after b3 and not re-run in S3 positives.

## Limits
Development data only; sensor->base extrinsic, sensor height and initial covariance are provisional (**NV**); the sensor faces up (ceiling) in these bags and the final mount looks down: recheck needed. Replay is best-effort at 1x on the same host as the consumers, so 7-8 % message loss is a property of the setup, not the live driver. The band gate of note 14 is not active. Cold boot not measured.
