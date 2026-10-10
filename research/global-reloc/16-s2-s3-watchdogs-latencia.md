# 16 — Stationary S2 first, supervised S3 fallback: state machine, faults, latency, resources, message integrity

Date: 2026-10-10. Branch `feat/global-reloc`. Board: Orin NX 16 GB, `jetson_clocks` active (all cores 1984 MHz in every run), replay 1x, one run at a time on an idle board.
**All data are DEVELOPMENT** (same windows as notes 13-15, own-bag maps or eco03 on the eco01 map, reference = Voxel-SLAM, sensor facing up, mount provisional). **No reliability claim, no hardware test.** Level reached: `VALIDATED_IN_REPLAY` only (note 18, section 7).
Labels: **V** measured / read in code, **I** inferred, **NV** not validated.
Raw data on the NX: `/home/jetson/work/reloc-s2s3-20261010/` (`runs/<name>-<tag>/`, `batch-*.log`, `cases-*.csv`, `windows/`, `integrity/`, `band/`). Tables: `results/16-s2s3/` (`table-runs.csv`, `table-faults.csv`, `summary.md`, `band_rows.json`, `bag-integrity-*.json`). Figure: `figures/16-accept-time.png`.
Commands: `R=<root> tools/global_reloc_3d/batch_s2s3.sh cases-<tag>.csv <tag>`, `batch_faults_s2s3.sh <tag>`, `summarize_s2s3.py <runs> sa,sb,sn,sr,fin table.csv`.

## 1. Audit of the note-15 node (V, read in code) and what changed

| Finding | Evidence | Action |
|---|---|---|
| No S2 -> S3 path: `mode` was `s3` or a fixed-3 s `s2`, one stage per attempt | `reloc_node.py` / `reloc_core.TRANSITIONS` at `dce3863` | new `mode:=s2s3`; `s3` and `s2` kept unchanged as baselines |
| FAST-LIO2 was watched only in `ACCUMULATING`. It could die during the 10-15 s search and the accepted pose would still be published, propagated with the last odometry | `tick()`: no check in `SEARCHING`; `tick_handoff()` used `latest_odom` with no age test | `lio_fault()` in every state that needs odometry (exit or no `/Odometry` for 3 s -> `ERROR`); no publication from odometry older than 1 s; `HANDOFF_FAILED odometry_lost_after_pose` if it dies after the pose |
| "S2 N=1" of note 15 was not a stationary S2 | window `s2a3`: stationary episode 3.4 s, the 3 s exposure ended about 2.3 s after motion began (window list + run timing) | corrected here; real stationary starts derived from the IMU-certified episodes (section 3) |
| "7-8 % IMU loss at replay" (note 15) is a measurement artifact | section 7 | corrected; `StreamStats` now also reports `rate_hz_stamp` |
| Retry (`max_attempts=2`) starts a new 60 s clock | `new_attempt()` | kept as operational recovery only. Protocol 02 line 141: retries share the deadline, so a second attempt is never a protocol success |
| AMCL readiness = `/initialpose` has a subscriber | upstream `nav2_amcl/src/amcl_node.cpp` (branch `jazzy`): the subscription is created in `on_configure` (l. 259, 1571); a pose received while inactive is stored and applied on activation (l. 560-566, 294-295); malformed / wrong frame / outside the map is rejected with a log only (l. 539-557) | no change: a pose AMCL drops shows up as `HANDOFF_FAILED amcl_not_confirmed`. **NV**: the image's Nav2 build was not diffed against that branch head |
| Method / evaluator separation | node container never mounts a reference; `evaluate_online.py` runs afterwards in another container | unchanged; `truth_used: false` in every metrics file |

## 2. State machine (`mode:=s2s3`), one attempt, one clock

```
WAIT_SENSORS -> IMU_CONDITIONING -> LIO_STARTING -> S2_ACCUMULATING -> S2_SEARCHING -> ACCEPTED -> HANDOFF -> LOCALIZED
                                                          |                 |
                                                          +--> S2_INSUFFICIENT --> S3_SUPERVISION_REQUIRED --> S3_ACCUMULATING --> S3_SEARCHING --> ACCEPTED ...
                                                                    |                      |                                              |
                                                                 TIMEOUT                TIMEOUT                                        REJECTED
any state -> TIMEOUT (60 s) | ERROR;  HANDOFF -> HANDOFF_FAILED
```
Mapping to the requested names: INITIALIZING = `WAIT_SENSORS` + `IMU_CONDITIONING`; IMU_READY = `LIO_STARTING`; S2/S3_VERIFYING happen inside the search worker (`verify.decide`, frozen), so `*_SEARCHING` covers them.

Rules, each enforced in `reloc_core.TRANSITIONS` and unit-tested (`test_reloc_core.py`, 18 tests):
- S2 is always first. `S3_ACCUMULATING` is reachable only from `S3_SUPERVISION_REQUIRED`, which is reachable only from `S2_INSUFFICIENT`.
- `/initialpose` is published in one place, from `HANDOFF`, reachable only from `ACCEPTED`, reachable only from a `*_SEARCHING` state.
- **One clock.** `t_input` (first IMU or LiDAR message) is set once per attempt. S2 time is not given back: the 60 s deadline and the 30 s acquisition limit keep their origin through S3.
- **Budget check before S3:** `s3_viable` = at least 22 s left (worst build 5.7 s + worst search 15.3 s of note 15 + 1 s; a planning estimate, not an acceptance threshold). Otherwise `TIMEOUT s3_budget_insufficient`. While waiting for the supervisor the same test runs every tick: `TIMEOUT s3_not_authorized_in_budget` at 38 s.
- **Supervision:** the node asks through `/global_reloc/state` (`supervision_required: true`) and waits for `std_msgs/Bool true` on `/global_reloc/s3_authorize`. A grant is used only if it arrives while the request is pending (volatile subscription, earlier grants ignored). The node publishes no velocity and starts no motion. `s3_auto_authorize:=true` exists for replay only, where bags carry their own motion and no supervisor.
- **Limits from own odometry:** radius 2.8 m / path 6 m / 30 s. On radius or path the acquisition stops and the state message carries `supervisor_stop_required: true`. Nothing physical is stopped by the software.
- **Buffers:** S3 uses every cloud of the same attempt and the same FAST-LIO2 session, S2 clouds included (accumulation continues while S2 searches). Nothing crosses attempts (`min_stamp`). One search worker at a time (`start_search` raises otherwise).
- S2 exposure is a parameter (`s2_exposure_s`, protocol main value 3 s; 1 s and 5 s are the predeclared ablations).

**Proposed protocol amendment (not applied, for Narcís):** a human supervisor needs reaction time. With S2 deciding at 16 s (median) and S3 needing its motion to end by 30 s and 22 s of compute, the supervisor has until about 38 s to grant and should already be moving before 30 s. In replay this fits because the recorded motion starts by itself. On the robot it will often not fit. Options: (a) keep one 60 s clock and accept `TIMEOUT` as the normal outcome of a slow supervisor; (b) define supervised S3 as a second attempt with its own 60 s origin at the grant, reported separately and never pooled with first-attempt results. I recommend (b) for operation and (a) for the benchmark. Until decided, the code implements (a).

## 3. Stationary S2 (P0)

Windows. The certified windows of notes 13-15 start 2 s before the robot moves. For a real S2 the start must be the beginning of a stationary episode. `windows/<ds>-stat.csv` on the NX derives it from the same IMU-certified episodes (`z<nn>` = episode of `c<nn>`). Episodes long enough for gate 2 s + LIO 0.7 s + 3 s exposure (>= 6.2 s): eco01 z03 (20.9 s), hospital z00, z01, z04, z07 (8.8-18.4 s), z09 (7.2 s, bag ends too early to run); marginal: hospital z08 (6.0 s), eco01 z00 (5.4 s, bag start). That is 7 runnable starts in **5 independent 5 m blocks** (hospital z00/z04 and z07/z08 share a block). Own-odometry displacement during the S2 exposure was <= 0.021 m in all of them (**V**, `s2_odom_radius_m`).

| Config (own map) | N runs | Accepted in S2 | Went to S3 | Outcome | Accept at s, S2-accepted (med / p95 / max) | LOCALIZED at s |
|---|---|---|---|---|---|---|
| S2 3 s, stationary start | 10 (7 starts + 3 repeats) | 8 | 2 (hospital z00, twice) | 10 correct_accept | 16.3 / 18.3 / 18.9 (N=6 distinct) | 21.8 / 23.4 / 24.0 |
| S2 5 s, stationary start | 5 | 4 | 1 (hospital z00) | 5 correct_accept | 18.0 / 20.7 / 21.2 | 23.6 / 26.1 / 26.5 |
| S2 3 s, motion follows the 2 s gate (c-windows) | 17 | 14 | 1 | 15 correct_accept, 1 timeout (hospital c02), 1 error (hospital c03) | 16.6 / 21.4 / 22.1 (all 20 S2 accepts) | 21.9 / 26.7 / 27.2 |
| S2 1 s (diagnostic) | 22 | 3 | 18 | 20 correct_accept, 1 unscorable (reference gap), 1 timeout (hospital c02) | - | - |

Pose error of the S2 accepts: XY max 0.068 m, yaw max 0.51 deg (stationary starts: 0.064 m / 0.42 deg). Repeats are identical to the centimetre (z03: 0.008 m twice; `g00` 0.033 m / 0.066 deg twice): the search is deterministic for the same input.

What works: the node needs **no motion**. A 3 s stationary submap is enough for the frozen gate in 4 of 5 blocks.
What fails or is weak:
- hospital z00: S2 (1, 3 and 5 s) ends `ambiguous_no_alternative_margin`: support 0.996 but a distinct runner-up within 0.10. More stationary exposure does not help; the motion of S3 resolves it (accepted at 42 s, error 0.064 m). hospital z04, 2.9 m away, is accepted in S2. This is the gate abstaining, not an error.
- hospital c02: the IMU gate passes only at 39.7 s (same as note 15); the 30 s acquisition limit is already over, S2 has no submap, S3 is not viable: `TIMEOUT s3_budget_insufficient` at 40.5 s. Legitimate abstention.
- hospital c03 (1 of 2 runs): FAST-LIO2 hung in its own start-up (section 5).
- The 1 s exposure: the rule "check span >= 1.0 s" is met when the cut takes 11 clouds instead of 10 (stamp jitter at the boundary), so 3 of 22 runs were accepted in S2. All 3 are correct. The gate was not relaxed; per protocol the 1 s variant is a diagnostic and is not counted as S2 evidence.
- N: 5 stationary blocks, 2 datasets, development data. Nowhere near the 30 blocks the protocol asks for.

## 4. S2 -> S3 transition and negatives

Forced through S3 (1 s S2, replay grant): 18 S3 accepts + the 3 natural ones at 3/5 s = **21 S3 accepts, 20 correct, 1 unscorable**. Accept at 42.0 / 47.4 / 47.6 s, LOCALIZED at 48.0 / 53.4 / 56.7 s, XY max 0.084 m, yaw max 1.2 deg. The baseline `mode:=s3` on the same windows (N=5, new watchdogs active): accept at 38.2 / 39.9 / 39.9 s, 5 correct. So going through S2 first costs about 0-4 s on these windows (the S2 search shares the CPU while S3 accumulates), inside the +-2.5 s run-to-run spread of note 15 for most of them.
No grant: `TIMEOUT s3_not_authorized_in_budget` at 38.0-38.2 s, no pose (N=4: 2 stationary windows, `g19`, `g22`). Grant on the real topic at 20 s (`g20`, tag `gb`): S3 -> `LOCALIZED` at 45.8 s, error 0.018 m. Grant sent before the request (`g21`, and `g20` tag `ga` by a timing mistake of mine): ignored, as designed.

Negatives, `s2s3` 3 s, replay grant: wrong map N=13 (8 motion windows, 5 stationary starts), hole map N=3. **16 of 16 `REJECTED reject_geometry` after both stages, 0 poses published** (32 stage decisions). End of attempt 42.4-47.6 s. Wilson 95 % upper bound for 0/16: 19 %. Baseline `s3` wrong map: 1 of 1 rejected.

Totals for `s2s3` today: 50 accepted poses scored against the reference (25 at 3 s, 5 at 5 s, 20 at 1 s), **0 wrong accepts**, plus 4 accepted poses in the fault runs, all correct. These are repeats of about 24 windows in about a dozen places; the Wilson bound for 0/50 (7 %) assumes independence that these data do not have.

## 5. Watchdogs and faults (V, eco01 c04, `mode:=s2s3`; `table-faults.csv`)

Nominal with every watchdog active (`g00`, twice): `LOCALIZED` from S2 at 24.3 s. Positives and negatives of sections 3-4 all ran with the watchdogs on: no false trigger in 78 runs.

| Case | Outcome (decision time from first input) | Pose published |
|---|---|---|
| LiDAR absent / IMU absent | `TIMEOUT sensors_unavailable` 60.0 s | no |
| 30 % LiDAR loss / IMU stamps +1000 s | `ERROR lio_not_ready_5s` 7.3 s | no |
| 30 % IMU loss | `TIMEOUT imu_not_stationary` 60.0 s | no |
| FAST-LIO2 exits at start / hangs at start | `ERROR fastlio_exited_1` 2.3 s / `lio_not_ready_5s` 7.3 s | no |
| FAST-LIO2 killed during S2 search | `ERROR fastlio_exited_-9` 10.5 s, search worker killed | no |
| FAST-LIO2 frozen (SIGSTOP) at the end of S2 accumulation / during S2 search | `ERROR lio_stalled` 9.0 s / 13.4 s | no |
| FAST-LIO2 killed during S3 accumulation | `ERROR fastlio_exited_-9` 16.5 s | no |
| FAST-LIO2 killed after the pose was handed over | before the fix: `HANDOFF_FAILED amcl_not_confirmed` at 49.2 s; after: `HANDOFF_FAILED odometry_lost_after_pose` at 20.9 s | yes: the accepted pose (0.033 m), published before the fault |
| small_gicp binary returns failure | S2 and S3 both `reject_no_hypothesis` -> `REJECTED` 30.4 s | no |
| small_gicp binary missing | `ERROR search_worker_exit_1` 11.1 s | no |
| registration never returns | worker stops at its deadline -> `TIMEOUT search_deadline` 60.0 s | no |
| deadline 15 s (injected) | `TIMEOUT s2_search_deadline` 15.0 s, worker killed | no |
| deadline 35 s (injected), S2 insufficient | `TIMEOUT s3_budget_insufficient` 15.6 s | no |
| AMCL absent | `HANDOFF_FAILED amcl_not_available` 49.1 s | no (no subscriber) |
| node SIGKILL at 10 s, new node | 0 FAST-LIO2 left; new node, clean state, joined mid-motion: `TIMEOUT imu_not_stationary` | no |
| moving for 60 s, then second attempt | attempt 1 `TIMEOUT imu_not_stationary`; attempt 2 (new clock, operational recovery) `LOCALIZED` at 24.5 s, error 0.036 m, 0 stale messages | attempt 2 only |

In all 30 fault rows: no pose without `ACCEPTED`; after shutdown 0 `fastlio_mapping`, 0 search workers, 0 `register_v2` left (`orphans.txt`).

Unresolved (kept, not hidden): **FAST-LIO2 start-up hang**, `hospital-c03-s2s3x3-own-sb`. Its log stops after `Multi thread started` (no `Node init finished`), 0 % CPU, RSS flat at 33 MB, and after SIGINT + SIGKILL the process took 17 s to go away. The same window ran normally 40 minutes later. 1 event in about 100 FAST-LIO2 starts today; note 15 had a different one (2 odometry messages, then silence). Cause unknown: this is our replay setup in Docker, not evidence about FAST-LIO2. The node ended `ERROR lio_not_ready_5s`, no pose. Side effect: `stop_lio()` blocks the node while it waits (terminal state already reached). A restart of FAST-LIO2 inside the same attempt would stretch the protocol's 5 s readiness limit, so it was not implemented; the operational answer today is the second attempt.
Not tested: reversed-order stamps, FAST-LIO2 divergence (alive but wrong), AMCL losing the pose after `LOCALIZED` (the node does not supervise AMCL afterwards), symmetric places.

## 6. Latency and the 60 s (V unless marked)

Which clock is bound: protocol 02 line 93, "60.0 s monotonic end-to-end wall time from first permitted input", acceptance before the deadline (line 129). Frozen, not redefined. All times below are wall time (`time.monotonic`), not CPU time.

| Clock | Definition | Measured |
|---|---|---|
| A cold boot | power button -> stack ready | **not measured.** Only component known: `systemd-analyze` 17.1 s to graphical target (note 15), which is not a cold-boot measurement. Needs a supervised power cycle (note 18, H1.13) |
| B stack start | `docker run` -> relocalizer node spinning / AMCL active | 3.7 / 4.0 / 8.4 s and 3.3 / 3.7 / 8.1 s (med / p95 / max, N=53, host timestamp resolution 1 s; warm image cache, no driver start) |
| C protocol | first sensor message -> `ACCEPTED` | table below |
| D handoff | `/initialpose` published -> AMCL confirmed (3 updates within 0.30 m / 8 deg of the propagated pose over >= 5 s) | 5.3 / 6.4 / 11.0 s (N=56); 5 s of it is the hold time by definition |

| Stage (s; med / p95 / max) | S2 3 s accepted (N=20) | S3 after S2 (N=21) |
|---|---|---|
| IMU gate (waits for 2 s of stillness) | 2.3 / 7.4 / 8.3 | 2.8 / 7.4 / 8.3 |
| FAST-LIO2 ready | 0.6 / 0.7 / 0.7 (all runs) | same |
| S2 exposure | 2.9 (3 s setting) | same, then waits |
| S2 submap build | 0.6 / 0.9 / 0.9 | same |
| S2 search worker (FFT + small_gicp + verification + confirmation) | 9.8 / 12.6 / 12.7 | 9.1 / 11.3 / 11.5 |
| of which FFT / small_gicp (accepted stage) | 5.8 / 8.0 / 8.1 and 3.2 / 3.7 / 3.7 | 6.6 / 9.2 / 9.5 and 3.7 / 4.6 / 4.6 |
| S2 decision at | 16.0 / 21.5 / 22.1 (all 69 S2 decisions) | same |
| S3 acquisition ends at (motion-dependent; 30 = limit) | - | 26.3 / 30.0 / 30.0 |
| S3 submap build | - | 4.1 / 5.5 / 5.6 |
| S3 search worker | - | 11.2 / 14.8 / 15.1 |
| **`ACCEPTED` at (clock C)** | **16.6 / 21.4 / 22.1** | **42.0 / 47.4 / 47.6** |
| `LOCALIZED` at | 21.9 / 26.7 / 27.2 | 48.0 / 53.4 / 56.7 |

60 s compliance on these data: every acceptance came before 60 s (worst 47.6 s). Attempts that could not make it ended without a pose: `TIMEOUT` at 38.0-40.5 s (S3 not viable or not granted) or at 60.0 s (sensors, IMU never still, registration hung). The wait for stillness is inside the 60 s: the gate took up to 8.3 s on accepted runs and 39.7 s on hospital c02. `LOCALIZED` is not bound by the protocol deadline (acceptance is) but also stayed under 60 s (worst 56.7 s).
A + B + C as one requirement: about 17 s (host, not a cold-boot measurement) + 4 s + 17-22 s for S2 gives roughly 40-45 s for a still robot (**I**, never measured as a whole), and over 60 s whenever S3 is needed.

## 7. Resources and message integrity

Whole stack without the bag player (FAST-LIO2, node, search worker, registration, scan conversion, map server, AMCL, lifecycle manager), 78 runs, sampled at 2 Hz from `/proc` (`proc_cost.py`; short peaks can be missed):

| Quantity | med / p95 / max |
|---|---|
| CPU, % of one core, mean over the run | 81 / 104 / 108 |
| CPU p95 within a run / max | 223 / 238 / 242 and 233 / 249 / 257 (search bursts; 8 cores available) |
| PSS peak sum / RSS peak sum (MB) | 319 / 396 / 430 and 465 / 567 / 601 |
| System RAM rise (tegrastats, includes page cache) | 421 / 518 / 544 MB |
| GPU (GR3D) | 0 % in all runs, no CUDA process |
| CPU temperature max / board power peak | 63.8 / 64.7 / 64.9 C and 12.4 / 12.5 / 12.5 W |

Per process (CPU mean % / PSS peak MB, medians; N=72): relocalizer node 32 / 172 (max PSS 315: it holds up to 30 s of clouds as float64), search worker 99 while alive / 61, FAST-LIO2 13.5 / 37, AMCL 6.7 / 16, `register_v2` 26 MB, scan conversion 1.8 / 15, map server 1.2 / 14, lifecycle 1.4 / 8.5. Worker `maxrss` 86 / 93 / 93 MB.
Against note 15 (S3 only: CPU mean 69 %, PSS 339 MB): `s2s3` runs two searches when S2 fails, hence the higher mean CPU; memory is the same.

**IMU "loss" (correction of note 15).** Read from the bags without ROS (`bag_stream_integrity.py`, bags opened read-only):

| Bag | IMU messages / span | Mean rate | Median step | Header-step histogram |
|---|---|---|---|---|
| eco01 | 104507 / 522.59 s | 199.98 Hz | 4.79 ms | two clusters, about 4.25 ms and 5.75 ms |
| eco03 | 54101 / 270.52 s | 199.98 Hz | 4.80 ms | same |
| hospital_02 | 126851 / 634.28 s | 199.99 Hz | 4.81 ms | same |

The stream is complete at the sensor's 200 Hz. The stamps alternate between a short and a long step, so the median step (4.79 ms) is shorter than the mean (5.00 ms), and the gap counter used by the node and by the witness subscriber (`round(step / median) - 1`) counted every long step as a lost sample: 6.4-7.9 % on the bag itself, 6.9 % (median) in the runs. Per stage:
1. Original bag: complete (mean rate 199.98-199.99 Hz; no duplicate, no backward stamp).
2-5. Replay, publication, DDS, subscription: the node received 200.0 messages per wall second in all 78 runs (p95 200.0), and per window it received as many IMU messages as the bag holds (6 windows checked).
6. Inside the node: every received sample reaches the gate; IMU callback max 1.7 ms in the run inspected.
So there is no IMU loss at any stage in replay. Whether the live driver loses samples was never measured (note 18, H1.2).

Consequence worth a decision (**I**): the IMU gate requires ">= 95 % of `ceil(2 s / median cadence)`" samples (amendment 06). With the median at 4.79 ms that is 398 of 400 real samples: 99.3 % of the true rate, not 95 %. A real loss of 3 samples in 2 s would hold the gate closed. It did not bite here (gate at 2.0 s on 13 of 23 stationary starts). Proposal, not applied: compute the expected count from the mean rate or the specified 200 Hz.

**LiDAR.** Here the loss is real and is in the recording: steps of exactly 200 ms. Missing frames by header gap: eco01 115 of 5225 (2.2 %), eco03 2.8 %, hospital_02 2.4 %. Mean rate 9.72-9.78 Hz. In replay the node saw the same frames as the bag in 3 of 6 windows and 1, 2 and 4 fewer in the others (<= 0.9 % added by replay / DDS / subscription; the window edges are approximate). Where the 2-3 % was lost at recording time (driver, DDS or recorder) cannot be told from the bag. `/Odometry` and body clouds: median 8 and 4 gaps per run, consistent with FAST-LIO2 skipping the scans around the missing frames.

## 8. Band gate of note 14: diagnostic only

`band_diag_batch.sh` ran `support_breakdown.py` after the fact on the frozen outputs of every stage of today's runs. It never entered a decision and no threshold was chosen.
| Stage decisions | N | All-point support (min / med / max) | Band support 0.10-1.80 m (min / med / max) |
|---|---|---|---|
| accepted, correct map | 52 | 0.977 / 0.996 / 1.0 | 0.985 / 0.999 / 1.0 |
| ambiguous, correct map | 23 | 0.989 / 0.999 / 1.0 | 0.987 / 0.999 / 1.0 |
| rejected, wrong map or hole | 29 | 0.249 / 0.468 / 0.61 | 0.222 / 0.327 / 0.615 |
Same picture as note 14 on the same development windows. Today's negatives were all stopped by the existing support gate (max 0.61 < 0.70), so these runs say nothing new about whether the band gate is needed.

## 9. Mistakes and invalid items (kept)

- Note 15 "S2 N=1 effective": the window was mostly in motion. Corrected in section 3.
- Note 15 "7-8 % message loss of the replay setup": artifact of the loss estimator. Corrected in section 7.
- I first read `stationary_s` of the certified windows as "still for that long after the start". It is the length of the episode that ends 2 s after the start. Found when the own-odometry radius read 2.8 m on a "stationary" run (`eco01-c03-s2s3x3-own-k1`, kept, tagged smoke).
- `g20` tag `ga`: I sent the grant 0.7 s before the request existed, so it tested "early grant ignored", not "granted". Rerun as `gb`.
- `g10` was meant to freeze FAST-LIO2 mid-accumulation; it landed at the S2 cut. Labelled as it happened.
- Code changed during the day (mount extrinsic parameter, handoff odometry check, `rate_hz_stamp`). Batches `sa sb sn sr` ran before those; `ga` after the first, `gd` after the second. The final tree was re-run on 6 cases (tag `fin`: 3 positives, 2 negatives, 1 baseline): same outcomes, same errors.
- hospital z09 was skipped (bag ends 74 s after the start, runner needs 75 s).

## 10. Limits

Development data, own-bag or same-day maps, sensor facing up, mount and covariance provisional (note 17). Five stationary blocks. Replay of recorded motion: no supervisor, no reaction time, no live driver. Cold boot not measured. Nothing measured on the Nano. Next: notes 17 and 18.
