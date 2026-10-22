# 20 — Phase 1: audit of the online candidate, evidence check, reproduction, freeze

Date: 2026-10-10. Branch `feat/global-reloc`. Git at start `ab28151`, candidate commit `c026a7b` (local, not pushed).
Scope: audit and freeze only. No alternative was researched, no threshold or parameter was changed.
Labels: **V** measured or read in code, **I** inferred, **NV** not verified.
Level of every result here: `VALIDATED_IN_REPLAY`, development data. `LOCALIZED` means AMCL agrees with the node's own propagated pose; it is not an independent accuracy statement.
Master plan and phase status: [19-plan-cierre-y-entrega.md](19-plan-cierre-y-entrega.md). Identity record: `results/19-f1-audit/candidate-identity.txt`. Reproduction table: `results/19-f1-audit/repro-runs.csv`.

## 1. Architecture as audited (V, read in code)

```
/livox/imu, /livox/lidar -> reloc_node (ImuGate) -> spawns FAST-LIO2 (cold, owned child)
  /Odometry + /cloud_registered_body -> Acquisition -> lio_submap.assemble (thread) -> search_v2.py (killable child)
     correlative FFT search -> register_v2 (small_gicp, child of the worker) -> verify.decide (frozen gate)
  ACCEPTED -> HANDOFF: /initialpose (map, base_footprint) -> HandoffMonitor on /amcl_pose -> LOCALIZED
```
State machine `reloc_core.TRANSITIONS`, mode `s2s3`: `WAIT_SENSORS -> IMU_CONDITIONING -> LIO_STARTING -> S2_ACCUMULATING -> S2_SEARCHING -> ACCEPTED -> HANDOFF -> LOCALIZED`; S2 without a pose -> `S2_INSUFFICIENT -> S3_SUPERVISION_REQUIRED -> S3_ACCUMULATING -> S3_SEARCHING`. Terminal: `LOCALIZED, HANDOFF_FAILED, REJECTED, TIMEOUT, ERROR`.

| Requirement | Finding | Evidence |
|---|---|---|
| No pose without acceptance | Holds. One `init_pub.publish` in the file (`reloc_node.py:542`), inside `tick_handoff`, called only in state `HANDOFF` (`:345-352`), with an assert (`:532`). `HANDOFF` is reachable only from `ACCEPTED`, `ACCEPTED` only from a `*_SEARCHING` state (`reloc_core.py:34-42`, test `test_only_search_can_accept`). `T_map_src` is assigned only in `collect_search` after `s['accepted']` and the deadline check (`:508-513`) | code; 0 of 108 original runs and 0 of 4 audit runs with a pose and no `ACCEPTED` |
| Ground truth not used online | Holds in code: no method file opens a reference (`grep alidarState|reference` over `reloc_node, reloc_core, lio_submap, search_v2, verify, correlative_search, register_v2.cpp`: only `truth_used: False` strings). **But the harness did mount it** (finding B2) | code; B2 |
| S2 always first, S3 only when authorized | Holds. `S3_ACCUMULATING` only from `S3_SUPERVISION_REQUIRED` only from `S2_INSUFFICIENT`. Grant accepted only while the request is pending (`:239-241`); `s3_auto_authorize` defaults to false (node and launch) | code, tests `S2S3Test`, runs `g19-g22` of note 16 |
| One clock, no restart between S2 and S3 | Holds. `t_input` set once per attempt (`reloc_core.py:88-90`); 60 s and 30 s limits keep that origin; the search worker gets `remaining - 0.5 s` (`:328`) and is killed on expiry (`:355-357`); an acceptance after the deadline is discarded (`:510`) | code, runs `g15-g17` |
| Watchdogs | FAST-LIO2 exit or 3 s without `/Odometry` -> `ERROR` in every accumulating and searching state (`lio_fault`, `:421-431`); 5 s readiness limit (`:371`); no publication from odometry older than 1 s (`:526`); `odometry_lost_after_pose` (`:349`) | code, 30 fault rows of note 16 |
| Child processes | FAST-LIO2 and the worker run in their own sessions with `PR_SET_PDEATHSIG` (`:69-70`); `finish()` kills the worker and stops FAST-LIO2 unless `LOCALIZED`/`HANDOFF_FAILED`; `shutdown()` on SIGINT/SIGTERM | code; `orphans.txt` = 0 in the 4 audit runs |
| Timestamps / TF | `/initialpose` stamp = header stamp of the latest odometry, pose propagated to that stamp. Node publishes `odom -> camera_init` (static) and `odom -> base_footprint`; never `map -> odom` | code. **NV:** how AMCL treats a stamp that is not "now" on the live driver clock (H1.1, H1.3) |
| Restarts | `max_attempts` default 1. A retry builds a new `Attempt`, gate, buffers; stale messages refused by `min_stamp` | code, run `g24` |

## 2. Bugs

Fixed in `c026a7b`, each demonstrated first:

| # | Bug | Demonstration | Fix | Regression |
|---|---|---|---|---|
| B1 | `ImuGate.add` walked the 0.1 s evaluation grid across any forward stamp jump inside the IMU callback. The single-threaded executor, the watchdogs and the 60 s deadline are not served meanwhile. No pose is published (fails silent, not unsafe) | measured on the laptop: jump 1e3 s -> 0.07 s, 1e4 s -> 0.55 s, 1e5 s -> 5.5 s, 1e6 s -> 49.5 s. A sensor-clock-to-epoch jump (1.7e9 s) would block for about a day (**I**, extrapolated). Run `g05` of note 16 (+1000 s) was too small to show it | skip the grid times whose window is empty in one step (`reloc_core.py`, 3 lines). Ready stamp and evaluation counters on a 5 s gap are identical to the old walk (7.5 s, 20 evaluations, 19 failed) | `test_stamp_jump_does_not_block_the_node`, `test_short_gap_gives_the_same_ready_time_as_before` (both fail on `ab28151`) |
| B2 | `run_online.sh` mounted the whole map directory into the method container. For eco01 and hospital that directory contains `vs/run/alidarState.txt`, the reference. Note 16 says "node container never mounts a reference": false for the mount, true for the code | `docker run -v map_out/eco01_1x:/map:ro ... find /map -name 'alidarState*'` -> `/map/vs/run/alidarState.txt` | mount the YAML and its image only; `online_inner.sh` aborts (exit 3) if any `alidarState*` is visible | the guard runs on every run; the 4 audit runs passed it. Re-run of two note-16 cases with the reference absent gives the same outcomes, `hospital z00` to the last digit (0.0669 m, 0.2715 deg) |

Impact on the note-16 evidence: B1 cannot occur on the bags (no stamp gap over 3 s; the changed branch is not entered). B2: no code path reads the file, and the outcome is unchanged without it. The evidence stands.

Open, not critical, not fixed (no demonstrated wrong pose; changing them is Phase 2 work):

| # | Finding (V read in code unless marked) | Consequence |
|---|---|---|
| O1 | With no IMU and no LiDAR message at all, `t_input` stays `None`, so `expired()` is never true: the node stays in `WAIT_SENSORS` for ever. Consistent with the protocol (clock starts at the first input), but there is no terminal state. Fault runs covered one missing stream, not both | needs an external boot watchdog or a decision on a start-up timeout |
| O2 | After `HANDOFF_FAILED` with the pose already sent, AMCL keeps that pose, FAST-LIO2 stays alive, and the node stops publishing `odom -> base_footprint` (`on_odom` only serves `LIO_STATES`) | whoever enables navigation must gate on `/global_reloc/state == LOCALIZED`, never on AMCL having a pose |
| O3 | The node does not supervise AMCL after `LOCALIZED` (known, note 16) | H1.14 |
| O4 | After `LOCALIZED` the node keeps every odometry sample and the S2/S3 clouds in memory for as long as it lives. Growth not measured (**NV**) | bounded buffers or node exit after handoff; measure in Phase 3 |
| O5 | `stop_lio()` can block the executor 5 s or more (17 s seen once, note 16). Only after a terminal state | none on the decision |
| O6 | S3 uses clouds accumulated before the grant (`S2_SEARCHING ... S3_SUPERVISION_REQUIRED` are accumulating states). In replay the recorded motion precedes the grant; on the robot nobody moves before it | replay S3 timing is not representative of supervised operation (already stated in note 16) |
| O7 | `evaluate.reference_at` refuses a stamp whose bracket is over 0.20 s even when the stamp coincides with a reference sample. With 2.2-2.8 % of LiDAR frames missing in the bags, some accepted poses will be unscorable | seen once here (run `f1`); evaluator left unchanged |
| O8 | `register_v2` has no recorded build command in the repo; images exist only as local image IDs on one board | reproducible on this NX only; Phase 7 |
| O9 | The frozen gate uses all-point support; protocol 02 asks for the 0.10-1.80 m band crop before support. Known deviation (note 14) | decision 4 below |

## 3. Reports against the original results (V)

Recomputed from `results/16-s2s3/table-runs.csv` (78 rows) and `table-faults.csv` (30 rows); 108 run directories exist on the NX, tags `sa sb sn sr fin` = 78, `ga gb gc gd k1` = 30. Five rows compared field by field with the raw `metrics.json` / `evaluation.json` on the NX: identical. Deployed tools on the NX matched `ab28151` by sha256 (17 files).

| Claim in note 16 | Recomputed | Verdict |
|---|---|---|
| 50 accepted poses scored in `s2s3`, 0 wrong accepts | 25 + 5 + 20 = 50 `correct_accept`, 1 unscorable, 0 `wrong_accept` | matches |
| 16 of 16 negatives rejected, 0 poses | 13 cross + 3 hole `correct_reject`, `pose_published` False; plus 1 baseline `s3` | matches |
| Stationary S2 3 s: 10 runs, 8 in S2, 2 in S3 | 8 / 2, all `correct_accept` | matches |
| S2 accept 16.6 / 21.4 / 22.1 s (N=20) | 16.6 / 21.4 / 22.1 with `fin` included (N=22) | matches |
| S3 accept 42.0 / 47.4 / 47.6 s (N=21) | 42.0 / 47.3 / 47.6 (N=22 with `fin`) | matches |
| CPU 81 / 104 / 108 %, PSS 319 / 396 / 430 MB | 81.3 / 103.9 / 107.5, 319.1 / 396.2 / 429.7 | matches |
| "Pose error of the S2 accepts: XY max 0.068 m ... stationary 0.064 m" | S2 accepts: max 0.052 m, 0.51 deg (N=29). 0.068 and 0.064-0.067 m belong to **S3** accepts of the same table rows | **erratum** in note 16, conservative direction. S3 max 0.084 m / 1.2 deg matches |
| "Repeats are identical to the centimetre" (z03 0.008 m twice) | third repeat here: 0.014 m | holds at centimetre level; not bit-identical (the pose stamp differs between runs) |
| "node container never mounts a reference" | false for the mount (B2) | corrected in `c026a7b` |
| Invalid / kept items of section 9 | present in the tables and logs | matches |

Independence, restated: 78 runs are repeats of about 24 windows in about a dozen places, 2 buildings, maps from the same or a same-day session. Five stationary blocks. No rate claim follows from them.

## 4. Reproduction on the NX (V)

Replay 1x, idle board (load 0.00 at start), `jetson_clocks` active, one run at a time, tree = `c026a7b`, image `socialtech:reloc-online`, new root `/home/jetson/work/reloc-f1-audit-20261010/` (original evidence untouched; window lists copied, not moved).

```
rsync -a --exclude __pycache__ tools/ <nx>:/home/jetson/work/reloc-f1-audit-20261010/tools/     # + register_v2 copied from the note-16 root
R=/home/jetson/work/reloc-f1-audit-20261010 bash tools/global_reloc_3d/batch_s2s3.sh cases-f1.csv f1    # then cases-f1b.csv f1b
```
Expected before running: same class as the original run, accept time within the +-2.5 s run-to-run spread of note 15.

| Run | Class | State, stage | Accept at s (original) | XY / yaw error (original) | AMCL after handoff, XY median / max | Pose published |
|---|---|---|---|---|---|---|
| `eco01-z03-s2s3x3-own-f1` (S2) | correct_accept | `LOCALIZED`, s2 | 18.92 (18.87, 18.72) | 0.014 m / 0.003 deg (0.008 m) | 0.050 / 0.057 m | yes, after `ACCEPTED` |
| `hospital-z00-s2s3x3-own-f1` (S3) | **unscorable_reference_gap** | `LOCALIZED`, s3 (S2: `ambiguous_no_alternative_margin`) | 41.88 | not scorable (O7) | 0.033 / 0.044 m | yes |
| `hospital-z00-s2s3x3-own-f1b` (S3, rerun) | correct_accept | `LOCALIZED`, s3 | 41.78 (42.03, 41.78) | 0.067 m / 0.27 deg (0.064, 0.067 m) | 0.033 / 0.039 m | yes |
| `eco01-z03-s2s3x3-cross-f1` (wrong map) | correct_reject | `REJECTED reject_geometry` after S2 and S3, end 42.3 s (42.3) | - | best support 0.44 (S2), 0.38 (S3) against the 0.70 gate | - | **no** |

All four: IMU received at 200.0 Hz, `truth_used: false`, 0 orphan processes, exit 0, reference not visible in the method container. `f1` is an invalid trial for scoring (kept, reason above), not a method failure: its AMCL track stayed within 0.044 m of the reference.
N=1 per case: this confirms that the reported behaviour reproduces on the frozen tree. It adds nothing to reliability.

## 5. Tests (V)

`cd tools/global_reloc_3d && for t in test_*.py; do python3 -B $t; done`, same in `tools/nav_bench`.

| Where | global_reloc_3d (11 files, 44 tests) | nav_bench (11 files) |
|---|---|---|
| Laptop (Python 3.10, numpy 1.21.5, scipy 1.8.0) | 43 passed, 1 skipped, 0 failed | 10 files pass; `test_planar_odom_node.py` fails ("300 of 300 TF missing"): it needs the `planar_odom` node of the robot image |
| NX, inside `socialtech:reloc-online` | 43 passed, 1 skipped, 0 failed | 11 files pass, `test_planar_odom_node.py` included |

Skipped: `test_register_synthetic.py` (needs `REGISTER_BINARY`; not set in either run, so the C++ caller has no unit test executed here; it is exercised by every end-to-end run). `test_reloc_core.py`: 20 tests (18 at `ab28151` + 2 regressions).
Limits: no test drives `reloc_node.py` itself without ROS; its paths are covered by the end-to-end and fault runs only.

## 6. Frozen candidate `GR-S2S3-F1`

Commit `c026a7bd45fc34384c741a1d526ac228a4cf66d3`. Full hashes (code, binaries, images, bags, maps, priors, references): `results/19-f1-audit/candidate-identity.txt`.

| Item | Frozen value |
|---|---|
| Node | `mode:=s2s3`, `s2_exposure_s:=3.0`, `deadline_s:=60`, `acquisition_limit_s:=30`, `max_attempts:=1`, `handoff_timeout_s:=30`, `s3_auto_authorize:=false` on the robot (true only in replay) |
| Constants | LIO ready 5 s, LIO stall 3 s, S3 radius 2.8 m / path 6 m, S3 compute reserve 22 s, handoff odometry age 1 s, handoff check 0.30 m / 8 deg / 5 s / 3 updates |
| IMU gate | `imu_ready.py` unchanged: 2 s window, residual p95 <= 0.006119 rad/s, median norm <= 0.027835 rad/s, raw max <= 0.05 rad/s, acc variance sum <= 0.04 (m/s^2)^2, >= 95 % of `ceil(2 s / median step)` samples |
| FAST-LIO2 | image binary, `mid360.yaml` with `pcd_save_en/map_en/path_en: false`, `gyr_cov: 1.0`, `acc_cov: 1.0` (`reloc_node.start_lio`) |
| Submap | voxel 0.20 m, fit = first 2/3, check = last 1/3, IMU levelling |
| Search | `search_v2.py` defaults: grid 0.5 m, yaw step 2.5 deg, radius 15 m, top 10, GICP voxel 0.3 m, `OMP_NUM_THREADS=2` |
| Acceptance gate | `verify.decide`: GICP converged; support >= 0.70 within 0.15 m; matched RMSE <= 0.10 m; lead >= 0.10 over a distinct runner-up; independent search of the later batch within 0.20 m / 5 deg with the temporal rule |
| Handoff | covariance 0.15 m / 5 deg; `sensor_height` 0.53; `base_from_body_xyyaw` [0, 0, 0] (all PROVISIONAL) |
| AMCL | `nav_bench/config/amcl.yaml` + `overlay.py set_initial_pose=false`; scan `config/scan.yaml` |
| ROS | Jazzy, CycloneDDS, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, random domain 100-199 per run, sim time in replay |
| Runtime | `docker run --network host --cpus 8`, images by ID (identity file) |

Relation to the evidence: notes 15-16 were produced at `ab28151`. `GR-S2S3-F1` differs by B1 (not reachable on those bags) and B2 (harness mounts). Transfer of the note-16 numbers to the candidate is an inference backed by 3 reproduced cases, not a re-measurement of 108 runs.
Any change to a row of this table, to a hashed method file or to an image creates a new candidate name.

## 7. Pending decisions (for Narcís)

1. **Single 60 s deadline.** Proposed and already what the code does: the bound is first sensor message -> `ACCEPTED`, one clock, no restart between S2 and S3. `LOCALIZED` (handoff, 5 s of it is the hold time by definition) and cold boot (power -> first sensor message, never measured) are reported as separate clocks and never added to or hidden inside the 60 s. Reason: protocol 02 fixes the origin at the first permitted input and the event at acceptance; a restart at S3 would let every failed S2 buy 60 more seconds. Cost: a human supervisor who grants after about 38 s gets `TIMEOUT`; that is the benchmark outcome. An operational second attempt is a separate, separately reported attempt, never pooled.
2. **IMU sample count: 95 % written, 99.3-99.5 % effective.** The expected count comes from the median stamp step (4.79-4.81 ms) while the true mean is 5.00 ms, so the gate needs 397-398 of the 400 real samples of a 2 s window. Kept as it is in `GR-S2S3-F1`: it is the rule the evidence was produced with, and it fails closed (`TIMEOUT imu_not_stationary`). Risk: a live driver losing 3 samples in 2 s holds the gate; live loss was never measured (H1.2). Changing the expected count to the 200 Hz rate is a parameter change: decide before Q-eval, as a new candidate.
3. **`base_footprint` convention.** Provisional: `base_footprint` = IMU origin projected to the floor, yaw of the IMU x axis, height 0.53 m, identity mount. Every `metrics.json` carries `PROVISIONAL_not_measured`. A lever-arm error moves `/initialpose` one to one (0.20 m uses two thirds of the 0.30 m limit); a mount-yaw error adds directly to the 8 deg limit; the height shifts the scan band, not the 3D pose. Blocked on a physical measurement (note 17, section C).
4. **Acceptance gate frozen.** `verify.decide` stays the only rule. The band gate of note 14 stays diagnostic: its threshold may not be chosen on scored data, and note 16 gave no case where it was needed. If proposed, it is a second pre-declared configuration evaluated once on Q-eval.

## 8. Risks

- Reliability is unknown: 5 stationary blocks, development data, own or same-day maps. Wilson bounds on these repeats would overstate the evidence.
- Sensor faces up; the final mount looks down. Everything must be re-checked after the remount and a new map.
- FAST-LIO2 start-up hang, about 1 in 100 starts, cause unknown; ends `ERROR`, no pose.
- Live driver never exercised: stamp base, stream loss, cold boot time.
- Reference = Voxel-SLAM of the same bag: errors below a few centimetres are not resolvable.
- Open findings O1-O9.

## 9. Mistakes of this audit (kept)

- First insertion of the regression tests landed after the `return` of a helper function, so they never ran ("18 tests OK"). Caught by the test count; moved into `GateTest` and shown to fail on the old code.
- The first S3 reproduction was unscorable (O7). Rerun once; both runs are in the table.
- AMCL's handling of the `/initialpose` stamp was not re-read upstream in this audit; note 16's citation is taken as is (**NV** here).
