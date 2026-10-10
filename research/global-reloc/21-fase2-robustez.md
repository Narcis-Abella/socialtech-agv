# 21 — Phase 2: functional robustness (O1-O4, fault matrix, gate against the base driver, 200 restarts)

Date: 2026-10-10. Branch `feat/global-reloc`, local commits only. Git at start `41f58c8` (F1 candidate `GR-S2S3-F1` = `c026a7b`, untouched).
**Verdict: PARTIAL.** One critical defect is open (section 4b): when FAST-LIO2 diverges during the S2 search, a wrong `/initialpose` can go out (2 of 3 runs of case h27). No navigation permit was given in any of them.
Candidate of this phase: **`GR-S2S3-F2`** = method commit `98926f7`, harness = branch head. Identity: `results/21-f2/candidate-identity.txt`.
Labels: **V** measured or read in code, **I** inferred, **NV** not verified.
Level of every result: `VALIDATED_IN_REPLAY`, development data, Orin NX at 1x, `jetson_clocks` on. **Nothing ran on hardware. No robot moved. A replay is not a sensor, a software restart is not a cold boot, a virtual CAN bus is not a base.** `LOCALIZED` is the node's own state, not an external accuracy statement; accuracy numbers come from the offline evaluator, which is the only code that reads the reference.

## 1. What changed in the method

The search, `verify.decide` (acceptance gate), the submap builder and the 60 s clock are byte-identical to F1 (`git diff c026a7b 98926f7 -- search_v2.py verify.py correlative_search.py lio_submap.py` is empty). Changes are in `reloc_core.py`, `reloc_node.py` and one new consumer, `nav_gate.py`.

| Finding (note 20) | Change | Constants (none is an acceptance threshold) |
|---|---|---|
| O1 no sensors -> no terminal state | `WAIT_SENSORS` without any IMU or LiDAR message for 30 s -> `TIMEOUT(no_sensor_input)`. Clock B (from node start); the 60 s protocol clock still starts at the first sensor message. Watchdog timers run on the steady clock: under `use_sim_time` rclpy timers follow `/clock` and stop with it. | `SENSOR_WAIT_S=30` |
| O2 what navigation may read | `/global_reloc/nav_permit` (Bool, 10 Hz, `true` only in `LOCALIZED`). `nav_gate.py` forwards `/cmd_vel` to `/cmd_vel_gated` only after 3 consecutive fresh `true` from exactly one publisher whose node name is `global_reloc`; closed on anything else. Closed = each incoming command replaced by zero, plus 1 s of zeros at the closing edge. | TTL 0.5 s, need 3 |
| O3 nothing watched after `LOCALIZED` | `LOCALIZED` is revocable and final: `LOCALIZATION_LOST(reason)`, no automatic way back. Reasons: `supervisor_stalled` (own tick unserved > 2 s), `fastlio_exited_*`, `odometry_stalled` (3 s), `odometry_implausible` (3 consecutive steps > 3 m/s), `amcl_silent` (no `map->odom` for 3 s; AMCL republishes it on every scan), `amcl_incoherent` (AMCL vs own propagated pose > 0.5 m or 15 deg held 3 s), `input_stamp_*`. Also `HANDOFF_FAILED(odometry_implausible)` and `ERROR(lio_diverged)` before. | see `reloc_core.py:28-40` |
| O4 unbounded buffers | Clouds, IMU and build released at the end of the attempt; odometry trimmed to 1200-2400 rows; monitor trace capped at 720. | `ODOM_KEEP=1200` |
| new, found here (h15) | `InputIntegrity`: from `LIO_STARTING` on, a sensor stamp that does not increase, or advances > 1 s beyond wall time, ends the attempt (`ERROR`, `HANDOFF_FAILED` during hand-over, `LOCALIZATION_LOST` after). | `INPUT_JUMP_S=1` |

Invariants kept (V, code + every run below): one `/initialpose` publication, only from `HANDOFF` after `ACCEPTED`; `truth_used: false`; one 60 s clock; the method container refuses to start if a reference file is visible.

### Decisions with alternatives

| Decision | Alternatives considered | Why this one |
|---|---|---|
| Gate as a separate topic-level node in front of the driver | check inside `tracer_base`; SROS2 access control; lifecycle-manage Nav2 | No third-party patch (repo rule), testable now. Weakness: it only guards the topic it sits on (section 6). The in-driver check or SROS2 is the stronger design and is a decision for Narcís. |
| Permit = heartbeat with TTL, not a latched state | latched `LOCALIZED` topic; service call | A dead or hung supervisor must close the gate by silence. Measured: closed 0.31-0.41 s after the last heartbeat. |
| Gate requires one publisher named `global_reloc` | accept any single publisher | Pilot run p2 showed a lone foreign publisher of `true` opening the gate for 0.7 s after the node died. Node names can be forged: real protection needs SROS2. |
| Coherence monitor compares AMCL with the pose propagated from the accepted one (absolute) | sliding re-anchor on AMCL | A re-anchor follows a slowly diverging AMCL. Cost: odometry drift will revoke eventually; 496 s / 177 m gave max 0.13 m, 2.6 deg against limits 0.5 m / 15 deg (V, one bag). |
| `LOCALIZATION_LOST` final, no automatic re-localization | restart the attempt | Re-localization while possibly moving is S3, which needs supervision. Out of scope for F2. |
| Zeros on block + 1 s burst, then silence | flood zeros continuously | A continuous flood would mask a dead upstream from the base's own timeout. |
| Refuse non-monotonic input at the first event | tolerate N events; drop the bad message | The node is not between the sensor and FAST-LIO2, so it cannot filter. 0 such events in 256 earlier runs without injection, so no measured availability cost. |

## 2. Harness corrections (own mistakes, all kept in `results/21-f2/invalid-or-superseded/`)

1. **Fault relay published best-effort; FAST-LIO2 subscribes reliable** (`laserMapping.cpp:916,922`), so in every relay case FAST-LIO2 received nothing and the run ended `ERROR(lio_not_ready_5s)` whatever the fault. Inherited from F1. The relay now publishes reliable (depth 2000), counts received / forwarded / altered and the matched subscribers, and writes them per run. Control `h40` (relay, no fault): 14995/14995 IMU and all LiDAR forwarded, 2 and 3 subscribers, `LOCALIZED` at 24.3 s as `h00`.
2. Matrix `f2a` (first run): 15 relay rows invalid for that reason (6 `FAIL` + 9 false `PASS`). `h35` failed because the teardown resumed the frozen node. Together the 7 non-conforming rows of `f2a`: `h10, h11, h19, h20, h28, h29` (relay artefact) and `h35` (state read after teardown). None was a method defect.
3. `f2_check.py` now takes the state at the harness stop time and reports what happens afterwards in `after_teardown`. Expectations were not changed.
4. Each case names a no-fault control and an evidence expression (anomaly injected, delivered, observed). False -> `INVALID`, never `PASS`.
5. Evidence `eval` scope bug: control `h40-v1` scored `INVALID` (`table-v1.csv`), re-scored `PASS` (`table-v1b.csv`).
6. Pilot p1: nominal cases ended `odometry_stalled` because the bag ended; the harness now stops the node while the bag still plays.
7. Matrix `f2c`: `h12-h15` `INVALID` because their evidence required FAST-LIO2 odometry, which no longer exists when the attempt aborts at 2.6-2.9 s. Evidence restated before the rerun `f2d` (relay altered the stream, node saw it, subscribers matched).
8. Storage stalls (section 8): `f2_check.py` invalidates a run whose cost sampler has a hole > 3 s; `io_stall_audit.py` lists runs overlapping a kernel NVMe timeout.
9. Checker let an unscored published pose pass (`h27`, two matrix runs). Found by reading the pose by hand; fixed (`C?`).

## 3. Non-conforming cases of the second matrix (`f2b`, candidate `17172ab`), audited one by one

| Case | Observed | Cause (V) | Action |
|---|---|---|---|
| `h15` LiDAR frames swapped 20 % | `HANDOFF_FAILED(amcl_not_confirmed)`, class **wrong_accept**: `/initialpose` 0.08 m, **13.97 deg** off | FAST-LIO2 kept publishing on reordered input (odometry 392 msgs, 64 non-monotonic, 2.15 m/s peak); the pose was propagated through it. Gate never opened (0 permits) | **Real defect.** Fixed by `InputIntegrity` (`98926f7`), unit test added. Rerun: `ERROR(input_stamp_backward_lidar)` at 2.9 s, no pose |
| `h20` IMU stamps +1000 s after `LOCALIZED` | stayed `LOCALIZED`; expected revocation | The pose stayed correct against the reference (AMCL max 0.063 m, 100 % within limits): FAST-LIO2 tolerated it. The expectation was a prediction, not kept artificially | Expectation unchanged. The same integrity rule detects the jump: rerun `LOCALIZATION_LOST(input_stamp_jump_imu)` 0.6 s after the fault |
| `h36` node frozen 5 s then resumed | stayed `LOCALIZED`; expected `supervisor_stalled` | Run invalid: the board's NVMe stalled (kernel `nvme0: I/O tag 684 timeout, completion polled` 17:42:50). Cost sampler hole 24.7 s, FAST-LIO2 odometry gap 22.5 s, node blocked in a file write after `SIGCONT`. Gate closed by stale permit 0.31 s after the freeze | Rerun without stall: `LOCALIZATION_LOST(supervisor_stalled)` 5.01 s after the freeze |

## 4. Final fault matrix (candidate `98926f7`): 46 cases, all valid; 43 conform, 1 unsafe (h27), 2 wrong predictions with correct behaviour (h44, h45)

Tables `results/21-f2/table-f2c.csv` (41 rows: 37 `PASS`, 4 `INVALID` superseded by) `table-f2d.csv` (7 rows `PASS`: `h12-h15` + new `h41-h43`). eco01 window c04, mode `s2s3`. Invariants checked on every row: P1 pose only after `ACCEPTED`; G1 gated output non-zero only while permitted; G2 gate closed within 0.7 s of revocation; T1 terminal state; O1 no orphan process; C1 evaluator class not `wrong_accept` / `pose_without_acceptance`. As run, no row failed an invariant; **that was wrong for `h27`**: its published pose was classed `unscorable_reference_gap` (reference bracket 0.2003 s against a 0.20 s limit) and the checker let it through. Read by hand it is a wrong acceptance (section 4b). The checker now fails an unscored published pose (`C?`). `table-f2e.csv` holds the reruns of h27 and the new h44, h45.

| Group | Case | Expected (written before) | Observed | Detection after fault |
|---|---|---|---|---|
| control | h00 nominal, h40 relay pass-through | `LOCALIZED` held | `LOCALIZED` held 45 s, monitor max 0.11 / 0.18 m | - |
| sensors | h01 nothing, h02 `/clock` only | `TIMEOUT(no_sensor_input)` | same | 30 s |
| | h03 input stops in IMU gate | `TIMEOUT(imu_not_stationary)` | same | 60 s deadline |
| | h04 stops in S2; h06 10 s gap in S2 | `ERROR(lio_stalled)` | same | 3 s |
| | h05 2 s gap in S2; h09 2 s LiDAR blackout | any safe terminal | `LOCALIZED`, correct (0.03 m) | - |
| | h07 stops / h08 10 s gap after `LOCALIZED` | `LOST(odometry_stalled)` | same | 2.88 / 2.86 s |
| | h10 IMU lost, h11 LiDAR lost after `LOCALIZED` | `LOST` | `LOST(odometry_stalled)` | ~3 s |
| stamps | h12 IMU dup 20 %, h13 IMU swapped 5 %, h14 LiDAR dup 20 %, h15 LiDAR swapped 20 % (from start) | any safe terminal | `ERROR(input_stamp_backward_*)`, no pose | 2.6-2.9 s |
| | h41 LiDAR swapped from S2 search | `ERROR/HANDOFF_FAILED(input_stamp_backward_lidar)` | `ERROR(...)`, no pose | first event |
| | h42 LiDAR / h43 IMU swapped after `LOCALIZED` | `LOST(input_stamp_backward_*)` | same | first event |
| | h16 IMU, h17 both: epoch jump in S2 | `ERROR/TIMEOUT/REJECTED` | `ERROR(input_stamp_backward_imu)` | first event |
| | h18 both epoch jump in IMU gate | any safe terminal | `TIMEOUT(imu_not_stationary)` (F1 rule, unchanged) | 60 s |
| | h19 both epoch jump, h20 IMU +1000 s after `LOCALIZED` | `LOST` | `LOST(input_stamp_backward_imu / jump_imu)` | 0.6 s |
| `/clock` | h21 back, h22 forward 1000 s during S2 search | any safe terminal | `LOCALIZED`, correct | - |
| | h23 back, h24 forward after `LOCALIZED` | `LOCALIZED/*` | held `LOCALIZED` 53 s (monitor max 0.17 m) | - |
| FAST-LIO2 | h25 killed after `LOCALIZED` | `LOST(fastlio_exited_*)` | `LOST(fastlio_exited_-9)` | < 0.1 s |
| | h26 frozen after `LOCALIZED` | `LOST(odometry_stalled)` | same | 2.9 s |
| | **h27** gyro bias 0.5 rad/s from 4 s (diverges) | not `LOCALIZED` | **UNSAFE**: `HANDOFF_FAILED(odometry_implausible)` after a **wrong pose was published** (section 4b); gate never open. Third run: `ERROR(lio_diverged)`, no pose | in hand-over, after publication |
| | h44 / h45 gyro bias 0.1 / 0.2 rad/s from 4 s | not `LOCALIZED` (prediction) | `LOCALIZED`, held 53 s, **correct**: pose 0.04 / 0.05 m, AMCL 100 % within limits (max 0.18 m, 3.9 deg). FAST-LIO2 absorbed the bias. Prediction wrong, expectation not edited, rows stay `FAIL EXPECT` | - |
| | h28 gyro bias after `LOCALIZED` | `LOST` | `LOST(amcl_incoherent)`, monitor 1.15 m | **9.0 s** |
| | h29 accel x3 after `LOCALIZED` | `LOST` | `LOST(odometry_implausible)`, 6.2 m/s | 0.7 s |
| AMCL | h30 killed, h31 frozen after `LOCALIZED` | `LOST(amcl_silent)` | same | 2.88 s |
| | h32 foreign `/initialpose` moves AMCL | `LOST(amcl_incoherent)` | same (15.7 m) | 4.32 s |
| | h33 killed during hand-over | `HANDOFF_FAILED(amcl_not_confirmed)` | same; pose published (correct), gate never open | 30 s |
| supervisor / gate | h34 node `SIGKILL`, h35 node frozen | gate closes; state file stays `LOCALIZED` | gate closed 0.41 s after the last heartbeat | 0.41 s |
| | h36 frozen 5 s, resumes | `LOST(supervisor_stalled)` | same; gate closed 0.41 s after freeze, never reopened | 5.01 s |
| | h37 gate killed while open | no output | output stops with the gate | - |
| | h38 second permit publisher from start | gate never opens | 0 non-zero commands (`publishers_-2`) | - |
| | h39 foreign `true` after node death | gate stays closed | closed (last non-zero 4.0 s before) | - |

Notes: (a) 46 cases on one window are 44 different faults, not 44 independent samples of each; no rate is claimed. (b) `h28` is the weak point: a slow FAST-LIO2 divergence is seen only through AMCL disagreeing, and AMCL takes its motion from the same odometry; 9 s with the gate open is measured, a slower drift is unbounded (NV). (c) After `HANDOFF_FAILED` AMCL keeps the published pose (h27, h33): the gate stays closed, but anything reading `/amcl_pose` directly is not protected (finding O2b of note 20, still open by design).

## 4b. Open critical defect: a pose propagated through diverging odometry (h27)

Fault: gyro bias step of 0.5 rad/s on the IMU from 4 s after the bag starts (S2 exposure runs 2.9-6.1 s, the search until ~19.5 s). Three runs on this window:

| Run | Method tree | State | `/initialpose` (x, y, yaw) | Nominal at the same instant (+-0.5 s) | Permit / gated output |
|---|---|---|---|---|---|
| `h27-f2b` | `17172ab` | `HANDOFF_FAILED(odometry_implausible)` | -13.36, -2.57, -1.58 | -13.62, -6.60, -2.86 | 0 / 0 |
| `h27-f2c` | `98926f7` | `HANDOFF_FAILED(odometry_implausible)` | -13.20, -3.26, -1.68 | same | 0 / 0 |
| `h27-f2e` | `98926f7` | `ERROR(lio_diverged)` | none | - | 0 / 0 |

- Published pose ~3.4-4.0 m and ~70 deg from the nominal one (the nominal run is correct to 0.03 m); AMCL then sat 7.5-9.2 m (median) from the reference. The evaluator could not score the pose itself (reference bracket 0.2003 s > 0.20 s), which is how it slipped through two matrix runs.
- Mechanism (V from the odometry log): the submap and the accepted registration are from before the divergence; the pose is then carried ~13 s through FAST-LIO2 odometry to the publication stamp. The odometry drifted at 0.4-1.8 m/s, under the 3 m/s plausibility bound, until after the publication; the bound tripped 0.4-20 s later.
- Bounded how: no permit, gate closed throughout, state ends `HANDOFF_FAILED`. Not bounded: AMCL holds a wrong pose; anything reading `/amcl_pose` or the TF directly sees it. With a slower divergence the same path could end in `LOCALIZED`, because the hand-over monitor compares AMCL with the node's own propagated pose (I, not reproduced: 0.1 and 0.2 rad/s were absorbed by FAST-LIO2 and stayed correct).
- F1 has the same path (the propagation is F1 code); F1 never tested it.
- Not fixed here. Lowering the speed bound until this bag passes would be tuning on the test. Options, a design decision:

| Option | For | Against |
|---|---|---|
| A. Re-verify at publication: score a fresh short submap against the map at the propagated pose with the frozen gate | independent of the odometry; closes the path for S2 and S3 | new step in the 60 s budget (~1-2 s), needs the whole benchmark rerun for false rejects |
| B. S2 only when the robot is still stationary at publication (odometry displacement since the submap under a bound) | simple; matches the real boot (the gate keeps the robot still until `LOCALIZED`) | most development windows move during the search: invalidates them as S2 positives, changes the protocol |
| C. Publish with the submap stamp and no propagation, let AMCL integrate | no odometry in the path | AMCL integrates the same odometry; only moves the problem |

Recommendation: A, measured on the full F1 set before it is accepted.

## 5. Impact on F1 evidence and regressions

F1 commit, notes and NX roots are untouched. The relay cases were rerun on the **frozen F1 method code** (`git archive c026a7b`, hashes of `reloc_node.py` / `reloc_core.py` equal to the F1 identity; only `fault_relay.py` replaced) and on F2.

| F1 claim | Published in F1 | Frozen F1 code + corrected relay | F2 | Verdict |
|---|---|---|---|---|
| note 16 `g03` LiDAR -30 % | `ERROR(lio_not_ready_5s)` | `LOCALIZED` 25.9 s | `LOCALIZED` 24.3 s | **F1 row was an artefact.** The method tolerates 30 % LiDAR loss in this window (1 run) |
| note 16 `g05` IMU stamps +1000 s | `ERROR(lio_not_ready_5s)` | same, 2095/2095 IMU delivered and shifted, 0 odometry | same | F1 state confirmed, now with delivery evidence |
| note 16 `g04` IMU -30 % | `TIMEOUT(imu_not_stationary)` | same | same | valid then and now (node-side IMU gate, before FAST-LIO2) |
| note 15 `f03` LiDAR -30 % (mode s3) | `ERROR(lio_not_ready_5s)` | `LOCALIZED` | `LOCALIZED` | **artefact**, as `g03` |
| note 15 `f05` IMU +1000 s (mode s3) | `ERROR(lio_not_ready_5s)` | same | same | confirmed |

No other F1 result used the relay. Notes 15 and 16 are not edited; this table is their erratum.

Regressions on F2 (`batch-g2c.log`, `batch-r2c.log`):
- 25 fault cases of note 16: same state and reason as F1 in 24; `g03` differs as explained.
- F1 reproduction: eco01 z03 S2 `correct_accept`, accepted at 18.8 s (F1 18.9 s). hospital z00 S3 `LOCALIZED`, accepted at 41.9 s, **unscorable in this repetition** (reference gap at the query stamp, the same gap that made the first F1 attempt unscorable); the previous F2 tree scored it `correct_accept` at 41.8 s.
- 14 of 14 negatives (wrong map, map hole) `correct_reject`, no pose. Wilson 95 % upper bound for a wrong acceptance on these: 21.5 %. They are the F1 cases, not new evidence.
- 83 F1 runs re-scored with the stricter F2 evaluator: 0 class changes (`rescore-f1-with-f2-evaluator.csv`).
- Tests in the NX image: `global_reloc_3d` 11 files (27 in `test_reloc_core`, 1 skipped as in F1), `nav_bench` complete, all pass.
- 60 s deadline: worst `attempt_end` without a pose 60.03 s; worst acceptance 43.0 s (200 restarts).

## 6. Gate -> `tracer_base` -> virtual CAN

`gate_can_test.sh` in `socialtech:gate-can` (method image + `iproute2`, `can-utils`), `--cap-add NET_ADMIN`, **no `--network host`**: `vcan0` exists only in the container's private network namespace. The script aborts if `can0` is visible; it was not (`container_links.txt`: `lo`, `eth0`, `vcan0`). Host `can0` stayed `DOWN`, `STOPPED`, RX/TX 0 packets before and after (`host_can0_before/after.txt`, identical). Driver: unmodified `tracer_base` of the image (`tracer_ros2` `2c4938db`), `port_name:=vcan0`, `/cmd_vel` remapped to `/cmd_vel_gated`. Source: constant Twist 0.1 m/s, 0.05 rad/s at 10 Hz. One run (`results/21-f2/gate-can/`).

| Phase | Motion frames `0x111` | Non-zero | Timing |
|---|---|---|---|
| permit `false` 3 s | 30 | 0 | - |
| permit `true` 5 s | 50 | 48 | first non-zero 0.21 s after `true` |
| revoked (`false`) 3 s | 50 | 0 | zero from the first frame |
| `true` 5 s | 51 | 48 | 0.30 s |
| publisher alive but silent 3 s | 50 | 4 | last non-zero **0.39 s** after the last heartbeat |
| `true` 5 s | 50 | 48 | 0.29 s |
| permit publisher exits | 52 | 0 | - |
| **bypass**: direct publish on `/cmd_vel_gated` | 5 | **5** | the gate does not see it |
| **gate killed while open** | 0 after the kill | last frame on the wire **non-zero** (100 mm/s, 50 mrad/s) | no further frame |

CAN IDs seen: `0x111` (502) and `0x421` (1, at driver start). Read in code: `tracer_base` sends one motion frame per Twist, with no check and no timeout of its own.

Residual, by construction of a topic-level gate:
1. Any publisher on the driver's topic bypasses it (measured). Needs the check inside the driver, or SROS2.
2. If the gate (or the whole computer) dies while open, the last command stays the last frame. Stopping then depends on the base firmware's command timeout, **not verified here** (no base on a virtual bus) -> F4-F6.
3. A forged node name `global_reloc` would satisfy the publisher check.
4. No physical E-stop or watchdog was exercised.

## 7. 200 software restarts

`batch_restarts.sh`, cases `cases-restarts.csv` (seed 20261010): 36 stationary windows (eco01 54 runs, eco03 45, hospital 101), start jitter +-0.5 s, pre-roll 2.5-4 s, lead 10/12/14 s, own DDS domain. Each run = fresh container, fresh node, cold FAST-LIO2 process on a warm board: **software restart, not a cold boot**. Runs of one window are near-identical inputs, not independent: the effective sample for scene effects is 36 windows. A pilot of 10 ran on the previous candidate (`table-rs.csv`, 9 `LOCALIZED` + 1 timeout, kept, not counted); all 200 below ran on `98926f7`. No counter was reset, no run removed.

| Quantity (N = 200, 0 invalid) | Value |
|---|---|
| `LOCALIZED`, evaluator `correct_accept` | 188 (94.0 %, Wilson 95 % 89.8-96.5 %) |
| wrong acceptance / pose without acceptance | 0 / 0 (Wilson upper 1.9 %; 36 windows -> do not read as a field rate) |
| FAST-LIO2 start failures (`lio_not_ready`, exit) | 0 of 200 (upper 1.9 %). F1 saw ~1/100; not reproduced, not explained |
| not localized, all safe timeouts, no pose | 12: `s3_budget_insufficient` 10 (hospital z02 5/5, c02 5/6), `imu_not_stationary` 2 (hospital c02, eco03 z02) |
| invariant failures, orphan processes | 0, 0 |
| first input -> accepted | median 17.8 s, p95 34.6 s, max 43.0 s |
| first input -> `LOCALIZED` | median 23.3 s, p95 39.8 s, max 48.5 s |
| `/initialpose` XY error (accepted) | median 0.03 m, p95 0.06 m, max 0.08 m (reference floor unknown) |
| PSS peak, whole stack without the player | median 255 MB, p95 322 MB, max 381 MB |
| board `MemAvailable` | 14171 MB before the first, 14203 MB after the last, min 14166 MB |

The 12 failures are properties of two windows (S2 ambiguous and too little time left for S3; a 2.3 s stationary span against a 2 s IMU rule with jittered start), deterministic per window, not start-up faults.

## 8. Memory and CPU, F1 vs F2; storage stalls

Same window (eco01 c00), 515 s of replay, `LOCALIZED` held until the bag ends (`mem-*.json`, PSS from `smaps_rollup`, span 469 s after the first minute).

| | F1 frozen code | F2 `17172ab` | F2 `98926f7` |
|---|---|---|---|
| node PSS first -> last (MB) | 87.9 -> 90.1 | 80.0 -> 80.9 | 83.1 -> 85.6 |
| node slope (MB/min) | +0.28 | +0.09 | +0.28 |
| FAST-LIO2 PSS first -> last (MB) | 37.8 -> 45.3 (+1.0/min) | 40.8 -> 47.0 (+0.9/min) | 39.5 -> 45.9 (+1.0/min) |
| stack PSS peak (MB) | 256.8 | 253.6 | 254.3 |
| stack CPU mean / p95 (% of one core) | 64.8 / 80.8 | 66.8 / 82.7 | 67.5 / 84.5 |

- The two F2 runs differ from each other as much as F2 from F1: **no memory difference between F1 and F2 is resolved** with one run each. What is shown: node growth <= 2.5 MB in 8 min, odometry buffer bounded (1411 rows at the end, cap 2400), CPU +2-3 points (supervision, permit).
- FAST-LIO2 grows ~1 MB/min while the robot travels (its map). Same in F1. Not bounded by this work; behaviour beyond 8 min **NV** (longest bag 634 s, so the 30 min soak planned in note 19 was not possible in replay).
- Supervision held `LOCALIZED` 496 s over 177 m with no false revocation (max discrepancy 0.13 m, 2.6 deg), on both F2 trees.

**Storage stalls on the NX (platform finding, V):** the kernel logged `nvme nvme0: I/O ... timeout, completion polled` 75 times since 2026-10-05 (40 on 2026-10-10). A request that hits it blocks for up to 30 s. `h36-f2b` was hit (section 3). 19 runs of the final campaign and 13 of note 16 overlap a timeout line; none of them shows a sampler hole and all ended in their expected state (`stalls-f2c.csv`). One F1 run, `hospital-c03-s2s3x3-own-sb` `ERROR(lio_not_ready_5s)`, overlaps two: a stall is a candidate cause of that F1 false reject (hypothesis, NV). Consequence for the method: file writes on the supervisor thread can block it; measured outcome is fail-closed (gate shut by stale permit, then `supervisor_stalled`), at the price of a lost localization.

## 9. Residual risks

| Risk | State | Goes to |
|---|---|---|
| Base keeps the last command if the gate or the computer dies | measured on vcan; firmware timeout NV | F4-F6, Narcís |
| Direct publishers on the driver topic, forged permit node name | measured / read in code | design decision (in-driver check, SROS2) |
| **Wrong `/initialpose` when FAST-LIO2 diverges during the search (section 4b)** | measured 2 of 3, **open, critical** | F2 not closed until fixed and the matrix rerun |
| Slow FAST-LIO2 drift after `LOCALIZED`: detection 9 s in one case, unbounded in general; monitor not independent of the odometry | measured once | F5 (independent data), possibly a second sensor |
| AMCL keeps a published pose after `HANDOFF_FAILED` | by design, gate closed | Nav2 integration |
| NVMe stalls on the production board | measured, cause NV (host, not touched) | Narcís |
| Memory beyond 8 min; FAST-LIO2 map growth | NV | F3 / hardware soak |
| Cold boot, live Livox driver (stamp steps at time sync would now end the attempt), real CAN, E-stop | NV | F3-F6 |
| Reliability: one window for faults, 36 windows for restarts, own maps, sensor facing up, provisional mount | unchanged from F1 | F4-F5 |
| Thresholds of section 1 are provisional engineering values, validated on one window | V as stated | F5 |

## 10. Reproduce

NX roots (raw data, not in git): `/home/jetson/work/reloc-f2c-20261010/` (final candidate: `runs/`, `table-f2c.csv`, `table-f2d.csv`, `table-rs2.csv`, `gate-can/v1/`, `chain3.sh`), `/home/jetson/work/reloc-f2-20261010/` (earlier candidates, invalid and superseded rows, `f1ref/` = frozen F1 code with the corrected relay). F1 roots untouched.

```
R=<root> bash tools/global_reloc_3d/batch_faults_f2.sh <tag> [case-prefix]        # fault matrix, one row per case in table-<tag>.csv
R=<root> bash tools/global_reloc_3d/batch_faults_s2s3.sh <tag>                     # 25 cases of note 16
R=<root> bash tools/global_reloc_3d/batch_restarts.sh cases-restarts.csv <tag> 0 199
python3 tools/global_reloc_3d/restart_summary.py table-<tag>.csv runs
python3 tools/global_reloc_3d/io_stall_audit.py runs
docker run --rm --cap-add NET_ADMIN -v <root>/tools:/tools:ro -v <out>:/out socialtech:gate-can bash /tools/global_reloc_3d/gate_can_test.sh   # needs: sudo modprobe vcan
```
