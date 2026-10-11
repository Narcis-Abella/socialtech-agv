# 21 — Phase 2: functional robustness (map check of the handed-over pose, driver cell, fault matrix, 50 restarts)

Date: 2026-10-11. Branch `feat/global-reloc`, local commits only. Git at the start of Phase 2 `41f58c8`; F1 candidate `GR-S2S3-F1` = `c026a7b`, untouched.
**Verdict: FASE 2: VERIFIED — VALIDATED_IN_REPLAY.** No independent physical validation; full AGV safety not demonstrated; F4-F6 pending; restart protocol revised from 200 to 50 for the corrected candidate. One judgement inside the verdict is stated in B13 (criterion F, revocation latency 1.6-2.3 s after an injected fault) so that it can be overruled.**
Candidate: **`GR-S2S3-F2b`** = method `5bb95ef` + `f524d04`, gate `8a88d16`, harness = branch head. Identity and hashes: `results/21-f2/f2h/candidate-identity.txt`. Two trees ran: `f2g` (`5bb95ef`) and the final `f2h` (`f524d04`, permit withheld in doubt, B7); the gate files are identical in both. The earlier candidate `GR-S2S3-F2` (`98926f7`, verdict PARTIAL, case h27 open) and all its evidence are kept as **Part A** below and in `results/21-f2/` (top level); nothing of it was overwritten.
Labels: **V** measured or read in code, **I** inferred, **NV** not verified.
Level of every result: `VALIDATED_IN_REPLAY`, development data, Orin NX at 1x, `jetson_clocks` on. **Nothing ran on hardware. No robot moved. Host `can0` was never used (down, counters 0 before and after). A replay is not a sensor, a software restart is not a cold boot, a virtual CAN bus is not a base.** `LOCALIZED` is the node's own state; accuracy numbers come from the offline evaluator, the only code that reads the reference.

# Part B — final candidate `GR-S2S3-F2b`

## B1. h27: root cause

- Path (V, code): `ACCEPTED` stores `T_map_ci = T_map_src @ T_level_local @ inv(T_ci_first)` (map <- FAST-LIO2 odometry frame, from the registration of a submap built at 2.9-6.1 s). At hand-over the node published `T_map_ci @ T_ci_now`, about 13 s later (search 12.5 s). Everything between the submap and the publication rests on FAST-LIO2 odometry alone.
- Fault of h27: gyro bias step 0.5 rad/s from 4 s. The registration is right (submap mostly from before the bias takes effect); the odometry then drifts at 0.4-1.8 m/s, under the 3 m/s plausibility bound, so `/initialpose` left ~3.5 m / 70 deg wrong (Part A, section 4b).
- Signals that exist online without ground truth: the map, the newest deskewed scans, the odometry itself (not independent), AMCL (starts from our pose: not independent before it converges).
- My evaluator hid it: reference bracket 0.2003 s > 0.20 s -> `unscorable_reference_gap`, counted as not wrong. Now an unscored published pose fails the check (`C?`), and a second score with a 0.5 s bracket is reported next to it, labelled, never replacing the rule.

## B2. h27: solution

**Map check** (`map_check.py`, killable worker like the search): the newest 3 s of deskewed body clouds (>= 6) are placed in the map with the pose under test; only the relative odometry inside those 3 s is used. GICP (same `register_v2`, voxel 0.3) starts from that pose on the first 2/3; the frozen geometric rule of `verify.decide` (support_015 >= 0.70, matched_rmse <= 0.10, unchanged constants) is applied on the last 1/3; the refined pose must not be a different pose (`search_tiles.distinct`, 0.30 m / 8 deg, the constants already used to tell hypotheses apart). No acceptance threshold was changed or added to make h27 pass.

Used three times:
1. **Before `/initialpose`** (`h`): fail -> `HANDOFF_FAILED(map_check_<solver|geometry|shift|timeout>)`, nothing published. The message carries the **refined pose and the stamp of the newest checked scan**; nothing is propagated past a check.
2. **Before `LOCALIZED`** (`c`): after AMCL is coherent with our pose, a second check on fresh scans; fail -> `HANDOFF_FAILED(map_check_*_after_pose)`, no permit.
3. **While `LOCALIZED`** (`s`), every 3 s, on **AMCL's pose** (not ours): a failed check is repeated at once; 2 consecutive failures -> `LOCALIZATION_LOST(map_incoherent)`.

Why not circular: a diverged odometry puts the scans where the map is not, or bends the 3 s window so GICP does not converge; the check asks the map, not the odometry. Uniqueness comes from the global search; the check only asks "still there?", so the alternative-margin rule is not re-evaluated.

| Alternative | Verdict |
|---|---|
| A. Re-verify against the map at publication | chosen, extended to (2) and (3) |
| B. S2 only if still stationary at publication | rejected: stationarity would be read from the same odometry; invalidates most development windows |
| C. Publish the submap-time pose and let AMCL integrate | rejected: AMCL integrates the same odometry |
| Lower the 3 m/s speed bound | rejected: tuning on the test |
| First version of A: check, then propagate to "now" | **failed in test** `d07-t1`: bias from 18 s, check passed at support 0.715, pose published 1 s later 0.55 m / 26 deg wrong. Kept in `table-t1.csv`. Hence "publish only what was checked" |
| Supervise our own propagated pose | rejected: normal odometry drift would revoke; AMCL's pose is what navigation uses |
| Revoke on 1 failed check | not adopted: rmse sits at 0.074-0.094 against the 0.10 limit on good data (B8); one noisy check would end the session |

Cost of the stamp choice (V): nav2_amcl cannot look up the odometry at the message stamp in this stack (`Failed to transform initial pose in time`, also in F1) and applies the pose as if current. With a ~1 s old stamp AMCL starts v x 1 s behind when the platform moves; it converges, later. Stationary at boot (the intended use): no effect expected, **NV** on hardware.

## B3. Targeted tests (41 runs, eco01 window c04, tags `t1`-`t7`; each row has a control and an evidence expression that the fault reached FAST-LIO2)

| Case | Fault | Runs, final design | Result |
|---|---|---|---|
| `h00`, `h40` controls | none / relay pass-through | 3 | `LOCALIZED`, pose 0.03 m; checks support 0.995, rmse 0.081, shift 0.01 m |
| `h27` | gyro 0.5 rad/s from 4 s | 3 (`t4`-`t6`) + matrix | no pose, `HANDOFF_FAILED(map_check_solver)` x3, `(map_check_geometry)` x1 |
| `d01`, `d02` | 0.5 from 8 s / 15 s | 2 + matrix | no pose (`map_check_solver`, `map_check_geometry`, `odometry_implausible`) |
| `d04`, `d05` | 1.0 from 8 s / 0.5 during 6-12 s | 1 + matrix | `ERROR(lio_diverged)`; `d05-t1` `LOCALIZED`, correct (0.016 m) |
| `d07` | 0.5 from 18 s (just before publication) | 3 + matrix | no pose, `map_check_geometry` x4 (first design: wrong pose, see B2) |
| `d08` | 0.5 from 21 s (after the pose, before `LOCALIZED`) | 3 + matrix | pose published correct (0.03 m) in all; then `HANDOFF_FAILED` x2, `LOCALIZED` -> `LOST` x2 (B7) |
| `d03`, `d06`, `d10`, `h44`, `h45`, `d09` | 0.1-0.3 rad/s, accel x1.5 | 8 | `LOCALIZED`, pose correct (max 0.05 m, 0.98 deg): FAST-LIO2 absorbs them. `h44`, `h45`, `d09` were predicted to fail: **FAIL EXPECT kept**, wrong predictions of mine |
| `h28` | 0.5 after `LOCALIZED` | 1 + matrix | `LOCALIZATION_LOST(map_incoherent)` |
| S2 eco01 z03 / S3 hospital z00 / cross map | F1 cases | 2 each | correct / correct / rejected |

Earlier runs of the first design (`t1`-`t3`) are in the tables with their tag; `d03-t4` shows `C?` (pose unscorable at 0.20 s, before the wide score existed; `d03-t7` and the matrix score it: 0.04-0.05 m).

## B4. Gate bypass and gate death (risks A and B)

Read in the pinned sources (V): `tracer_base` (tracer_ros2 `2c4938db`) sends one CAN frame per Twist (`TwistCmdCallback -> SetMotionCommand`), ugv_sdk `f2704eac` has no command timeout; whoever can publish on the driver's topic moves the base, and the last frame stays the last frame.

Design (commit `8a88d16`, no third-party patch):
- **Driver cell**: `tracer_base`, `gate_outlet.py` and `can_deadman.py` in a container with a private network namespace (`--network none`) that owns the CAN interface. No ROS participant outside the cell can discover the driver.
- `nav_gate.py` sends each allowed command as a datagram (sequence, sender clock, magic) over a unix socket whose directory is mounted only into the gate and the cell. `gate_outlet.py` publishes it to the driver, drops malformed / out-of-order / stale datagrams, and publishes **zero at 10 Hz when the gate is silent for 0.3 s**.
- `can_deadman.py` watches `0x111` on the bus: last frame non-zero and bus silent 0.3 s -> it writes zero frames until the driver speaks again. It **refuses any interface whose name does not start with `vcan`**: running it on a real bus is a decision for Narcís (it transmits on the base's command ID).

Test `gate_cell_test.sh`, real `tracer_base`, `vcan0` inside the cell, twice (`results/21-f2/f2g/gate-cell/v1`, `v2`; `all_pass: true`):

| Check | Result (v1 / v2) |
|---|---|
| Cell links; main graph sees `/tracer` nodes or the socket dir | `lo`, `vcan0` only; no; no |
| permit false / revoked / publisher gone | 0 non-zero frames (v2: 1 frame 5 ms after the edge) |
| permit true | first non-zero 0.26-0.29 s / 0.22-0.30 s after `true` |
| publisher alive but silent | last non-zero 0.36 / 0.39 s |
| bypass: publish on `/cmd_vel_gated`, on `/cmd_vel` with the gate closed, write to the socket from the main graph | 0 non-zero of 65 frames; socket `No such file or directory` |
| velocity source frozen 3 s while open | zeros (outlet zero-fill) |
| outlet killed while open, then restarted | dead-man zeros after 0.30 s; flow resumes |
| gate killed while open (v1) | 1 non-zero frame 8 ms after the kill, then zeros |
| driver killed while open (v2) | dead-man zeros after 0.30 s |
| host `can0` | unchanged, down, counters 0 |

What this does and does not close:
- Closed in the validated configuration (V on vcan): a ROS publisher outside the cell cannot command the driver; gate, outlet, source or driver death ends in zero frames on the bus within 0.3-0.4 s.
- **Not closed**: a process with root on the host, or one started inside the cell; death of the dead-man, of the cell or of the whole computer. Then stopping depends on the base firmware's command timeout, **NV** (no base on a virtual bus) -> F4-F6. No statement about a physical safe stop is made.
- Deployment needs the physical CAN interface moved into the cell's namespace (host networking): not done, not tested, decision for Narcís.

## B5. AMCL (risk C)

| Item | State |
|---|---|
| AMCL lost / silent / killed / frozen | `amcl_silent` 2.9 s after the fault (`h30`, `h31`), `amcl_not_confirmed` in hand-over (`h33`) |
| AMCL moved by a foreign `/initialpose` | `LOST` 3.7-4.2 s after (`h32`: `map_incoherent` in the final matrix, `amcl_incoherent` in `f2g`, whichever fires first; FAIL EXPECT by reason, kept); permit withdrawn 2.6 s earlier (B7) |
| Slow drift, late revocation | supervision now asks the map about AMCL's pose every 3 s, independent of the odometry (B7 for the measured latency) |
| AMCL keeps the pose after `HANDOFF_FAILED` | unchanged by design. Since B2 a pose only goes out after a passed check, so the kept pose was map-coherent when published. The contract for consumers is the permit, enforced by the gate; `/amcl_pose` alone is not an authorization |
| `LOCALIZED` as only signal | permit = heartbeat with TTL; `LOCALIZED` now needs acceptance + map check + AMCL coherence + second map check |

## B6. O1-O4 after the change (matrix B8)

| | Status | Evidence |
|---|---|---|
| O1 no sensors | works | `h01`, `h02` `TIMEOUT(no_sensor_input)`; `g01`, `g02` |
| O2 permit + gate | works | every matrix row: gated output non-zero only while permitted; closing 0.41 s worst after a frozen / killed node (`h34`-`h36`) |
| O3 revocable `LOCALIZED` | works | 18 revocations in the matrix, reasons as predicted except `h32` (B8) |
| O4 bounded buffers | works | odometry 1412 rows after 485 s; the new 3 s scan buffer adds ~13 MB to the node (B10) |
| monotonic watchdogs, bad stamps, FAST-LIO2 hangs | work | `h03`-`h08`, `h12`-`h24`, `h41`-`h43`, `g06`-`g11` |
| scratch files of the check | in `/dev/shm`, removed after each check and at start; no leftover after 158 checks (V) |

## B7. Revocation latency after `LOCALIZED`, measured with the reference

New evaluator tool `permit_exposure.py`: seconds of non-zero gated output while AMCL's pose is outside 0.30 m / 8 deg of the reference. Not predefined as a pass / fail criterion; no requirement exists yet (decision for Narcís).

| Run | Fault after `LOCALIZED` | Tree | Gate open | of which AMCL outside | Worst AMCL error while open |
|---|---|---|---|---|---|
| `h00`, `d09`, `h45`, memory run (485 s) | none / absorbed | `f2h` | 40-485 s | **0 s** | 0.13-0.16 m, 3.1-4.3 deg |
| `h32` foreign `/initialpose` (15 m) | | `f2g` | 10.2 s | 2.9 s | 15.7 m |
| | | `f2h` (+`u2`) | 8.2 / 7.3 s | **0 / 0 s** | 0.06 m |
| `h28` gyro 0.5 rad/s | | `f2g` (+`t4`) | 14.0 / 17.2 s | 2.3 / 3.5 s | 1.36 m, 45 deg |
| | | `f2h` (+`u2`) | 12.9 / 12.5 s | **2.3 / 1.6 s** | 1.23 m, 39 deg |
| `d08` gyro 0.5 rad/s from 21 s | | `f2g` (+`t6`) | 10.6 / 14.0 s | 2.9 / 3.1 s | 2.30 m, 27 deg |
| | | `f2h` | 0.7 s | **0.14 s** | 0.53 m, 8 deg (`u2`: `HANDOFF_FAILED`, never open) |

- Tree `f2g` (method `5bb95ef`) kept the permit until the loss rules fired (3 s hold, two failed checks). Commit `f524d04` withholds the permit **as soon as there is doubt**: AMCL outside the coherence limits, or a failed map check whose repeat has not passed. State and loss rules unchanged; the gate reopens if the doubt clears (never needed in a no-fault run: `withheld_in_doubt` = 0 in all of them, 485 s included).
- Effect: foreign pose 2.9 s -> 0 s; divergence starting before `LOCALIZED` 3 s -> 0.14 s; **divergence starting after `LOCALIZED`: still 1.6-2.3 s (N = 2)** with AMCL up to 1.2 m / 39 deg wrong. Cause: AMCL takes its motion prior from the same odometry and goes wrong between two map checks (period 3 s, check 0.9 s); the doubt needs one failed check or 15 deg / 0.5 m of AMCL-vs-own discrepancy.
- This is a detection latency, not zero and not hidden. Bound by construction: check period + check time (~4 s) after the scans stop fitting the map. Shorter needs a shorter period (CPU: one check = 0.9 s of one core) or an odometry-independent motion sensor (wheel encoders). At the nominal 0.34 m/s, 2.3 s is 0.8 m of travel.

## B8. Final fault matrix (tree `f2h` = `f524d04`): 56 cases, all valid

`results/21-f2/f2h/table-f2h.csv`. eco01 window c04, mode `s2s3`. 46 cases of the first matrix + `d01`-`d10`. Invariants per row: P1 pose only after `ACCEPTED`; **P2 pose only after a passed fresh map check; P3 `LOCALIZED` only after a passed second check** (new); G1 gated output non-zero only while permitted; G2 gate closed within 0.7 s of revocation; T1 terminal state; O1 no orphan process; C1 not `wrong_accept`; `C?` no unscored published pose.

| Verdict | Rows | |
|---|---|---|
| `PASS` | 51 | |
| `FAIL EXPECT`, behaviour correct, prediction wrong, **not edited** | 3 | `h44`, `h45`, `d09`: FAST-LIO2 absorbs 0.1-0.3 rad/s; pose correct (<= 0.05 m), AMCL never outside limits while open (B7) |
| `FAIL EXPECT` by reason | 1 | `h32`: predicted `amcl_incoherent`, got `map_incoherent` 3.7 s after the fault (both rules run; in `f2g` the other fired first). Gate closed 2.6 s before the state change |
| **`FAIL O1`** | 1 | `h35` (node frozen): one process matching `register_v2` alive when the harness counted, and the launch log lacks the node's exit line. Permit and gate correct (closed 0.41 s after the freeze). **Not reproduced in 3 repeats (`u3`-`u5`), not seen in `f2g`; cause not identified.** Hypothesis (I): the freeze caught a map check in flight and the node was still in `shutdown()` when the launcher was stopped. Open, non-critical: no pose, no permit, container teardown removes it |
| `INVALID` | 0 | (`u1`: 5 targeted rows invalid, my deployment mistake, binary missing; rerun as `u2`) |

- Pose published in 38 rows: XY error median 0.033 m, max 0.047 m; yaw max 1.0 deg; all scored with the 0.20 s rule. Wrong `/initialpose`: **0**. Pose without acceptance / without map check: 0.
- `h27`, `d01`, `d02`, `d07`: accepted, **nothing published**. `d04`: `ERROR(lio_diverged)`.
- 18 revocations; closing after the permit drop 0.41 s worst (`h34`-`h36`), otherwise the gate was already closed when the state changed.
- Map check on good data (38 rows): support 0.86-1.00, **rmse 0.079-0.090 against the 0.10 limit**. The margin on rmse is thin; it is the frozen F1 constant and was not touched. False `HANDOFF_FAILED(map_check_*)` on a no-fault run: 0 of 38 here, 0 of 43 in the restarts; in the 485 s run 0 of 158 supervision checks failed (max rmse 0.094). A noisier environment could fail it: availability risk, not a safety one.
- 25 cases of note 16 (`batch-g2h.log`): same terminal states as on the previous candidate (`batch-g2c.log`); `LOCALIZED` up to 5.9 s later.
- Storage stalls: 7 matrix / note-16 rows overlap a kernel NVMe timeout (`stalls-f2h.csv`); none shows a sampler hole (max 0.6 s), all conform. Kept as valid.

## B9. F1 regressions and negatives (`batch-r2h.log`)

| Case | Expected (F1) | `f2h` |
|---|---|---|
| eco01 z03, S2 | correct accept | correct, accepted 18.8 s |
| hospital z00, S3 | correct accept through S3 | correct, accepted 41.8 s (x2) |
| 14 negatives (7 cross-map, 3 map hole, 4 S3 cross-map) | rejected | **14 of 14 rejected** (`reject_geometry`), no pose |

The search, the acceptance gate, the submap builder and the 60 s clock are byte-identical to F1 (`git diff c026a7b HEAD -- search_v2.py verify.py correlative_search.py lio_submap.py search_tiles.py` is empty).

## B10. 50 software restarts (protocol changed from 200)

**Change of protocol, stated as such:** the original criterion was 200 restarts. 200 were run on candidate `98926f7` (Part A, section 7) and stay as evidence **of that version only**. The corrected candidate got 50. Reason: every method change invalidates the previous campaign, 200 runs cost 3.3 h of board time, and the campaign was in fact run twice here (`f2g`, then `f2h` after the permit change). 50 is not equivalent to 200: 0 of 50 leaves a Wilson 95 % upper bound of 7.1 %, against 1.9 % for 0 of 200. Neither says anything about hardware.

Design (`cases-restarts50.csv`, seed 20261011): the 36 stationary windows once, plus 14 extras on the hard ones (hospital c00, z00 (S3), c02, z02 x2 each; eco03 z02 x2; eco01 c01, c02, z01; eco03 c00). Start jitter +-0.5 s, pre-roll 2.5-4 s, lead 10/12/14 s, own DDS domain, fresh container, node and FAST-LIO2 per run.

| Quantity (`f2h`, N = 50, 0 invalid) | Value |
|---|---|
| `LOCALIZED`, pose correct | 41 (82 %, Wilson 69-90 %), S3 included (hospital z00 x3) |
| wrong acceptance / pose without acceptance / permit without correct pose | 0 / 0 / 0 (Wilson upper 7.1 %) |
| AMCL error ~2 s after `LOCALIZED` (41) | median 0.036 m, max 0.10 m, 0.4 deg |
| not localized, all without permit | 9: `s3_budget_insufficient` 6 (hospital c02 3/3, z02 3/3, known), `imu_not_stationary` 1 (eco03 z00), **`HANDOFF_FAILED(amcl_not_confirmed)` 2 (hospital c06, z06)** |
| FAST-LIO2 start failures, orphans, invariant failures | 0, 0, 0 |
| first input -> accepted | median 18.0 s, p95 41.9 s, max 42.7 s |
| first input -> `LOCALIZED` | median 27.8 s, p95 49.7 s, max 49.9 s (previous candidate 23.3 / 39.8 / 48.5 on its own mix) |
| `/initialpose` error (43 published) | XY median 0.03 m, max 0.075 m; yaw max 0.39 deg; 42 by the rule, 1 only with the 0.5 s bracket (0.075 m, class kept `unscorable_reference_gap`) |
| PSS peak, stack without the player | median 256 MB, p95 339 MB, max 355 MB |
| board `MemAvailable` | 14188 MB before, 14193 MB after |

- The `f2g` campaign (`f2g/table-rs50.csv`) gave the same outcome on every one of the 50 cases; the three non-trivial rejections repeat when rerun alone (`table-rs50b.csv`): deterministic per case.
- **New false rejects, caused by the h27 fix (2 of 50; these windows were 11 of 11 on the previous candidate):** in hospital c06 / z06 the platform moves at 0.65-1.0 m/s at hand-over. `/initialpose` is correct for its stamp (0.04 m) but 0.96 s old, AMCL applies it as current (B2) and starts 0.6 m behind along a corridor, never comes within the coherence limits, and the attempt ends `HANDOFF_FAILED` after 29 s with AMCL 0.5-0.7 m off. No permit. Not fixed: compensating the lag with odometry is the first design that failed `d07`. Expected with the old per-window rates on this mix: 43.4 `LOCALIZED`; observed 41.
- `imu_not_stationary` (eco03 z00, r006): this case has the shortest stationary span of the set (pre-roll 2.65 s, jitter +0.4 s) against the 2 s rule; the IMU stream had no gap. Same mechanism as the two cases of Part A. It overlapped an NVMe timeout in `f2g` and not in the reruns: counted as valid.
- 9 runs overlap a kernel NVMe timeout; no sampler hole, outcomes equal to the `f2g` run of the same case. Kept.

## B11. Memory, CPU, time

Same window as Part A (eco01 c00, 515 s of replay, `LOCALIZED` held 485 s over 177 m), one run per tree.

| | F1 | `98926f7` | `f2g` | **`f2h`** |
|---|---|---|---|---|
| node PSS first -> last (MB) | 87.9 -> 90.1 | 83.1 -> 85.6 | 97.6 -> 98.5 | **97.0 -> 99.0** |
| node slope (MB/min) | +0.28 | +0.28 | +0.11 | +0.14 |
| FAST-LIO2 PSS (MB) | 37.8 -> 45.3 | 39.5 -> 45.9 | 41.0 -> 45.7 | 40.8 -> 45.6 |
| stack PSS peak (MB) | 256.8 | 254.3 | 271 | **272.6** |
| stack CPU mean (% of one core) | 64.8 | 67.5 | 77 | **77.6** |
| map checks while `LOCALIZED` | - | - | 158, 0 failed | 158, 0 failed |

- Cost of the fix (V, one run each): +13 MB in the node (3 s scan buffer), +16 MB stack peak, +10 points of one core (a 0.9 s check every 3 s), `LOCALIZED` ~4 s later (one check before the pose, one before `LOCALIZED`, and AMCL's later convergence when moving).
- No growth beyond the earlier trees in 8 min; odometry buffer 1412 rows (cap 2400). FAST-LIO2 grows ~0.6-1 MB/min while travelling, as in F1. Beyond 8 min: **NV** (longest bag 634 s).
- Supervision held 485 s with no doubt and no revocation (AMCL vs own pose max 0.15 m, 3.7 deg; limits 0.5 m, 15 deg).

## B12. NVMe of the NX

- Kernel log (V, no root): `nvme nvme0: I/O tag ... timeout, completion polled`, **131 since 2026-10-05** (per day: 1, 0, 4, 3, 27, 59, 37; `f2h/nvme-timeouts-per-day.txt`). All in hours with experiments, bursts under load, none in idle hours. The message means a completion interrupt was missed and the request was recovered by polling after the 30 s timeout: a stall, not a reported data error.
- No `EXT4` error, no `I/O error`, no PCIe AER line in the journal (0 matches). Device "SSD NVME 256GB", firmware VC400622, link 8 GT/s x4, controller state live. 122 GB free. Single boot since 2026-10-05 (the journal starts there).
- SMART / error log: **NV**, needs root. Command for the human: `sudo nvme smart-log /dev/nvme0` and `sudo nvme error-log /dev/nvme0` (read-only).
- Effect on the tests: one run invalidated in Part A (`h36-f2b`). Final candidate: 16 runs overlap a timeout, none has a sampler hole > 0.6 s, none changed outcome against its twin. Evidence was always written; hashes in the identity file.
- Mitigation in the method: map-check scratch moved to `/dev/shm`, so a storage stall cannot block the supervision worker. The node still writes `supervision.json` every 10 s to disk; a stall there is caught by `supervisor_stalled` (permit already dropped by the heartbeat TTL).
- Nothing on the host was changed. Cause **NV**; candidates for Narcís: the SSD model / firmware with this carrier, APST / ASPM settings. A production robot should not ship with this unexplained.

## B13. Verdict against the criteria

| | Criterion | State | Evidence |
|---|---|---|---|
| A | h27 fixed, no known false accept | met | h27 x5 on the final design, `d01`, `d02` x3, `d07` x5: nothing published |
| B | O1-O4 work | met | B6 |
| C | F2 matrix complete and valid | met: 56 valid; 51 pass, 4 wrong predictions kept, **1 `O1` failure not reproduced (open, non-critical)** | B8 |
| D | F1 regressions | met | B9 |
| E | no known wrong `/initialpose` in scorable tests | met: 0 in 38 + 43 + 3 scored poses. Caveat: a correct but 1 s old pose makes AMCL start behind when moving (B10) |
| F | no undue navigation authorization | met at grant: every permit was given on a pose within limits (matrix, 41 restarts). **After an injected odometry fault the permit outlived a correct AMCL pose by 1.6-2.3 s (B7).** Read as "zero exposure after any fault" this criterion is not met and cannot be by a periodic check |
| G | gate revocable, bypass closed in the validated software configuration | met on vcan | B4 |
| H | virtual CAN integration | met, real `tracer_base`, 2 runs | B4 |
| I | 50 valid restarts of the final candidate | met (and 50 more on `f2g`) | B10 |
| J | memory / resources | met for 8 min | B11 |
| K | no open critical error in F2 scope | met; open non-critical: `h35` O1, hand-over lag when moving | B8, B10 |
| L | reproducible evidence | met | B15, identity file |
| M | physical limits identified | met | B14 |

## B14. Residual risks and limits

| Risk | State | Goes to |
|---|---|---|
| Permit outlives a correct pose by up to ~2.3 s (bound ~4 s) when the odometry diverges after `LOCALIZED` | measured, N = 2 | requirement from Narcís; wheel odometry; F5 |
| False reject when the platform moves > ~0.5 m/s at hand-over; AMCL left 0.5-0.7 m off without permit | measured, 2 of 50, deterministic | procedure (stand still until `LOCALIZED`; stop at the end of S3); F6 |
| rmse margin of the map check (0.08-0.094 vs 0.10) | measured on 3 maps, 0 false failures in 239 checks | F5 independent data |
| GICP could lock a diverged pose onto a look-alike place within 0.30 m / 8 deg of the truth-free pose | not observed; **NV** | F5 |
| `h35` shutdown left one process once in 5 | open, cause not found | F3 |
| Dead-man, cell or computer dies; root on the host | stop depends on the firmware timeout, **NV** | F4-F6 |
| Physical CAN inside the cell namespace; dead-man on a real bus | not done | Narcís |
| NVMe stalls | measured, cause NV | Narcís |
| Memory beyond 8 min, cold boot, live Livox driver, real CAN, E-stop | NV | F3-F6 |
| One window for the fault matrix, 36 windows / 3 maps for restarts, sensor facing up, provisional mount, development data | unchanged | F4-F5 |

Own mistakes of this round: (1) first map-check design propagated past the check (`d07-t1`, wrong pose published, kept); (2) supervision first anchored on the node's own pose; (3) predictions `h44`, `h45`, `d09`, and `h32`'s reason, wrong; (4) `u1` run without the registration binary; (5) the revocation latency was only measured with the reference after the first 50 restarts, which cost a second full campaign; (6) evaluator unit test for the wide bracket written wrong at first.

## B15. Reproduce

NX roots (raw data, not in git): `/home/jetson/work/reloc-f2h-20261011/` (final: `runs/`, tables, `chain5.sh`), `/home/jetson/work/reloc-f2g-20261011/` (method `5bb95ef`: targeted `t1`-`t7`, matrix, restarts, `gate-cell/v1`, `v2`), Part A roots unchanged.

```
R=<root> bash tools/global_reloc_3d/batch_faults_f2.sh <tag> [case-prefix]
R=<root> bash tools/global_reloc_3d/batch_faults_s2s3.sh <tag>
R=<root> bash tools/global_reloc_3d/batch_s2s3.sh cases-f1.csv <tag>            # also cases-f1b.csv, cases-sn.csv
python3 tools/global_reloc_3d/gen_restart_cases.py --extra hospital:c00,... > cases-restarts50.csv
R=<root> bash tools/global_reloc_3d/batch_restarts.sh cases-restarts50.csv <tag> 0 49
python3 tools/global_reloc_3d/restart_summary.py table-<tag>.csv runs
python3 tools/global_reloc_3d/io_stall_audit.py runs
docker run ... socialtech:mapper-verify python3 -B /t/permit_exposure.py --run-dir /runs/<run> --reference ... --map-yaml ...
bash tools/global_reloc_3d/gate_cell_test.sh <out> [FINAL=gate|driver]            # needs: sudo modprobe vcan (host), never can0
cd tools/global_reloc_3d && for t in test_*.py; do python3 -B $t || break; done   # 12 files; test_reloc_core 29 tests
```

---

# Part A — history: candidate `GR-S2S3-F2` = `98926f7` (2026-10-10, verdict PARTIAL, superseded)

Kept unchanged below. Its open defect (section 4b) is closed by B1-B3; its gate risks (section 6) by B4; its 200 restarts (section 7) remain evidence of that version only. Evidence: `results/21-f2/` top level.

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
