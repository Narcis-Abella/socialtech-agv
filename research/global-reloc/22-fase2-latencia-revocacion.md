# 22 — Phase 2: revocation latency after `LOCALIZED`

Date: 2026-10-11. Branch `feat/global-reloc`, local commits, nothing pushed. F1 `c026a7b` untouched.
Candidate: **`GR-S2S3-F2c`** = `GR-S2S3-F2b` (note 21, Part B) + method commits `4211c6b`, `6cb779a`. Gate files unchanged (`8a88d16`). Identity and hashes: `results/22-f2-latency/f2l/candidate-identity.txt`.
Labels: **V** measured or read in code, **I** inferred, **NV** not verified.
Level of every result: `VALIDATED_IN_REPLAY`, development data, Orin NX at 1x, `jetson_clocks` on. Nothing ran on hardware, no robot moved, host `can0` never used. The reference is read only by the offline evaluator.

**Verdict: `FASE 2: VERIFIED — VALIDATED_IN_REPLAY`** (section 9), with a proposed software requirement for the revocation time that Narcís has not approved yet. No latency is declared safe for physical motion.

## 1. The problem and its root cause

Note 21 (B7): when FAST-LIO2 diverges after `LOCALIZED`, AMCL follows the same odometry and leaves 0.30 m / 8 deg while the permit is still true.

Metric (fixed before the new runs, `permit_exposure.py`): `latency_s` = time from the first AMCL pose of the run of outside poses that is still going on when the gate closes, to the last non-zero gated output. Resolution: AMCL publishes a pose every 0.1-0.5 s. `0` = the gate closed while AMCL was still inside the limits.

Previous candidate, same metric (V, `f2l/before-f2h-f2g.jsonl`):

| Run (tree) | AMCL outside at | first failed check: scans / verdict | gate closed | latency | worst error meanwhile |
|---|---|---|---|---|---|
| `h28-f2h` (`f524d04`) | 10.63 s | 12.1 / 13.14 s | 13.17 s | **2.54 s** | 1.23 m, 39 deg |
| `h28-u2` (`f524d04`) | 11.13 s | 12.1 / 13.09 s | 12.76 s | **1.63 s** | 0.60 m, 13 deg |
| `h28` x2, `d08` x2 (`5bb95ef`, permit kept until the loss rule) | | | | 2.76-3.74 s | up to 2.3 m, 45 deg |

(Seconds after `LOCALIZED`. Note 21 gave 2.3 / 1.6 s for the first two: that was the sum of scored outside time; `latency_s` also counts the poses with no reference bracket.)

Decomposition of `h28-f2h` (V): the last passed check used scans up to 9.1 s; AMCL left the limits at 10.63 s; the next check started at 12.1 s (**1.47 s waiting for the 3 s period**) and answered at 13.14 s (**1.04 s per check**); the gate followed in 0.03 s. In `h28-u2` the gate closed 0.33 s before the verdict because the AMCL-vs-own-pose doubt fired first.

Where the 1 s of a check went (V, `metrics.json` of the same runs and a profile): GICP 0.15 s; a new Python interpreter and its imports ~0.4 s; ~0.3 s voxelizing 3 s of scans (`np.unique` over rows); the rest file I/O and scoring.

Root cause: **scheduling, not sensitivity**. One check per 3 s, each a new process, so a pose that stops fitting the map is looked at 0-3 s later and judged 1 s after that. The only faster signal (AMCL against the node's own propagated pose, 0.5 m / 15 deg) is blind to this fault by construction: both move with the same odometry.

## 2. Options

| Option | Verdict |
|---|---|
| A. Same check, much more often: persistent worker, cheaper check, back to back | **chosen** |
| B. Permit only while a passed check is recent (freshness) | **chosen**: bounds the exposure when checks are late or missing, whatever the reason |
| C. Judge the newest scans on their own with the unchanged rule (tail) | **chosen**, after run `k11_r3-w1` (section 4) |
| D. Revoke on the first failed check | not needed: the permit is already withheld at the first failure; the state keeps the 2-failure rule so that one blocked view does not end the session |
| E. A stricter support / rmse / shift limit for supervision, or a trend rule ("support fell by x") | rejected: a new threshold chosen on the runs it would be scored on. The frozen constants stay |
| F. Lower the 3 m/s odometry plausibility bound | rejected in note 21, same reason |
| G. Raw gyro against the odometry | rejected: not independent (FAST-LIO2 integrates that gyro) and it cannot tell a bias from a turn |
| H. Wheel odometry of `tracer_base`, or commanded against measured motion | the right independent signal. Not in the bags, the replay does not execute commands: **NV**, recommended for F4 |
| I. A cheap per-scan score at AMCL's pose, no GICP | not pursued: AMCL's own 0.05-0.15 m error moves a 0.15 m support score; it would need option E |

## 3. What changed (commits `4211c6b`, `6cb779a`)

- `map_check.py --serve`: one worker process, map loaded once, one JSON request per line. It serves the check before `LOCALIZED` and every check after it. The check before `/initialpose` stays a one-shot process. Both call the same `run_check()`.
- Voxelization inside `map_check.py` replaced by one with the **same output bit for bit** (unit test against `offline_s2.voxel`): the three cell indices packed into one integer. `offline_s2.py` (F1) is not touched.
- `MAP_CHECK_PERIOD_S` 3 -> 0.5 s. A check now takes 0.30-0.35 s (0.40-0.45 s when GICP struggles), so they run one after another, about every 0.55 s.
- `MapSupervisor` (`reloc_core.py`): **the permit is true only if the last check passed and a check has passed on scans at most `MAP_FRESH_S` = 2.0 s old.** A late worker, a starved board or a silent cloud stream withhold the permit; they do not need to be detected one by one. Hung worker: killed after the existing 8 s limit, counted as a failed check, restarted.
- Tail rule, after `LOCALIZED` only: the unchanged geometric rule (support_015 >= 0.70, rmse <= 0.10) must also hold on the newest 0.5 s of scans (never fewer than the 3 newest), placed with the same refined pose. Before `LOCALIZED` the tail is recorded and not enforced, so the hand-over decides exactly as in note 21.
- Unchanged: every acceptance threshold, the loss rule (2 consecutive failed checks -> `LOCALIZATION_LOST(map_incoherent)`), the AMCL-vs-own monitor, the other supervision rules, the gate, the search, the 60 s clock. No ground truth online.

Why 2.0 s: one check may be late or lost without the gate flapping (worst age of the last pass in 15 no-fault runs: 1.22 s), two may not.

## 4. How the design was reached (every tree kept, `results/22-f2-latency/intermediate/`)

| Tag, NX root | Tree | Runs | Result |
|---|---|---|---|
| `w0`, `f2i` | - | 3 | **INVALID**, infrastructure: my new root lacked `tools/nav_bench`; nothing ran |
| `w1`, `f2i` | worker, period 1 s, freshness | 3 x `k11` | latency 0.94, 0.92, **2.19 s**: in `r3` a check passed at support 0.700 with AMCL 0.46 m / 12 deg off. The rule looks at the last second of a 3 s window as a whole -> option C |
| `w2`, `f2i` | + tail rule, confirm check served | 23 | controls clean; revoked N = 19: median 0.56 s, p95 1.58 s, **max 1.81 s**. The tail decided 1 of 308 checks (`k11-w2`, doubt 0.6 s earlier). Age of the last pass up to 1.84 s: 0.16 s from the freshness limit, too thin |
| `w3`, `f2j` | period 0.5 s | 5, stopped by me | worker CPU measured for the first time: **0.57 CPU-s per check, 88 % of one core** back to back, of which GICP 0.15 s -> faster voxelization |
| `x0`, `x1`, `f2k` | + fast voxel (`4211c6b`) | 1 + 16, stopped by me | revoked N = 13: median 0.52 s, **max 0.76 s**. **No-fault control `k40-x1`: one failed check, `tail_missing` (2 scans in the newest 0.5 s), gate closed for 0.7 s.** My rule failed for lack of data -> `6cb779a` |
| `y1`-`y6`, `f2l` | **final** (`6cb779a`) | section 5 | |

Own mistakes in this round: the incomplete root (`w0`); a wait loop that matched its own command line (no effect on data); `tail_missing` treated as a failure (found by the control, fixed); the first estimate "0.3 s per check" was a guess, the measurement said 0.57 s.

## 5. Targeted runs on the final tree (`f2l`, eco01 c04, `batch_latency.sh y1`, 23 runs, all valid, all PASS)

Faults enter FAST-LIO2 through the relay after `LOCALIZED` (25-29 s after the first input); evidence that the relay altered the stream and that FAST-LIO2 was fed is in `table-y1.csv`. Expectations written in the script before the runs: latency <= 1.5 s expected, never above 2.5 s; controls with no failed check and no withheld permit.

| Run | Fault into FAST-LIO2 (rad/s, s after first input) | End | AMCL outside at | Gate closed | **Latency** | Worst error exposed | Checks / failed |
|---|---|---|---|---|---|---|---|
| `k00` | none | LOCALIZED | - | 54.42 s | **0.00 s** | - | 99 / 0 |
| `k40` | none, through the relay | LOCALIZED | - | 54.43 s | **0.00 s** | - | 99 / 0 |
| `k10` | gyro +0.5 from 31 s | LOST map | 13.21 s | 14.17 s | **0.96 s** | 0.86 m, 13 deg | 27 / 2 |
| `k11` | gyro +0.5 from 35 s | LOST map | - | 10.83 s | **0.00 s** | - | 21 / 2 |
| `k12` | gyro +0.5 from 38.5 s | LOST map | 13.87 s | 15.33 s | **1.46 s** | 0.56 m, 9 deg | 29 / 2 |
| `k13` | gyro +0.5 from 42 s | LOST map | - | 17.00 s | **0.00 s** | - | 32 / 2 |
| `k14` | gyro +0.5 from 47 s | LOST map | - | 25.71 s | **0.00 s** | - | 48 / 2 |
| `k20` | gyro +0.3 from 35 s | LOST map | - | 18.61 s | **0.00 s** | - | 35 / 2 |
| `k21` | gyro +0.4 from 35 s | LOST stalled (end of replay) | - | 56.52 s | **0.00 s** | - | 104 / 0 |
| `k22` | gyro +0.7 from 35 s | LOST map | 10.46 s | 10.87 s | **0.41 s** | 0.44 m, 12 deg | 21 / 2 |
| `k23` | gyro +1.0 from 35 s | LOST odom speed | 9.07 s | 9.81 s | **0.74 s** | 1.63 m, 38 deg | 17 / 0 |
| `k24` | gyro +2.0 from 35 s | LOST odom speed | 9.05 s | 9.69 s | **0.64 s** | 1.47 m, 85 deg | 17 / 0 |
| `k25` | gyro +1.0 from 42 s | LOST odom speed | 17.19 s | 17.54 s | **0.35 s** | 0.59 m, 49 deg | 32 / 1 |
| `k30` | gyro +1.0, 35-37 s only | LOST odom speed | 9.99 s | 10.44 s | **0.45 s** | 0.71 m, 20 deg | 19 / 0 |
| `k31` | accel x2 from 35 s | LOST map | - | 49.71 s | **0.00 s** | - | 92 / 3 |
| `k32` | accel x4 from 35 s | LOST odom speed | - | 8.17 s | **0.00 s** | - | 15 / 0 |
| `k33` | gyro -0.5 from 35 s | LOST map | 13.94 s | 14.67 s | **0.73 s** | 0.36 m, 18 deg | 28 / 2 |
| `d08` | gyro +0.5 from 21 s (after the pose) | LOST map | - | 3.08 s | **0.00 s** | - | 7 / 3 |
| `d07` | gyro +0.5 from 18 s (hand-over) | HANDOFF_FAILED, no pose | - | - | - | - | - / - |
| `h27` | gyro +0.5 from 4 s | HANDOFF_FAILED, pose out, no permit | - | - | - | - | - / - |
| `h32` | foreign `/initialpose` at 35 s | LOST map | - | 9.77 s | **0.00 s** | - | 20 / 2 |
| `k11_r2` | gyro +0.5 from 35 s | LOST map | 13.85 s | 14.71 s | **0.86 s** | 0.58 m, 12 deg | 28 / 2 |
| `k11_r3` | gyro +0.5 from 35 s | LOST map | - | 10.28 s | **0.00 s** | - | 20 / 2 |

(Times in seconds after `LOCALIZED`. "-" under "AMCL outside at": never outside while the gate was open. Full rows: `f2l/latency-y1.csv`, `table-y1.csv`.)

Distribution over the 23 runs of the final tree that ended revoked with the gate open (19 here + `h28`, `h29`, `h32`, `d09` of the matrix, section 8): **13 at 0 s; the other 10: 0.35, 0.41, 0.45, 0.64, 0.73, 0.74, 0.86, 0.96, 1.13, 1.46 s** (median of those 10: 0.74 s; over the 23: median 0, p95 1.13 s, max 1.46 s). FAST-LIO2 does not repeat under these faults (3 x `k11`: 0, 0.86, 0 s), so one run per condition is a sample, not a value.

- **Controls** (`k00`, `k40`): 99 checks each in 54 s, 0 failed, permit never withheld, last pass at most 1.17 s old.
- **Revocation worked in every adverse run** that left the limits. In 13 of 23 the gate closed before AMCL left the limits: the bent scans stop fitting before the pose is 0.30 m off.
- **Slowest: `k12`, 1.46 s** (0.56 m, 9.4 deg). AMCL was 0.39-0.55 m off, mostly along the corridor, and two checks still passed (support 0.863 and 0.726, tail 0.704, limit 0.70): walls stay within 0.15 m of the map when the error is along them. This residue is **sensitivity of the frozen rule in a corridor**, not scheduling. Making it disappear means option E or H.
- `k21` (0.4 rad/s) was absorbed by FAST-LIO2; its `LOCALIZATION_LOST(odometry_stalled)` is the end of the replay. The freshness rule withheld the permit 1.0 s before that rule fired (`withheld_stale` = 10).
- `d08` (bias from 21 s) and `h32` (foreign `/initialpose`): gate closed before any outside pose.

Before / after, same metric:

| | Trees | N revoked | median | max | worst error while exposed |
|---|---|---|---|---|---|
| note 21 | `f524d04` | 2 | 2.1 s | 2.54 s | 1.23 m, 39 deg |
| this note | `6cb779a` (`y1`, `f2l`) | 23 | 0 s (0.74 s over the 10 above zero) | **1.46 s** | 1.63 m, 38 deg (`k23`, 0.74 s); 85 deg (`k24`) |
| | `4211c6b` (`x1`, same but `tail_missing`) | 13 | 0.52 s | 0.76 s | 1.32 m, 71 deg |

The error reached during the exposure does not shrink in proportion: a diverged FAST-LIO2 moves the pose by metres per second (`k23`: 1.6 m in 0.74 s).

## 6. Proposed requirement (software, replay) and what bounds it

Proposal for Narcís, not a physical safety limit:

> **R-REV: from the instant AMCL's pose leaves 0.30 m / 8 deg to zero output of the gate: at most 2.5 s.**

- By construction (V, code + unit tests `test_map_supervisor_*`): the permit is true only if AMCL's pose fitted the map (frozen rule) on scans at most 2.0 s old. The gate closes at most 0.41 s after the permit goes (worst case of note 21) -> 2.4 s after the last scans that fitted, whatever happens to the checks.
- Measured against the reference: max 1.46 s in 23 + 13 revocations on the two last trees; margin 1.0 s. N is small: 0 violations in 36 leaves a 95 % upper bound of about 10 % (Wilson) for a run above 2.5 s under faults like these.
- What the bound does **not** cover: a pose that is outside 0.30 m / 8 deg while the scans still fit the map under the frozen rule (the corridor of `k12`; a wrong pose in a place that looks the same). There the check keeps passing and nothing here measures for how long it could. In the tested faults that state lasted at most 1.1 s (`k12`).
- At the nominal 0.34 m/s, 2.5 s is 0.85 m of travel and 1.46 s is 0.50 m. Whether that is acceptable depends on the obstacle-safety layer, which does not exist in this scope: **decision for Narcís**.
- Tighter needs an independent motion sensor (wheel odometry, option H) or a stricter supervision rule validated on held-out data (option E).

## 7. Cost on the Orin NX

Same window and length as note 21 B11 (eco01 c00, 515 s of replay, 177 m, one run per tree; `f2l/mem-f2l-m5.json`, run `mem-eco01-c00-m5`):

| | `f2h` (note 21) | **`f2l`** |
|---|---|---|
| `LOCALIZED` held | 485 s | 492 s |
| map checks while `LOCALIZED` / failed | 158 / 0 | **895 / 0** |
| permit withheld by a doubt | 0 | 0 (1.0 s at the very end: the replay stopped) |
| worst passed check: support / tail support / rmse | 0.99 / - / 0.094 | 0.989 / 0.992 / 0.096 (limits 0.70, 0.70, 0.10) |
| check worker CPU (% of one core) | ~30 (0.9 s per 3 s, I) | **53.4** (262.9 CPU-s, V from `/proc`) |
| stack CPU mean / p95 (% of one core, no player) | 77.6 / - | **95.1 / 117.7** |
| node PSS first -> last (MB) | 97.0 -> 99.0 | 94.2 -> 96.0 |
| stack PSS peak (MB) | 272.6 | 272.1 |

- **+17.5 points of one core** (of 8 on the NX) for a check about every 0.55 s instead of every 3 s; memory unchanged (the worker's map and the scan buffer already existed). Per check: 0.29 CPU-s, of which GICP 0.15 s; 0.57 CPU-s before the voxel change.
- The knob is `MAP_CHECK_PERIOD_S`. At 1.0 s the worker would cost about 29 % of a core (I, 0.29 CPU-s per check) and the scheduling part of the latency grows by up to 0.5 s; `w2` ran that period with the slower check (max 1.81 s). Decision for Narcís if the CPU is wanted elsewhere (Nano: 6 cores, **NV**).
- `LOCALIZED` comes 1-3 s earlier than in note 21 on the 9 restart windows run again (e.g. eco01 z00 28.7 -> 25.0 s, hospital c00 S3 43.3 -> 42.4 s): the two checks around the hand-over are faster. N = 1 per window.
- Beyond 8 min: **NV**, as before.

## 8. Regressions on the final tree

All on tree `6cb779a`, root `reloc-f2l-20261011`. Unit tests: 12 files pass (`test_reloc_core` 31 tests, `test_map_check` 10).

| What | Runs | Result |
|---|---|---|
| S2 eco01 z03, S3 hospital z00 (`batch-r2l.log`) | 2 (+1 repeat) | correct, accepted at 18.8 s / 42.0 s, `LOCALIZED` at 25.1 s / 48.4 s |
| Negatives: 11 cross-map + 3 holes | 14 (+1 repeat) | **14 / 14 `REJECTED(reject_geometry)`** |
| Note-21 matrix, the 41 cases that reach the hand-over or go beyond (`table-f2l.csv`) | 42 + 1 | 38 conform; `h44`, `h45`, `h32`: the same FAIL EXPECT as in note 21 (my wrong predictions, behaviour correct, not edited); **`h25` INVALID** (a 26.9 s storage stall starved the bag player before FAST-LIO2 was killed; the node revoked with `odometry_stalled`), run again as `f2l2`: PASS `fastlio_exited_-9`. `h35` PASS this time |
| Poses published on this tree (matrix 38, targeted 22, other maps 11) | 71 | 70 scored by the rule (0.20 s bracket): max 0.065 m, 1.04 deg. 1 scored only with the 0.5 s bracket (`h27-y1`, below). 0 wrong; 0 without acceptance or check; permit never true outside `LOCALIZED` |
| h27 (bias from 4 s) | 7 | 6 x nothing published (`map_check_solver` x3, `map_check_geometry`, `lio_diverged` x2). **`h27-y1`: pose published**, 0.07 m / 5.1 deg from the reference (wide 0.5 s bracket; unscorable at 0.20 s), then `HANDOFF_FAILED(amcl_not_confirmed)`, **permit never true** |
| d07, d01, d02, d05 (bias around the search and hand-over) | 5 | nothing published |
| d08 (bias after the pose) | 2 | pose correct (0.03 m); once `HANDOFF_FAILED`, once `LOCALIZED` 3.1 s then revoked, gate closed before any outside pose |
| Matrix runs that end revoked | 18 | reasons as in note 21 (`d09` now `map_incoherent`: predicted, it was absorbed before). Gate closing after the revocation: worst 0.41 s (`h36`, as before). Input stalls (`h07`, `h10`, `h11`, `h26`): the gate now closes ~1.07 s **before** the 3 s stall rule fires, by the freshness rule |
| Supervision on other maps, no fault (`table-sv.csv`): 5 hospital, 2 eco03, 3 eco01 windows, 42-64 s after `LOCALIZED` | 11 | 10 `LOCALIZED`, kept to the end, pose 0.007-0.065 m; `z06`: `HANDOFF_FAILED(amcl_not_confirmed)` as in note 21 (platform moving at the hand-over); `c06`, which failed the same way in note 21, localized (N = 1). **1036 checks, 3 failed** (below) |
| Long run, 492 s | 1 | section 7 |
| Gate cell, real `tracer_base`, `vcan0` (`f2l/gate-cell-v3/`) | 1 | `all_pass: true`; host `can0` unchanged (down, counters 0). Gate, outlet and dead-man files are byte-identical to note 21 |

**h27 in `y1`, read by hand.** The check before `/initialpose` passed (support 0.827, rmse 0.0965, shift 0.03 m / 1.5 deg) and the published pose was inside the limits; AMCL then got diverging odometry and never agreed, so `LOCALIZED` was never reached. This is the design working as in `d08`, not the defect of h27 (a wrong pose published). But note 21 said "h27 5 / 5 nothing published": over the 13 runs on four trees it is now 12 / 13. The decision path before `/initialpose` is the one of note 21 (same function, bit-identical voxelization), and FAST-LIO2 does not repeat under this fault, so I attribute the difference to that (I); it is not proven.

**Spurious failed checks in no-fault runs (V).** 3 of 1036 on the other maps (hospital c01 at 4.5 s; eco03 s2b1 at 4.4 and 27.8 s), 0 of 895 + 198 on eco01 c00 / c04. Each closed the gate for about 0.6 s (the repeat passed at once; the state stayed `LOCALIZED`). Cause found with a diagnostic rerun that keeps failed checks (`f2m/failed-check-r022/`, tree = final + an off-by-default switch): GICP stopped at its iteration limit (19 iterations, `gicp_converged: false`, 1405 inliers) on a pose that the checks 0.5 s before and after pass at support 0.999. The frozen solver rule counts "not converged" as a failure. Not changed: accepting unconverged solutions is a relaxation that needs its own validation. Consequence: 3 stops of ~0.6 s in 9.4 min of supervised replay on those windows, none in 12 min on eco01 c00 / c04 (N is small). Availability, not safety.

### Why the 50 restarts were not repeated

- Up to `/initialpose` the decision path is the one of note 21: same one-shot check, same `run_check()`, same rule, voxelization proven bit-identical; the tail is only recorded there. The check runs ~0.15 s faster, so the published stamp is that much fresher.
- The check before `LOCALIZED` is now served (same rule, ~0.6 s faster). After `LOCALIZED` everything changed, and the restart campaign stops there.
- So the campaign's question (does a fresh start localize, and never on a wrong pose?) is answered by code the change does not alter. What the change can alter is (a) the hand-over timing and (b) false withholding on other maps: covered by S2, S3 and 14 negatives, by 11 restart cases kept running 42-64 s after `LOCALIZED` on the three datasets, and by the 492 s run. The 50 restarts of note 21 remain evidence of candidate `GR-S2S3-F2b`, not of this one.

## 9. Closing criteria

| Criterion of the brief | Status |
|---|---|
| Revocation works in every adverse scenario tried | met (V): the 23 runs of section 5 and the 18 revoked runs of the matrix; no adverse run kept the permit |
| Residual latency measured and bounded within the experimental scope | met (V): max 1.46 s, 13 of 23 at 0 s; before 1.63-2.54 s (N = 2). Structural bound 2.4 s after the last scans that fitted the map. The limit of that bound is stated (section 6) |
| A justifiable temporal requirement is proposed, no physical limit invented | R-REV = 2.5 s, software, replay. **Not approved by Narcís** |
| No permit granted initially on a known wrong pose | met (V): 0 wrong in 71 published poses of this round (70 scored by the rule, max 0.065 m) |
| No critical regression | met: S2, S3, 14 / 14 negatives, matrix outcomes as in note 21, bypass closed on vcan. Not critical, reported: `h27` once published a correct pose (no permit); spurious failed checks (0.3 % on two maps) |
| CPU cost acceptable on the NX | +17.5 points of one core of 8, memory unchanged. Acceptable for the NX in my judgment; the period is a knob (section 7) |

Verdict: **`FASE 2: VERIFIED — VALIDATED_IN_REPLAY`** for candidate `GR-S2S3-F2c`.

- Latency requirement used: R-REV = 2.5 s (section 6), mine, justified by the freshness rule plus the measured gate closing time, with 1.0 s of measured margin.
- Pending from Narcís: approval of R-REV or another value; whether a 0.5-0.85 m blind travel is acceptable without an independent obstacle-safety layer; the decisions of notes 20 and 21 (firmware command timeout, CAN cell deployment, NVMe).
- Not claimed: physical validation, AGV safety, F4-F6, that the 50 restarts of note 21 are evidence of this candidate.
- If "no exposure at all after any fault" is the reading of the criterion, the verdict is PARTIAL: a check that looks at the map cannot see a pose error the map does not show.

## 10. Residual risks and limits

- **Residual exposure**: up to 1.46 s measured (0.56 m, 9 deg); in a corridor an along-axis error can pass the frozen rule for about 1 s. Violent divergences move the pose 1-1.6 m and tens of degrees inside 0.4-0.75 s (`k23`, `k24`): no check period makes that small; it needs an independent motion sensor.
- **One fault family**: IMU faults through the relay (gyro bias +/-0.3 to 2.0 rad/s, accel x2 / x4, one transient), one window of one map (eco01 c04) for the faults; other maps only without fault. A slow drift that FAST-LIO2 hides for minutes was not produced (0.3-0.4 rad/s is either absorbed or caught).
- **Availability**: spurious `solver` failures (section 8) and the thin rmse margin on good data (0.08-0.096 against 0.10) stop the output for ~0.6 s now and then. More scenes may be worse: **NV**.
- **CPU**: one core is now ~95 % busy on average for the whole stack; not measured together with Nav2 planners or the AI layer, nor on the Nano.
- **Storage**: 12 NVMe timeouts today, one run invalidated by a 26.9 s stall (`h25-f2l`). The check worker reads and writes only memory-backed files; the bag player and the recorders do not. Cause **NV**, SMART needs root.
- Everything of note 21 B14 still holds (firmware timeout, physical CAN, cold boot, real driver, memory beyond 8 min, hand-over while moving).
- The 50 restarts were not repeated for this candidate (section 8).

## 11. Reproduce

NX roots (raw data, not in git): `/home/jetson/work/reloc-f2l-20261011/` (final: `runs/`, tables, `chain6.sh`, `chain7.sh`), `reloc-f2i`, `reloc-f2j`, `reloc-f2k-20261011/` (intermediate trees, each with its `NOTE-*.txt` and tool hashes).

```
R=<root> bash tools/global_reloc_3d/batch_latency.sh <tag> [case-prefix]         # targeted faults after LOCALIZED + controls
python3 tools/global_reloc_3d/latency_summary.py exposure-<tag>.jsonl table-<tag>.csv runs > latency-<tag>.csv
R=<root> RUN_ENV="KEEP_PLAYING=1 PLAY_S=92 STOP_AT=85" bash tools/global_reloc_3d/batch_restarts.sh cases-restarts50.csv <tag> <i> <i>
R=<root> bash tools/global_reloc_3d/batch_faults_f2.sh <tag> <case-prefix>      # matrix cases of note 21
R=<root> bash tools/global_reloc_3d/batch_s2s3.sh cases-f1.csv <tag>            # also cases-f1b.csv, cases-sn.csv
bash tools/global_reloc_3d/gate_cell_test.sh <out>                                # virtual CAN only
cd tools/global_reloc_3d && for t in test_*.py; do python3 -B $t || break; done   # 12 files
```
