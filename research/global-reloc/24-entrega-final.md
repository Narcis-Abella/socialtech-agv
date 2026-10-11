# 24 — Handover: automatic global relocalization, candidate `GR-S2S3-F2c`

Date: 2026-10-11. Branch `feat/global-reloc`, PR base `feat/nav-bench-a` (never `main`). Documentation only: no code, parameter, benchmark or evidence file was changed by this note.
This is the entry point for review. It consolidates; every number points to the note and table that own it (the **source of truth** is the note named in the last column, not this file). Phase status lives only in [19-plan-cierre-y-entrega.md](19-plan-cierre-y-entrega.md).

**Scope in one line: `VALIDATED_IN_REPLAY` (F1, F2) and `VALIDATED_IN_REPLAY_ON_JETSON_NX` (F3). Nothing ran on the robot. F4, F5, F6 are `BLOCKED_PHYSICAL`. This is a software handover, not a validation of the AGV.**
Labels: **M** measured, **C** read in code, **I** inferred, **NV** not verified.

## 1. Problem and objective

Today the operator clicks the start pose in RViz (`/initialpose`). Goal (`AGENTS.md`): the robot finds its own initial pose in a known map at boot, with no operator, then AMCL takes over. Indoor, flat floor, ~0.34 m/s, Livox Mid-360 (mounted facing up in all bags; the final mount looks down), Orin NX 16 GB. What matters most is **wrong acceptance**: driving on a wrong pose.
Definitions (frozen in [02-protocolo-evaluacion.md](02-protocolo-evaluacion.md)): *accepted* = the method's own gate says localized; *wrong acceptance* = accepted with error > 0.30 m or 8 deg against the reference; one 60 s clock from the first sensor message to `ACCEPTED`. The reference is a Voxel-SLAM trajectory with its own unmeasured error (centimetre-level claims are not resolvable). `LOCALIZED` is the node's state (AMCL agrees with the propagated pose), never an external accuracy proof.

## 2. Research and selection

| Step | What was done | Result | Source |
|---|---|---|---|
| Survey | 20+ candidates read in upstream code and docs (AMCL global, KISS-Matcher, Quatro, Scan Context, BTC, FAST-LIO-Localization-QN, hdl_localization, lidar_localization_ros2, als_ros, prism_loc, Cartographer, 3D-BBS, TEASER++, OFL, STD, MapClosures, BEVPlace++, RING++, emcl2, GeoTransformer; no-LiDAR options). Licences and Jazzy/aarch64 status recorded as verified / not verified | working ranking approved by Narcís 2026-10-07: 0 AMCL global (baseline), 1 LIO submap + KISS-Matcher + small_gicp, then BBS, MapClosures, 3D-BBS | note 01 |
| Baseline | AMCL global localization, exploratory, S2 windows (eco01, hospital), 0.25x and 1x, 5000-20000 particles | `AMCL_WEAK_BASELINE`: no accepted pose in the complete 0.25x runs; final modes 16.4-16.7 m off. Development diagnosis, not a rate; 1x failures are integration limits and are not counted as localization failures | notes 07, 09, 10 |
| Hypothesis of the task (KISS + small_gicp) | built on aarch64/Jazzy (source build of the cores) and run | works when >= ~25 % of descriptor pairs are right (eco01 S2: 0.02 m / 0.09 deg); fails on hospital S2 (11.4 m off, rejected) and on a second eco01 place (<= 3 % right pairs). Cause is in the FPFH front-end, not ROBIN/GNC (oracle counts, evaluator-only). KISS is non-deterministic (pair counts vary between identical runs) | notes 11, 12, 13 |
| Selected | **descriptor-free candidate generator**: gravity-levelled query, FFT cross-correlation over the whole map (144 yaws, 0.5 m grid), top 10 -> small_gicp refine -> frozen gate. KISS does not run in the candidate | ablation, same inputs, same gate: KISS+tiles 3 correct / 10 wrong hypotheses (all rejected) vs FFT 14 correct / 1 false reject on 15 own-map windows. The gain is candidate generation, not a looser gate | note 13 |
| Gate | `verify.decide`: GICP converged, support >= 0.70 within 0.15 m, matched RMSE <= 0.10, lead >= 0.10 over a distinct runner-up, independent confirmation on the later batch (0.20 m / 5 deg) | frozen; the band-support gate of note 14 stays diagnostic (never tuned on scored data) | notes 13, 14, 20 |

Why this and not another: the FFT search is deterministic for identical input (M, noise floor 0), has no feature-quality failure mode, and its cost is bounded (9-13 s worker, ~1 core, 0 % GPU). Not evaluated: any other family in the table above beyond reading; they remain reserves, not rejected on data.

## 3. Architecture

```
/livox/imu, /livox/lidar -> reloc_node (IMU stationarity gate, 2 s)
  -> cold FAST-LIO2 (owned child) -> /Odometry + /cloud_registered_body
  -> S2: 3 s stationary exposure -> lio_submap (levelled, fit 2/3 | check 1/3)
        -> search_v2 (FFT, killable child) -> register_v2 (small_gicp) -> verify.decide
     S2 without a pose -> S3_SUPERVISION_REQUIRED -> (grant by a person) -> S3: up to 30 s of motion, 2.8 m / 6 m stop -> same search + gate
  -> ACCEPTED -> map check of the propagated pose (fresh scans) -> /initialpose (cov 0.15 m / 5 deg)
  -> AMCL coherent with our pose -> second map check -> LOCALIZED
  -> supervisor: persistent map-check worker every ~0.5 s on AMCL's pose; permit needs a fresh passed check
  -> nav_gate (permit AND /cmd_vel) -> private socket -> driver cell (zero-fill, CAN dead-man; virtual bus only)
```
State machine (C, `reloc_core.TRANSITIONS`): `WAIT_SENSORS -> IMU_CONDITIONING -> LIO_STARTING -> S2_ACCUMULATING -> S2_SEARCHING -> ACCEPTED -> HANDOFF -> LOCALIZED`; S2 failure -> `S2_INSUFFICIENT -> S3_SUPERVISION_REQUIRED -> S3_ACCUMULATING -> S3_SEARCHING`. Terminal: `LOCALIZED, HANDOFF_FAILED, REJECTED, TIMEOUT, ERROR`.

- **S2**: robot still, 3 s exposure, no motion needed. **S3**: fallback when S2 abstains (e.g. ambiguous place), needs a supervisor's grant (`s3_auto_authorize=false` on the robot, true only in replay) and motion.
- **One deadline of 60 s** from the first sensor message to `ACCEPTED`; no restart between S2 and S3 (a restart would let each failed S2 buy 60 s more). A grant after ~38 s gives `TIMEOUT` by design. `LOCALIZED` (handoff, 5 s of hold) and cold boot are separate clocks, never added to or hidden in the 60 s. Note 20 section 7.
- **Acceptance**: only `verify.decide` (frozen). A pose is published only from `ACCEPTED` (one `init_pub.publish`, `reloc_node.py`, asserted; C, note 20 section 1) and only after a map check on fresh scans passes (note 21 B2).
- **Revocation**: `LOCALIZED` is revocable. Failed supervisor checks, stale checks (> 2.0 s), odometry stall (3 s), FAST-LIO2 exit, AMCL pose that no longer fits the map -> `LOCALIZATION_LOST`, permit false, gate output zero. **R-REV-SW = 2.5 s** (adopted 2026-10-11): from AMCL leaving 0.30 m / 8 deg to zero gate output. An internal *software* requirement valid for the replay conditions of notes 22-23. **It does not mean a guaranteed physical stop.**
- **Fail-closed**: no pose without acceptance; permit withheld in doubt; any silent/dead component (node, gate, outlet, driver) ends in zero output within 0.3-0.41 s (virtual CAN test). Not tested on a real base.

## 4. Candidate identity

| Item | Value |
|---|---|
| Name | `GR-S2S3-F2c` = `GR-S2S3-F2b` + `4211c6b` (supervision every 0.5 s, persistent worker, fresh-check permit) + `6cb779a` (tail of a check is never fewer than 3 scans); gate cell `8a88d16` |
| Method commit that produced the evidence | **`6cb779a`** (tree `reloc-f2l-20261011`, hashes in `results/22-f2-latency/f2l/candidate-identity.txt`) |
| Branch head after it | `22f9a80` adds `KEEP_FAILED_CHECKS` to `reloc_node.py`/`run_online.sh` (diagnosis, off unless the env var is set; used only in tree `f2m`) and `385718c` the `DOCKER_CPUS` knob (default 8) and `f3_summary.py`. So `reloc_node.py` at HEAD (`63dbc763...`) differs from the F2c evidence tree (`b1577c38...`) by those 5 diagnostic lines (C: `git diff 6cb779a HEAD -- tools`, 3 files, no decision path touched). Note 23 ran with them |
| Frozen predecessors | F1 `GR-S2S3-F1` = `c026a7b` (untouched); F2 `98926f7` (PARTIAL, superseded); F2b (note 21) |
| Images (NX only, outside git) | `socialtech:reloc-online` `sha256:a15d214c...` (method), `socialtech:mapper-verify` `sha256:88380b4b...` (evaluator), `socialtech:gate-can` (vcan test only; not bit-reproducible); `register_v2` aarch64 binary `adeb0ee9...` |
| Evidence level | `VALIDATED_IN_REPLAY` (F2c tree) / `..._ON_JETSON_NX` (F3). Not hardware |

The 50 software restarts of note 21 belong to **F2b**, not F2c (note 22 section 8 gives why they were not repeated). F1 evidence transfers to later trees by inference plus reproduced cases, not by re-measurement.

## 5. Results (what is measured, where, and for which version)

All development data: own-bag or same-day maps (best case), 2 buildings (+ eco03 cross-session), replay 1x, Orin NX, `jetson_clocks` on, idle board. **No reliability rate follows from them**: the runs are repeats of ~24 windows in ~12 places, 5 stationary blocks. Wilson bounds below assume an independence these data do not have. The protocol's independent set (>= 30 stationary blocks per scenario) is F5.

| Quantity | Value | Version | Source |
|---|---|---|---|
| Own-map correct accepts, FFT v2 | 14 of 15 windows (1 false reject: FAST-LIO2 cold-started during a 120 deg turn); XY median 0.023 / p95 0.041 / max 0.043 m; yaw max 1.58 deg | v2 search (= candidate search) | note 13 |
| Cross-session (eco03 queries on eco01 map) | 10 of 10 correct, XY median 0.010 / max 0.040 m, yaw max 0.44 deg (~8 distinct places) | v2 | note 13 session 2 |
| S2 stationary 3 s | 10 runs, 8 accepted in S2, 2 went to S3 (hospital z00), 10 correct; S2 error max 0.052 m / 0.51 deg (erratum in note 20 section 3) | `ab28151` -> F1 | notes 16, 20 |
| S3 | 21 accepts, 20 correct, 1 unscorable; XY max 0.084 m, yaw max 1.2 deg | `ab28151` -> F1 | note 16 |
| Negatives (wrong map, 8 m hole) | 88 non-perturbed over notes 13-14: 0 accepted (Wilson upper 4.2 %); 16/16 in `s2s3` note 16; 14/14 on F2c | v2, F1, F2c | notes 13, 14, 16, 22 |
| Near misses | 5 negatives reached support 0.695-0.756; only lead/confirmation blocked them. A 20 % dropout run produced a **wrong primary** that only the lead gate stopped (hospital w03). The 0.70 support alone does not separate right from wrong | v2 | note 13 |
| AMCL handoff (isolated, 3 runs) | converged at 0 s, XY median 0.031-0.042 m, p95 <= 0.082 m, 0 losses | v2 | note 13 |
| 50 restarts, fresh container each | 41 `LOCALIZED` (82 %, Wilson 69-90 %); 0 wrong / 0 pose without acceptance / 0 permit without correct pose (Wilson upper 7.1 %); 9 safe rejections (6 `s3_budget_insufficient`, 1 `imu_not_stationary`, 2 `HANDOFF_FAILED(amcl_not_confirmed)` when the platform moves at hand-over); AMCL error 2 s after `LOCALIZED` median 0.036 m, max 0.10 m | **F2b** | note 21 B10 |
| Fault matrix | 56 valid cases: 51 conform, 4 with a wrong prediction of mine and correct behaviour, 1 orphan-process failure not reproduced (`h35`); 0 wrong `/initialpose` after the map-check fix (h27: a pose published 3.5 m / 70 deg off in F2 `98926f7`) | F2b | note 21 |
| Revocation latency after `LOCALIZED` | 0-1.46 s over 23 revoked runs (13 at 0), 36 revocations incl. later trees; 0 above 2.5 s (Wilson upper ~10 %). Quota 2 cores: 0.01-0.65 s. Blind travel at 0.34 m/s: 0.50 m (1.46 s), 0.85 m (2.5 s) | F2c | notes 22, 23 |
| Time, S2 / S3 acceptance (clock from first sensor message) | S2 18.80-18.95 s (N=3), S3 41.81-41.96 s (N=3); all 18 clean attempts of the F2c tree: median 18.9, p95 36.8, max 42.0 s. Margins to 60 s: 41 s / 18 s. Noise floor 0.15-0.29 s. `LOCALIZED` at 25.1 s (S2) / 48.4 s (S3) | F2c | note 23 sections 3-4 |
| Resources, 99 runs + 14 new | RAM of the system max 2200 MB, swap constant, GPU 0 % (no CUDA), Tj max 67.2 C, VDD_IN peak 14.05 W, <= 3.1 of 8 cores (p95); `search_v2` 103 % of a core while searching (PSS 62 MB), `reloc_node` 31 % (PSS 101 MB, peak 288 MB in S3 hospital runs), FAST-LIO2 17 % (39 MB) | F2c | note 23 section 2 |
| CPU quota | 4 cores: no change; 2 cores: accept +2.1-2.4 s (S3 44.25 s), same outcome | F2c | note 23 section 5 |
| Memory trend | node +0.22 MB/min, FAST-LIO2 +0.98 MB/min over 8 min (not a proof of no leak beyond 8 min; longest bag 634 s) | F2c | notes 22, 23 |

**Not measured / not supported by data**: a success rate on independent data; cold boot (power -> stack; only `systemd-analyze` 17.1 s as a component); Orin Nano; Nav2 or AI layer running next to the stack; sensor facing down (the final mount); a live Livox driver (stamp base of `/initialpose`, stream loss); the hand-over while moving (2 of 50 false rejects when the platform moves, with AMCL's stamp handling NV); physical stop, base firmware timeout, real CAN.

**Cases that are not successes** (kept, never counted as such): 2 of the 3 S3 repeats of hospital z00 in F3 (`b1`, `b3`) and the first F1 audit repeat are `unscorable_reference_gap` (reference not bracketed within 0.2 s; wide-bracket error 0.066 m / 0.25 deg is reported separately, not as a rule-based success); `h27-y1` published a pose scored only with the 0.5 s bracket (0.07 m / 5.1 deg); `h25` invalidated by a 26.9 s storage stall and rerun; invalid or superseded runs of earlier notes are listed in their own sections (16 section 9, 13, 21, 22). Mistakes of mine are kept in notes 16 section 9, 20 section 9 and 21.

## 6. Reproducibility

**Environment (C/M)**: Orin NX 16 GB, JetPack 7.2.1, ROS 2 Jazzy in Docker (CycloneDDS, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, one random domain 100-199 per run), `jetson_clocks` active. Python tests: numpy 1.21.5, scipy 1.8.0.
**Images** (documented; rebuild not re-executed in F7):
- `socialtech:robot-verify` (FAST-LIO2, Nav2, planar_odom; a local image of the NX, its build recipe is **not in this branch**: it comes with the PR #12/#13 stack, **NV** here); method image `tools/global_reloc_3d/docker/Dockerfile.online` (`FROM socialtech:robot-verify` + `python3-scipy`). `Dockerfile.gate-can` adds `iproute2 can-utils` for the virtual-bus test. Evaluator image `socialtech:mapper-verify` (PR #13).
- `libsmall_gicp.so` and `register_v2`: **open gap O8, only partly closed.** Recipe reconstructed from notes 11-12 (**documented, not re-run in F7; the exact compiler line was kept only in a command transcript, not in the repo**): in `socialtech:robot-nav` build small_gicp `fa0cfc9` (`cmake -DBUILD_WITH_TBB=ON -DBUILD_WITH_OPENMP=OFF`, note 11) and the KISS-Matcher core `8b58b11` (`-DUSE_SYSTEM_EIGEN3=ON -DUSE_SYSTEM_TBB=ON`, target `kiss_matcher_core`); then `g++ -std=c++17 -O2 -fopenmp tools/global_reloc_3d/register_v2.cpp` with the KISS/ROBIN/PMC archives and includes of note 12 (line 70) and `-lsmall_gicp -lflann_cpp -llz4 -ltbb`. `register_v2.cpp` still includes KISS headers although the candidate uses it in GICP-only `--init` mode. The binary of the evidence has sha256 `adeb0ee9...`; a rebuild will not match it bit for bit (I). Check: `REGISTER_BINARY=<bin> python3 -B tools/global_reloc_3d/test_register_synthetic.py` (passed on aarch64 in note 12, skipped on the laptop).
- Images are local to one board. An exported image or a CI recipe does not exist. (**NV** that a clean checkout rebuilds to the same behaviour.)

**Data** (NX, outside git): bags `/home/jetson/rosbags/{custom,benchmarks}`; maps `map_out/eco01_1x`, `maps/eco03_1x`, hospital map from `make_map.sh` (`socialtech:mapper-verify`, PR #13 worktree `feat/make-map`); windows `windows/*.csv` in each run root; run roots `reloc-online-20261009` (note 15), `reloc-f1-audit-20261010`, `reloc-f2h-20261011`, `reloc-f2l-20261011` (final tree), `reloc-f3-20261011`. Per-run evidence (`metrics.json`, `evaluation.json`, `launch.log`, `cost.csv`, `tegrastats.log`) stays there; tables, hashes and small logs are in `research/global-reloc/results/`.

**Commands (documented in the notes; executed there on the NX, not in F7)**
```
cd tools/global_reloc_3d && for t in test_*.py; do python3 -B $t || break; done      # 12 files; laptop needs numpy + scipy
R=<root> bash tools/global_reloc_3d/batch_s2s3.sh <cases.csv> <tag>                     # S2 / S3 / negatives (cases-f1.csv, cases-f1b.csv, cases-sn.csv)
R=<root> bash tools/global_reloc_3d/batch_faults_f2.sh <tag> <case-prefix>               # fault matrix of note 21
R=<root> bash tools/global_reloc_3d/batch_latency.sh <tag> [case-prefix]                 # revocation latency, note 22
R=<root> RUN_ENV="KEEP_PLAYING=1 PLAY_S=92 STOP_AT=85" bash tools/global_reloc_3d/batch_restarts.sh cases-restarts50.csv <tag> <i> <i>
DOCKER_CPUS=2 R=<root> bash tools/global_reloc_3d/batch_s2s3.sh cases-f3.csv <tag>       # CPU quota, note 23
bash tools/global_reloc_3d/gate_cell_test.sh <out>                                      # virtual CAN only (vcan0 inside a private-network cell)
python3 tools/global_reloc_3d/f3_summary.py stages|teg|cost <RUNS_DIR> [REGEX]          # aggregation
```
Single run (`run_online.sh`): env `NAME BAG MAPYAML MAPXYZ OFFSET TAG`, optional `MODE H DOCKER_CPUS DOMAIN ...`; output `$R/runs/<NAME>-<TAG>` (new dir each time, docker writes as root). It mounts the map YAML and image only and `online_inner.sh` aborts if any `alidarState*` is visible: **the method container never sees the reference**.
Diagnosis of a failed run: `inner.log`, `launch.log` and `work/attempt*/metrics.json` (state, reason, per-stage times), then `evaluation.json` (class: `correct_accept`, `wrong_accept`, `correct_reject`, `false_reject`, `unscorable_reference_gap`); `io_stall_audit.py <RUNS>` for NVMe stalls; `permit_exposure.py` for permit/gate timelines; `analyze_online.py`, `summarize_s2s3.py` for tables.

**Verified in F7 (laptop, static, no ROS, no containers)**: the 12 test files of `tools/global_reloc_3d` pass (numpy 1.21.5, scipy 1.8.0); relative markdown links of notes 01-24 resolve; the diff against the base contains no key, token, password, IP or host name (`AGENTS.md` mentions the words only as rules). **Not re-executed in F7**: every batch above, the image builds, the compiler line. A reviewer who wants proof of reproduction on another Jetson must run note 20 section 4 (`batch_s2s3.sh cases-f1.csv`) on a clean checkout: that is the closing criterion of F7 in note 19 and **it is not met by this delivery**.

## 7. Limits and safety

| Topic | Statement |
|---|---|
| Replay vs physical | Everything is replay of recorded bags at 1x. A replay is not a live sensor; a software restart is not a cold boot; a virtual CAN bus is not a base. No robot moved, host `can0` was never used (down, counters 0) |
| Gate | `verify.decide` is a geometric rule; support alone does not separate right from wrong (near misses 0.695-0.756). A pose wrong but consistent with the map (a place that looks the same, a corridor along-axis error) can pass the checks for about 1 s (`k12`) and could last longer: not bounded by data |
| Revocation | 0-1.46 s measured, R-REV-SW 2.5 s. Violent divergences move the pose 1-1.6 m in 0.4-0.75 s (`k23`, `k24`): no check period makes that small; it needs an independent motion sensor (wheel odometry) or a stricter rule on held-out data. 0.5-0.85 m of blind travel at 0.34 m/s needs Narcís's judgement against the obstacle layer, which does not exist in this scope |
| Physical stop | **Not demonstrated.** `tracer_base`/ugv_sdk have no command timeout (C, pinned sources); the gate cell and CAN dead-man exist but ran on `vcan` only; running `can_deadman.py` on a real bus is refused by the script and is Narcís's decision |
| Availability | spurious failed checks stop the output ~0.6 s now and then (3 in 1036 checks); 2 of 50 false rejects when the platform moves at hand-over (stamp of `/initialpose` ~1 s old, AMCL applies it as current: **NV** on hardware); RMSE margin thin on good data (0.079-0.096 vs 0.10) |
| Known weak cases | hospital z00 (S2 ambiguous, S3 resolves it); hospital c02 and z02 (`s3_budget_insufficient`, IMU never still); FAST-LIO2 start-up hang ~1 in 100 starts (ends `ERROR`, no pose); input starved by a storage stall ends `odometry_stalled` (intended) |
| Mount | height 0.53 m, identity extrinsic, covariance 0.15 m / 5 deg are PROVISIONAL (note 17); sensor faces up in all data; the final mount needs F4 and a new map |
| Memory beyond 8 min, Nano, Nav2/AI concurrency | not measured |
| **NVMe incident (open, no root cause)** | 145 kernel `nvme0 I/O timeout, completion polled` since 2026-10-05 (journal, no root); rate rose with the long campaigns; no EXT4/I-O/AER error; 8 of 113 runs overlapped a timeout, one with a real sampler hole (`h25`, invalidated and rerun); none of the 14 F3 runs. **SMART unread**: `sudo nvme smart-log /dev/nvme0` and `sudo nvme error-log /dev/nvme0` (read-only) are the missing evidence and need the human. Cause (load, firmware, power state, media) is **NV** |
| Findings not fixed (note 20, O1-O9) | no start-up watchdog if both streams are absent (O1; a start-up watchdog was added in F2, note 21), O5 `stop_lio` can block 5-17 s after a terminal state, O6 replay S3 timing not representative of supervised use, O7 evaluator refuses a bracket > 0.20 s, O9 band gate not applied. None invalidates the candidate in replay |

## 8. How to resume F4 / F5 / F6 (all `BLOCKED_PHYSICAL`; none was started)

| Phase | What must be done on the robot | Closes with | Reference |
|---|---|---|---|
| F4 | Narcís defines `base_footprint`; tape/CAD measure sensor height, sensor-to-base extrinsic with uncertainty; data checks D1-D4; decide the IMU sample-count rule (95 % written, 99.3-99.5 % effective); remount (looking down) and a new map | table of note 17 section C filled with who/how; `mount.status` leaves `PROVISIONAL_not_measured`; new candidate name | note 17, decisions 2-3 of note 20 |
| F5 | Record >= 30 independent stationary blocks for S2 and 30 for S3, negatives outside the map and against another building's map; build maps/references from a different session than the queries; **freeze commit, image IDs and evaluator before opening Q-eval; one run of the whole set** (a change after it consumes the set) | per-cell counts with Wilson 95 % bounds, blocks and starts separately, med/p95/max error and time, an overlay for each wrong accept | note 18 section 6, protocol 02 |
| F6 | Preconditions: F4 done, motors disabled, a person at the chair. H1.1-H1.14 (live sensors, robot still: live stamp base of `/initialpose`, stream loss, cold boot, supervisor reaction against the 38 s grant limit) and H2.1-H2.5 (S3 with a person moving the chair); expected outcome written before each test; a failed row stays failed | `VALIDATED_ON_HARDWARE_STATIONARY` / `VALIDATED_WITH_SUPERVISED_MOTION` | note 18 section 7 |
Also pending with Narcís (not phases): firmware command timeout, physical CAN and the dead-man on a real bus, a decision on the blind travel, the 0.5 s vs 1 s check period, whether the Nano is a deployment option (it would have to be measured), and the SMART commands above.

## 9. Summary for Narcís

- **Achieved**: a node that, at boot and without an operator, finds the pose in the 3D map (levelled FFT search -> small_gicp -> frozen gate -> map check -> `/initialpose` -> AMCL), with an S2 (still, 3 s) / S3 (supervised motion) split under one 60 s clock, fail-closed supervision and a gate cell that zeroes the base output. The task's KISS-Matcher hypothesis was built and tested: it works only when descriptors are discriminative (fails in hospital) and was replaced by the FFT search.
- **Measured (replay, NX)**: pose error ~0.01-0.07 m at hand-over, AMCL median 0.03-0.04 m after; 0 wrong acceptances in ~88 negatives and in 50 restarts (bounds in section 5); S2 accepts at ~19 s, S3 at ~42 s (margins 41 s / 18 s); revocation 0-1.46 s; 1.3 of 8 cores, 2.2 GB, 0 % GPU, 14 W peak.
- **Delivered version**: `GR-S2S3-F2c`, evidence commit `6cb779a`, branch head with two harness/diagnostic commits that do not alter the decision path (section 4).
- **Limits**: replay only; development data and best-case maps; no independent success rate; cold boot, Nano, Nav2 concurrency, down-facing mount, live driver, physical stop and the hand-over while moving not measured; NVMe incident open.
- **Next**: your decisions (section 8, last row), then F4 -> F5 -> F6 on the robot.
- **Please look at**: the near-miss and dropout cases (note 13) for the gate; the 0.5-0.85 m blind-travel judgement (note 22 section 6); `figures/` overlays; the hand-over-while-moving false rejects (note 21 B10); the four decisions of note 20 section 7 (not yet taken); and the open O8 gap in section 6 (the build line was not recorded in the repo).
