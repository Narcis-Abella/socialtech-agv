# Point 7 continuation: spatial generalization and failure diagnosis

Date: 2026-10-09. **V** = measured or read in pinned code/artifacts; **I** = inference; **NV-P** = pending. All runs here are DEVELOPMENT diagnostics. The same-run prior map contains observations from the query traversal. No new result is a formal generalization rate.

## Predetermined questions and controls (before new runs)

1. **Second place, frozen configuration.** The amended inventory (`/home/jetson/nav_out/global-reloc-stationarity-v3-20261007/inventory/windows.csv`) has only one eligible S2 3 s spatial block in eco01 and one in hospital. It has a different eco01 S3 start at sensor-header `1780999855.339199232`, approximately (-14.317,-4.335) m in evaluator-only map coordinates, about 8.7 m from the original S2 block (-22.916,-3.104) m. The method receives the sensor time and raw stream, never that reference position. **Hypothesis I:** a moving local cloud may yield a distinct correct accepted pose with the frozen 0.20 m input voxel, 0.30 m KISS voxel, complete prior + fixed 15/10 m tile grid and unchanged gate. **Falsifier:** no correct accepted pose under the original 60 s decision limit, even if the oracle pose would have high map support. Preserve a rejected result. The S3 prefix is reference-screened offline at 7.40 s, 0.84 m path and 0.79 m radius, but the method must stop on its *own* odometry/causal input; this screening cannot become a runtime stop condition. Report the exact input prefix used and its local odometry path.
2. **Hospital correspondence failure.** The original true-location tile was searched but KISS returned only 7 initial / 3 translation inlier pairs; oracle-pose held-out map support was 0.924. **Hypothesis I:** too few correct descriptor pairs survive, despite geometric overlap. Measure source/target density and extent, KISS feature counts/correspondence distribution where available, and score before/after GICP on the same frozen input. A correct valid KISS pose at the true location would reject this hypothesis.
3. **Gate separation.** The current code's nearest-neighbor checking uses the entire ceiling-inclusive map and later uses the checking batch as the fit input for its confirmation registration. The protocol calls for a calibrated 0.10–1.80 m verification band and independently retained checking geometry. **Hypothesis I:** the current acceptances are valid under the *implemented exploratory* gate but cannot be claimed compliant with the final protocol. Do not change the acceptance threshold or recast old results. Quantify diagnostics separately; any new gate requires a prospective amendment and independent calibration.
4. **Negative control.** Retain the original eco01-query/hospital-map rejection as a cross-map diagnostic. For any new wrong-map run, abstention must be measured from the method output before any reference inspection. Neither apparent registration convergence nor a low matched residual is enough to accept.

Acceptance remains `XY <= 0.30 m` and `yaw <= 8°` against the post-run evaluator reference; `wrong_accept` is any method acceptance outside either bound. Search: K=5, deadline 60 s, batch median separation >=0.5 s and span >=1 s, temporal agreement <=0.20 m / 5°. No threshold is selected from this note's outcomes. Original archives under `/home/jetson/work/global-reloc-3d-20261009/` remain untouched; new output paths are versioned.

## Session 2026-10-09 (Claude Code): what the code does, where it fails, what was changed

Recovered state: HEAD `c34c704`, this note's preregistration and `diagnose_geometry.py` untracked (Codex). Codex's interrupted run `generalization-v1/eco01-s3-b2-search-7s-v1` (hypothesis 1 above, frozen KISS config, 7 s S3 prefix) ended `reject_geometry`: 1/15 targets KISS-valid+GICP-converged, best support 0.101; its later-batch confirmation had 207 matches, 0 translation inliers. Evaluator: selected hypothesis 16.2 m / 91.8° off, tilted ~19° (**V**). Hypothesis 1 is falsified for the frozen KISS configuration (correct rejection, no wrong accept). All outputs below are under `/home/jetson/work/global-reloc-3d-20261009/claude-v1/`.

### KISS-Matcher as built (`8b58b11`, read in code, **V**)

| Stage | What it does (file) | Consequence for us |
|---|---|---|
| Input | Voxel grid at `voxel` (0.30 m) on both clouds (`KISSMatcher.cpp:51-61`) | Map 55k → ~20k keypoints; query 6k → ~2k |
| Keypoints | **Every** voxel with a valid normal is a keypoint; no detector (`FasterPFH.cpp:123-176`). Linearity filter off (`thr_linearity=1.0`, `KISSMatcher.hpp:47`) | Planar floor/ceiling/wall voxels dominate the descriptor set |
| Descriptor | FPFH, normal radius 3×voxel (0.9 m), FPFH radius 5×voxel (1.5 m) (`KISSMatcher.hpp:72-100`) | Rotation-invariant local shape only; no global context |
| Matching | NN in descriptor space, smaller cloud into larger, ratio test 0.9 (on by default), abs. distance < 30, mutual cross-check, ≤5000 pairs (`ROBINMatching.cpp:57-140`) | The ratio test compares against the *whole target*: more target = more near-duplicates = fewer survivors. This is why tiles help |
| Pruning | ROBIN pairwise-distance compatibility graph, max k-core, noise bound = voxel (`ROBINMatching.cpp:280-330`) | Needs a consistent set among the pairs; cannot create correct pairs |
| Solver | GNC-TLS rotation on chained TIMs, or Quatro (yaw-only SO(2)) if `use_quatro`; adaptive-voting translation (`GncSolver.cpp:437-520`) | — |
| Validity | `valid = translation_inliers non-empty` (`GncSolver.cpp:496-500`) | `valid` is not a confidence. 1 inlier is "valid" |
| Confidence API | Only inlier counts; no covariance. The official loop closure adds `num_inliers_threshold` and GICP overlap ≥55 % (`ros/src/slam/loop_closure.cpp:194-248`) | Our external gate is required, not optional |
| Determinism | TBB parallel reduce + FLANN: pair counts change between identical runs (eco tile: 69 vs 71 pruned; S3 full map: 79 vs 71 vs 92) (**V**, measured) | Repeats needed; record seeds is impossible (no seed) |
| Quirk | Quatro noise bound is a function-`static` (`GncSolver.cpp` QuatroSolver) — fixed by the first call in a process | Harmless with one call per process (our callers) |

small_gicp `fa0cfc9` (**V**, code): `converged` means the last Gauss-Newton/LM step was < 0.1° and < 1 mm (`registration_helper.hpp:31-32`, `optimizer.hpp:52,123`), 20 iterations max, hard gating at `max_correspondence_distance` (1.0 m), no robust kernel. It converges into wrong minima as readily as right ones; threads ≥2 add minor non-determinism (`registration_helper.hpp` note). It is a refiner, never evidence.

### Diagnosis: the failure is in the descriptor front-end, not in ROBIN/GNC (evaluator-only, **V**)

`register_v2.cpp` (superset of the frozen `register.cpp`, which stays untouched) dumps the cross-checked and the pruned pairs. `diagnose_correspondences.py` (evaluator-only) counts the pairs within 1.0 m of the reference-implied pose. Same frozen inputs as the point-7 runs (`probe-v2`, `probe-v3-scale`).

| Query / target | Config | Cross-checked pairs correct | After ROBIN correct | Trans. inliers |
|---|---|---:|---:|---:|
| eco01 S2 w01 / true tile | default | 99/232 (43 %) | 71/71 | 66 |
| eco01 S2 w01 / full map | default | 105/375 (28 %) | 79/80 | 61 |
| hospital S2 w01 / true tile `tile_6_1` | default | 0/42 | 0/7 | 3 |
| hospital S2 w01 / full map | default | 1/207 | 0/9 | 6 |
| hospital, no ratio test | full / tile | 2/471, 1/130 | 0/46, 1/58 | 3, 0 |
| eco01 S3 b2 / full map | default | 12/373 (3 %) | 7/92 | 4 |
| FPFH gain 8, 12; voxel 0.5 × gain 5, 8 | hospital full + tile | 0–1 per run | 0–1 | 0–3 |
| same scale sweep | eco01 S2 full (control) | 0/11 to 37/118 | 0–32 | 3–16 (worse than default) |

**Interpretation (I, strong):** when ≥~25 % of the cross-checked pairs are right, ROBIN isolates them almost perfectly (eco01 S2). In hospital and in the second eco01 place, ≤3 % are right and no back-end setting can recover. FPFH scale does not help (plus Codex's 0.20/0.40 voxel runs). Rejected hypotheses: "the true tile was not searched" (it was), "ROBIN prunes the right pairs" (there are none to prune), "missing map geometry" (oracle support 0.92). Not proven: *why* FPFH is undiscriminating here. Candidates: a single-viewpoint, upward-looking query vs a map accumulated from many viewpoints, planar-dominated keypoints with no detector, and repeated corridor/ceiling structure. That is not needed for the decision below.

### Change 1: gravity levelling of the query (**V**, implemented, tested)

`lio_submap.py --imu-bag`: mean `/livox/imu` specific force over the window, each sample rotated into the first local frame by FAST-LIO2's own odometry, gives "up"; the smallest rotation to +z (no yaw added) levels fit/check (`T_level_local` in the manifest, frame `local_first_imu_levelled`; `evaluate.py` composes it). Removed tilt: 0.56° (eco01 S2), 0.96° (hospital), 0.42° (eco01 S3); median sample deviation 0.06–0.69°. With levelled query and map, roll/pitch are known, so only yaw, x, y, z remain. A KISS 3-DoF hypothesis tilted by 19° (Codex S3) is physically impossible. That signal is recorded (`tilt_deg`) but is **not** a gate yet: adding a gate needs a prospective amendment.

### Change 2: descriptor-free candidate generator (`correlative_search.py`, `search_v2.py`)

Exhaustive search, no prior: for each yaw (2.5° step) the levelled query (≤15 m horizontal radius) is voxelized at 0.5 m and FFT-cross-correlated with a Gaussian likelihood field (σ 0.5 m) of the *whole* map. Every x, y, z in the map box is scored. Per-yaw NMS, then global NMS (1 m / 10°) → top 10. Each candidate goes through small_gicp (GICP-only `--init` mode of `register_v2`, init from the search), is scored on the check batch, and passes the **unchanged** `verify.decide()` gate. The confirmation is an independent correlative search of the later batch, with its best chosen on the fit batch. KISS is not removed: it remains the frozen baseline and a possible second generator. Justification: the evidence above (descriptor recall ~0 where it fails), and the search is deterministic with explicit uniqueness evidence (all of x, y, yaw scored).

### Results v2 (DEVELOPMENT; same-run maps; S3 windows not certified stationary at start)

**First three queries** (`search/<name>-v1`, frozen SHA-256 in `search/frozen-v1.sha256`): eco01 S2 w01 3 s correct_accept 0.0096 m / 0.10°; **hospital S2 w01 correct_accept 0.0079 m / 0.71°** (lead 0.25; KISS had failed here); **eco01 S3 b2 7 s correct_accept 0.0183 m / 0.43°** (lead 0.14; frozen KISS rejected). The correlative init was off by up to 1.13 m / 9.8° and GICP closed it.

**Window rule (fixed before running, administrative, evaluator-only):** `select_windows.py`, reference-trajectory points ≥5 m apart (10 m yields only 3 per bag: areas are 28×10 m and 27×11 m), 7 s, fresh FAST-LIO2 with a 3 s pre-roll, online S3 stop from own odometry at 2.8 m radius / 6 m path (`lio_submap.py --max-radius`). Windows: `results/{eco01_1x,hospital_02_r05b}-s5.csv`. 5 m-separated places share part of the scene: they are **distinct, not independent** samples. **Protocol deviation (§S3):** the pre-roll is not certified as near-stationary, and pre-roll motion is not counted in the S3 path. So these are `S3_dev_uncertified`, not compliant S3 attempts.

Per window: own map; cross map (the other bag's map, so any acceptance = wrong_accept); hole map (all prior points within 8 m XY of the window's reference position removed by evaluator-side `make_hole_map.py`; the method sees only the result). Runner `batch_generalization.sh`, evaluation after freeze `evaluate_batch.sh`, table `summarize_batch.py` → `results/13-v2-batch-*.csv`.

| Variant | N | correct_accept | reject (wrong hyp.) | false_reject | wrong_accept (Wilson 95 % upper) |
|---|---:|---:|---:|---:|---:|
| own map | 15 (9 eco01, 6 hospital) | **14** | 1 (eco01 w08) | — | 0 (0.20) |
| cross map | 15 | — | 15 correct_reject | — | 0 (0.20) |
| hole 8 m | 15 | — | 15 reject | — | 0 (0.20) |
| all negatives + own | 45 | | | | **0 (0.079)** |

Correct accepts (N=14): XY median 0.023 / p95 0.041 / max 0.043 m; yaw 0.11 / 0.90 / 1.58°. Support lead min 0.124. Search wall time 9.7 / 12.9 / 13.3 s (all 45 runs: 9.3 / 19.3 / 23.4 s), CPU 13.7 / 17.3 / 18.1 s, orchestrator max RSS 87 / 91 / 91 MiB (RSS, not PSS). Query span 6.9 s at 1x. Integrated boot→pose latency ≈ exposure + submap + search: **NV-P** (submap build not timed as an online stage).

**eco01 w08 failure (V, evaluator):** FAST-LIO2 cold-started while turning about 120°. Its odometry path is 4.98 m against 2.44 m in the reference, so the query cloud is corrupted. All 10 candidates share a 138° yaw error and support is ≤0.36. *Correction 2026-10-09 (audit):* the protocol class is **false_reject** (a positive with a representable pose that was not accepted), not a "correct rejection": the method's behavior was safe, but it did not localize. The window is also **ineligible** under §S3: there was no certified stationary start, and that is precisely the mechanism (FAST-LIO2 initializes its gyro bias as the mean gyro of the first samples, `IMU_Processing.hpp:156-196`, so a turning start corrupts it; **V** code, **I** that it caused w08). `summarize_batch.py` now labels every positive non-acceptance `false_reject`, with `primary_within_limits` as a separate column.

**Near misses (V, the reason the gate must improve):** cross map eco01 w05/w06/w07 primaries reached all-point support 0.76/0.71/0.70 with RMSE ≤0.10; only the lead gate (0.005–0.016) blocked them. Hole hospital w03/w05 reached 0.699/0.695 (lead 0.075/0.001). The 0.70 all-point support threshold alone does **not** separate right from wrong.

**Support breakdown (truth-free, `support_breakdown.py`, `breakdown-v1/`):** correct primaries have 0.10–1.80 m band support 0.97–1.00 (N=14). Cross-map primaries have ≤0.65 (N=15). Horizontal-normal points (floor/ceiling) have support 0.75–0.98 even for wrong hypotheses: they inflate all-point support and do not discriminate. **I:** this backs protocol §145 (band-limited verification). **Proposal for Narcís, not applied:** gate on band support (map-frame 0.10–1.80 m, map z = 0 floor) and optionally on vertical-normal support, with a threshold calibrated on data not used here. Today's numbers would put the separation between 0.65 and 0.97, but choosing it from these runs would be fitting on the scored data.

**Ablation, same inputs, same gate (`search/kiss-*`):** frozen KISS + tiles (v1) on the 15 own windows gives 3 correct_accept, 2 correct-but-unconfirmed, 10 wrong hypotheses (all rejected). On the 15 cross: 15 correct_reject. Correlative v2 gives 14 / 0 / 1. **V:** the gain comes from candidate generation, not from a looser gate (same `verify.decide`).

**Noise floor:** v2 identical-input repeats (eco01 w03 own, hospital w01 own, eco01 w06 cross): same decision, same primary, Δpose 0, Δsupport 0 (3/3). Deterministic within the observed resolution. KISS is not (pair counts vary run to run).

**Truth isolation (V):** method containers mount only the LIO capture, the raw bag (IMU), the prepared/hole map and code, with `--network none`. References are mounted only in evaluator containers after `sha256sum` of every `search.json` (`search/frozen-*.sha256`). Hole maps and window times are administrative inputs produced from the reference before the run; the method receives neither the positions nor the reference.

**Invalid runs (kept, not tabulated):** `*-hole-v1`: the batch passed a negative `--y` and a CRLF field to `make_hole_map.py`, so the hole map was never written and Docker created root-owned empty directories (`holes/*.xyz/`, `inputs|search/*-hole-v1`; not deleted). Rerun as `*-hole-v1h`. `probe-v1` correspondence dump: wrong index un-swapping (the library getter already un-swaps, `ROBINMatching.hpp:58-69`), so its oracle counts are void; `probe-v2` is the corrected run.

**Limits:** two buildings, same-run maps (the map contains the query's own observations: best case); 15 distinct but correlated places; uncertified S3 conditioning; all-point gate as implemented, not the protocol band gate; no AMCL handoff yet; RSS not PSS; Nano not measured. Figure: [figures/v2-band-overlays.png](figures/v2-band-overlays.png).

### Next
1. Narcís: decide the band-support gate amendment, and where its threshold is calibrated (held-out bag or independent session).
2. Cross-session test: map eco03 with `make_map.sh` and query eco03 windows against the eco01 map. This removes the same-run optimism; it needs an evaluator-only eco03→eco01 frame link.
3. Protocol-compliant S3: start windows at certified near-stationary moments; count pre-roll motion.
4. Handoff: `T_map_odom` from the accepted pose → `/initialpose` with covariance → AMCL convergence on replay (B7).
5. Cost: PSS/tegrastats; reduce GICP candidates (10 → 5) and reuse the field across batches; Nano.

## Session 2 (2026-10-09): cross-session validation, safety, S3, handoff

### Audit of the v2 method (V, code)

- **Architecture of the method that wins:** FAST-LIO2 body clouds + `/Odometry` → `lio_submap.py` (join, IMU levelling, optional online S3 stop, 0.20 m voxel, fit = first 2/3, check = last 1/3) → `search_v2.py`: `correlative_search.py` (whole-map likelihood field σ 0.5 m at 0.5 m; query ≤15 m radius; 144 yaws × FFT over x, y, z; top 10 after NMS) → `register_v2 --init` = **small_gicp GICP only** (1.0 m max correspondence, 0.30 m downsampling, 2 threads) → `verify.score` on the other batch → `verify.decide` (unchanged). **KISS-Matcher does not run in v2**: the FFT search replaces it as the candidate generator (`register_v2` skips `estimate()` when `--init` is given; `translation_inliers` is written as 0). KISS remains in v1 (`search_tiles.py` + frozen `register`), and in v2 work only as the evaluator-side map link below.
- **Separation:** method containers: `--network none`, mounts = LIO capture, raw bag (IMU), map, code, `libsmall_gicp`; no reference. Evaluator containers mount the reference after `sha256sum` of every `search.json`. Admin inputs that use the reference before a run: window start times (`select_windows.py`) and hole maps; neither carries a pose into the method. Caveat (**I**): the primary is chosen by check-batch support among 10 candidates and the same check support is then gated (≥0.70): a mild max-over-candidates optimism, as in v1. The fit/check split itself is respected (GICP never sees check).
- **Docs vs data:** numbers in the v2 tables were re-derived from `results/13-v2-batch-*.csv` (counts, quantiles, Wilson bounds); the one inconsistency was the w08 class (corrected above).

### Evaluator-only frame link eco03 → eco01 (`map_link.py`, `eco03/link-v1/link.json`)

eco03 map made with `make_map.sh` (PR #13 worktree `~/work/make-map` at `8c1f061`, image `socialtech:mapper-verify`): 0 resets, 95.7 m path, sensor height 0.531 m. The PGM, `world.ply` and reference come from one run in `/home/jetson/work/maps/eco03_1x/` (original bag untouched). The link is **map-to-map with the whole maps**: KISS-Matcher (voxel 0.5; 981 cross-checked pairs, 372 translation inliers) then GICP. It is independent of every query window and of the FFT generator. Result: `T_eco01map_eco03map` ≈ identity, −0.250 m in x, +0.015 m in y, 0.21° of yaw, tilt 0.013° (both maps are wall-aligned and levelled by `ply_to_map`; **I:** both sessions start within about 0.25 m of each other). 99.3 % of eco03's 0.10–1.80 m points have eco01 geometry within 0.5 m; in that overlap the median distance is 0.074 m and p95 0.137 m. 10 GICP restarts (±0.5 m, ±3°): spread ≤3.5 mm / 0.005°. The two eco03 halves registered separately disagree by **1.1 cm / 0.03°**. **I:** link uncertainty is ~1–2 cm, small against 0.30 m / 8°; both Voxel-SLAM references keep their own unmeasured error. `evaluate.py --link` composes it.

### IMU readiness (`imu_ready.py`) and certified S3 starts (`ready_windows.py`)

The note 06 IMU stationarity test, unchanged (calibrated on static bags, no evaluation data), is applied causally to 2 s windows on the robot's own IMU. Stationary episodes: eco03 4 (longest stationary span 3.4 s), eco01 6, hospital 11. A certified S3 attempt starts FAST-LIO2 cold on the **last** certified 2 s before motion and accumulates up to 30 s with the online 2.8 m / 6 m stop. Truth-free; reference used only to report XY and cut holes.
