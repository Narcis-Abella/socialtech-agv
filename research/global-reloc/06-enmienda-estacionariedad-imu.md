# IMU stationarity amendment before point 6

Date: 2026-10-07. **V** = measured in named bags/artifacts or pinned source; **I** = inference/delegated criterion; **NV-P** = unresolved. This is a prospective amendment made **before AMCL or any relocalization candidate was run**. It does not rewrite point-5 commit `1c8aad6055a5001d5cb8d04ab201d20081bdd558`, the zero-window result under the original continuity rule in [04](04-inventario-ventanas.md), or the zero-window result under the continuity-only amendment in [05](05-enmienda-continuidad-temporal.md).

## Provenance and independent calibration

**V, source/history:** the `raw ||gyro|| p95 <= 0.02 rad/s` screen first appears in [02](02-protocolo-evaluacion.md), line 105 of point-4 commit `31cd64ff0034072a76699546ab47281a47359871`, explicitly labelled **I — proposed ... (unvalidated)**. It is absent from AGENTS.md and is neither a direct Narcís requirement nor a measured sensor specification. [03](03-generador-posiciones.md) carried it into the generator design; `window_inventory.py` implemented it in original point-5 commit `1c8aad...`. Classification: **C, delegated/inferred proposed criterion, subsequently implemented**. The current request calls for an independent sensor check, not post hoc tuning on evaluation-window counts.

**V, read-only static records:** the existing NX directory `/home/jetson/rosbags/imu_static/` contains the two IMU-only MCAP bags below. Their metadata identify `/livox/imu` as `sensor_msgs/msg/Imu`, frame `livox_frame`; both driver logs identify Livox Driver2 v1.2.8, the same device handle and MID360 configuration. The bag header clocks are strictly monotonic. Their metadata SHA-256 are `5464b03d30f36886c12cf0413a2302915e67193db0b3f204b7ca58538016e17b` and `997b2242964d5d9db2e469b09082399317c1429458f685c803224a3c4cce6701`; full multi-GB MCAP hashes were not computed. The names and operator's description identify these as stationary acquisitions; low, stable accelerometer variance and gyro medians support that description. **NV-P:** no independent video/encoder/rigid-fixture trace proves absolutely zero motion or that the mount/temperature matched the earlier mapping runs. Both are therefore `USABLE_WITH_CAVEATS`, not an absolute motion ground truth.

| Static bag under `imu_static/` | IMU messages / sensor duration | Median header delta / mean rate | Raw gyro norm p95 | Robust bias `median(gx,gy,gz)` rad/s | Fixed-bias residual norm p95 / p99 | Classification |
|---|---:|---:|---:|---|---:|---|
| `imu_20261002_001722` | 11,864 / 59.314 s | 4.945479 ms / 200.00 Hz | 0.024573 | (+0.018046, −0.000338, +0.012187) | 0.004643 / 0.005535 | `USABLE_WITH_CAVEATS` |
| `imu_20261002_001929` | 8,614,712 / 43,074.004 s | 4.917378 ms / 200.00 Hz | 0.030379 | (+0.023372, −0.001403, +0.012187) | 0.005432 / 0.006904 | `USABLE_WITH_CAVEATS` |

The first/last sensor-header epochs are `1790893059566053134`/`1790893118879674874` ns and `1790893186607705474`/`1790936260612021117` ns. Maximum IMU header intervals are 6.106/9.705 ms, below the unchanged 4-times hard-gap rule; no duplicate/regressive header stamps were observed. Per-axis gyro summaries, raw/residual p90/p95/p99/p99.9/max, storage offsets and all twelve fixed hourly blocks are in the small calibration JSON reports archived with the new inventory. The long bag's hourly median gx moves from about 0.02124 to 0.02444 rad/s, gz from about 0.01112 to 0.01325; the short-vs-long gx median differs by 0.00533. Its accelerometer variance sum over fixed hours is about 0.000078–0.000165 g² (about 0.0075–0.0159 (m/s²)²). **I:** a substantial positive, slowly changing gyro bias plus residual sensor noise explains the raw threshold failure more plausibly than continuous physical rotation. Small unobserved motion or vibration remains possible.

**V, pinned FAST-LIO2:** [IMU_Processing.hpp at `a4743b095409588842a5b30ddfa27e29d2f99164`, lines 145–185](https://github.com/hku-mars/FAST_LIO/blob/a4743b095409588842a5b30ddfa27e29d2f99164/src/IMU_Processing.hpp#L145-L185) accumulates `mean_gyr` over IMU samples and assigns it to `init_state.bg`; lines 223–249 integrate using actual header-stamp deltas. Thus FAST-LIO explicitly models a nonzero starting gyro bias. That source does not guarantee its bias estimate is perfect or excuse moving starts.

## Frozen correction and alternatives

**I — criterio delegado, revisable por Narcís:** replace only the proposed `raw ||gyro|| p95 <= 0.02` screen. For each conditioning/stationary interval, compute its componentwise gyro median `b_window`, then require **both** `p95(||gyro - b_window||) <= 0.006119479213085752 rad/s` and `||b_window|| <= 0.0278346772285473 rad/s`. Comparisons are inclusive; quantiles use NumPy's linear interpolation. Retain the existing raw gyro norm maximum `<=0.05 rad/s` and SI accelerometer sample-variance sum `<=0.04 (m/s²)²`. There is no reference-derived bias subtraction: `b_window` comes solely from the IMU values and remains an evaluator-only statistic. Stationarity also still requires the independent reference A/B position-orientation test and the temporal-continuity checks of [05](05-enmienda-continuidad-temporal.md).

**Derivation frozen before evaluation-bag validation:** split each independent static capture into non-overlapping half-open 2 s windows anchored at its first `Imu.header.stamp`; require at least 95% of `ceil(2 s / measured median cadence)` messages. This yields 29 and 21,537 complete static windows. For each window compute the two statistics above. The thresholds are the **maximum of the two captures' empirical p99.9**: residual-p95 p99.9 = 0.004881655/0.006119479 and median-norm p99.9 = 0.021778037/0.027834677 rad/s. The observed quantization makes the long-record median-norm p99.9 equal its maximum. The 99.9% calibration quantile is a delegated tolerance for rare stationary noise, not a safety-certified false-acceptance bound. This calibration has no eco/hospital/eco03 input.

Options considered: (A) the original raw p95 limit rejects both labelled-static recordings; (B) a larger raw p95 derived from them would still conflate bias and angular variation; (C) subtracting one fixed vector across sessions would reject an ideal zero-bias stationary sensor and ignore the observed between-session drift; (D) the adopted window-centred residual plus independently calibrated median-norm cap separates rapid variation from the bias level without assuming a fixed vector. A perfectly constant slow physical rotation inside the cap can still pass the **IMU-only** screen; the reference A/B check is essential for offline eligibility. No choice was made by maximizing the number of mapping-bag windows. If the static provenance later proves wrong, this amendment must be revisited prospectively, preserving its results.

## Negative check after freezing the rule

**V, evaluator-only diagnostic:** `imu_stationarity_validation.py` divides each evaluation reference span into disjoint 2 s bins anchored at its first reference timestamp. It excludes IMU hard gaps, deficient IMU coverage and invalid reference brackets, then labels a bin `stationary_B` when maximum reference displacement <=0.05 m **and** maximum rotation <=2 degrees; `clearly_moving` when displacement >=0.4 m **or** rotation >=10 degrees; otherwise `ambiguous`. These diagnostic movement cutoffs are not inventory eligibility or method acceptance rules. No IMU threshold was adjusted after this check. The reference labels are used only **after** the independent static calibration and never reach an executor/method.

| Mapping set | Stationary-B bins passing / total | Clearly-moving bins rejected / total | Ambiguous bins passing / total | Median centred p95, moving |
|---|---:|---:|---:|---:|
| eco01_1x | 13 / 15 | 163 / 163 | 0 / 20 | 0.2112 rad/s |
| hospital_02_r05b | 18 / 25 | 153 / 153 | 0 / 58 | 0.2571 rad/s |
| eco03 | 1 / 2 | 90 / 90 | 0 / 10 | 0.1627 rad/s |

**I, limit:** this rejects the obvious motion sampled in these three bags, but says little about very slow, nearly constant yaw motion, other mounts, high vibration or future thermal drift. Some reference-quiet bins fail the IMU test; they are retained as exclusions, not “fixed” by threshold changes. The reference is Voxel-SLAM from the same run, not independent ground truth.

## Regenerated inventory, version 1.2 / schema 3

**V:** the generator records raw gyro norm p95/max, window median vector/norm, centred residual p95, acceleration variance, IMU sample counts and the unchanged temporal diagnostics in private evaluator fields. The method packet whitelist remains only raw sensor bytes plus original sensor timestamps; fixed public scenario budgets come through `window_contract.py`. The funnel below is sequential for selected level **B**: all LiDAR-frame anchors → non-overlapping candidates → pass LiDAR boundary/gap → pass reference bracket and A/B stationarity → pass IMU hard-gap/coverage/stationarity → valid prepared rows after grouping. Counts across variants/levels are correlated and **not independent trials**. S3 rows only mean a cold-start opportunity was prepared; FAST-LIO readiness, online odometry stop and physical post-check remain runtime-pending. The `S2 1s` cell remains a diagnostic ablation, not eligible for the main >=1 s multi-batch confirmation.

| Dataset | Scenario | Anchors | Non-overlap | LiDAR pass | Reference pass | IMU pass / prepared | Independent B blocks |
|---|---|---:|---:|---:|---:|---:|---:|
| eco01_1x | S1 | 5,110 | 4,830 | 2,889 | 24 | 14 | 3 |
| eco01_1x | S2 1s | 5,110 | 4,766 | 2,342 | 10 | 5 | 3 |
| eco01_1x | S2 3s | 5,110 | 4,766 | 1,404 | 5 | 3 | 1 |
| eco01_1x | S2 5s | 5,110 | 4,766 | 807 | 1 | 1 | 1 |
| eco01_1x | S3 cold | 5,110 | 4,199 | 2,659 | 16 | 4 | 3 |
| hospital_02_r05b | S1 | 6,188 | 5,782 | 3,334 | 95 | 20 | 3 |
| hospital_02_r05b | S2 1s | 6,188 | 5,779 | 2,738 | 35 | 6 | 3 |
| hospital_02_r05b | S2 3s | 6,188 | 5,779 | 1,708 | 6 | 1 | 1 |
| hospital_02_r05b | S2 5s | 6,188 | 5,779 | 983 | 6 | 1 | 1 |
| hospital_02_r05b | S3 cold | 6,188 | 4,435 | 2,739 | 83 | 6 | 3 |
| eco03 | S1 | 2,630 | 2,592 | 1,429 | 15 | 2 | 2 |
| eco03 | S2 1s | 2,630 | 2,630 | 1,147 | 5 | 0 | 0 |
| eco03 | S2 3s | 2,630 | 2,630 | 675 | 0 | 0 | 0 |
| eco03 | S2 5s | 2,630 | 2,630 | 368 | 0 | 0 | 0 |
| eco03 | S3 cold | 2,630 | 2,044 | 1,137 | 16 | 2 | 2 |

**V, split/caveat:** all cells still choose B only because A has fewer than 30 blocks; B has at most three blocks per cell. No multiplicity of scans, durations, revisits or sessions is turned into extra independent sites. For eco S2 3s, all three selected windows are final, **none tuning**; hospital and eco03 remain held out (and eco03 only supports its own 2D map). Hospital bag↔map attribution remains **I** without an immutable build manifest. All cells retain `insufficient_independent_coverage`. These counts do **not** satisfy the project acceptance bar or support a formal method comparison.

**V, reproduction:** two fresh runs with identical archived compact input indexes/small map/reference files produced byte-identical ordered outputs: `windows.csv` SHA-256 `f0fdb6b5c4a0971c3399b06c6991aae8b4ad51e26d8a11343ca3378e712066d8`, `windows.metadata.json` `39aef312f457b8499e5ea442dd65e8e76cccc6887fbfacaeb32148c6f384e854`, `exclusions.jsonl.gz` `648bbadccd15a571fcad32ee535f6e4cb42b112a57bc5dda02437810c661aacd`. The new private archive is `/home/jetson/nav_out/global-reloc-stationarity-v3-20261007/`, with `inventory/` and two small reports in `calibration/`. The metadata records source sizes/mtimes, small-file hashes, code/config hashes, exact stage counts, A/B and split. Original point-5 and continuity-v2 archives remain separate. Commands (run in the existing network-isolated mapper container with the static bags mounted read-only for calibration, then locally on the compact prepared inputs):

```sh
python3 -B tools/nav_bench/imu_static_calibration.py <static_imu_bag> <new_small_json>
python3 -B tools/nav_bench/imu_stationarity_validation.py --input-root <prepared-input>
python3 -B tools/nav_bench/window_inventory.py --input-root <prepared-input> --output <new-empty-directory>
```

The raw static MCAPs, compact sensor indexes and generated private inventories stay outside Git; full large-bag/cloud hashes were not computed. This report retains every unfavorable historical result and no AMCL/FAST-LIO/candidate run was executed for the amendment.

## Gate and acquisition plan

**Gate: `READY_FOR_POINT_6_EXPLORATORY_ONLY`.** There are nonzero eligible S1/S2 windows and prepared S3 starts, but at most **3 independent blocks versus the required 30 per cell**. A point-6 run could only be a labelled smoke test; no formal baseline or claim of comparative reliability is authorized by this evidence. The S2 3s tuning shortage makes even exploratory tuning fragile. Do not change A/B, the new IMU thresholds, LiDAR/reference gaps, 0.30 m/8 degrees, 5 m grouping, 35 s guard or the 30-block minimum to improve counts.

**I — acquisition plan, not executed:** first survey a larger mapped floor plan for >=5 m physically separated sites. Aim for roughly **60 distinct development/evaluation sites in eco** (first 30% chronological spatial groups for tuning, remaining 70% final, yielding about 42 possible final blocks before exclusions) and **40 distinct sites in a separately traceable held-out session/map**. If the mapped footprint cannot hold them, enlarge the mapped area; revisits or rotated starts at the same site do not count again. At each site remain genuinely stopped for 10–12 s: 2 s fresh IMU conditioning, the 3 s main S2 exposure, a separate 5 s ablation plus frame/gap margin. For a separately labelled S3 trial, start with >=2 s stationary IMU, then move within operational 2.8 m odometry radius, physically <=3 m, <=6 m path and <=30 s; end it before relocating to the next site. Preserve sensor `/livox/lidar` CustomMsg and `/livox/imu` Imu with original header times, calibration/TF, map PGM/YAML and 3D cloud where required, same-run Voxel-SLAM `alidarState.txt`, and an immutable run manifest tying bag, tool commits/config, map and reference outputs. Suggested bag names: `global_reloc_eco_sites_01` and `global_reloc_heldout_sites_01`; site IDs/truth remain evaluator-only. During capture check rolling header-delta histograms, duplicates/regressions, >=95% IMU samples over 2 s at measured ~200 Hz, hard gaps, LiDAR ~10 Hz with no missing 200–300 ms frames, and live quiet IMU residual/median-norm statistics under this **predeclared** calibration; `ros2 topic hz` alone is insufficient. Record any calibration/mount change as a new provenance item rather than silently reusing these limits. Development and held-out data must remain separate before any method result is examined.
