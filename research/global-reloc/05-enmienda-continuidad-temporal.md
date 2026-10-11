# Temporal-continuity amendment before point 6

Date: 2026-10-07. **V** = measured in the archived sensor indexes, source, or new artifacts; **I** = inferred/delegated technical criterion; **NV-P** = pending. This is a prospective amendment to point 4's *IMU continuity semantics*, made **before any AMCL baseline or relocalization candidate**. It does not amend the historical point-5 commit `1c8aad6055a5001d5cb8d04ab201d20081bdd558` or overwrite its outputs.

## Why the original rule is preserved but insufficient

**V, original protocol/result:** [02-protocolo-evaluacion.md](02-protocolo-evaluacion.md) defined an IMU or LiDAR gap as consecutive sensor-header delta strictly `> 1.5 ×` the dataset's median topic cadence. Any crossing invalidated the observation/conditioning interval, followed by 2 s fresh conditioning. The archived [original inventory](04-inventario-ventanas.md) produced **zero** eligible windows in eco01_1x and hospital_02_r05b. Its original `>1.5 ×` IMU flags numbered 8,297/8,183; maximum uninterrupted spans were 0.653/0.943 s. Those numbers, exclusions and SHA-256 values remain valid **under that original rule**.

**V, diagnostic from the same archived compact indexes:** IMU header spans contain 104,507 samples in 522.585 s (199.979 Hz) and 126,851 in 634.284 s (199.989 Hz). Median deltas are 4.7858075/4.812144 ms; maximum deltas are 20.939/19.655 ms. Of the 8,297/8,183 flagged intervals, 6,895/6,968 are followed immediately by an interval shorter than half the median. Moving 2 s intervals contain 397–404/398–403 actual samples. Storage timestamps have *more* irregularity (12,001/12,939 flags), so substituting bag storage time is not an evidence-based repair. **I:** the old 1.5-times rule mainly detects scheduling/timestamp irregularity, not thousands of proven missing samples. **NV-P:** the bag does not retain the packet `time_type` or a packet-loss counter, so a small true-loss component cannot be identified per event. The pinned driver can choose device packet time or host time depending on synchronization state [D1].

**V, source:** the pinned FAST-LIO IMU processor accumulates IMU samples for initialization and propagates using their real header-stamp deltas; it has no 1.5-times-cadence reset [F1]. This does not prove an arbitrary gap is harmless. **I — criterio delegado, revisable por Narcís:** separate a high-sensitivity diagnostic from an invalidating interval instead of interpreting every timing excursion as broken 2 s conditioning. The factor 4 and 95% coverage are fixed **before AMCL/candidate results**, and are not selected to make a desired window count.

## Amended, versioned rule

**I — criterio delegado, revisable por Narcís:** for each dataset, calculate the median strictly positive adjacent `Imu.header.stamp` delta `m` once. Reject duplicate or regressive stamps. Record `imu_timing_irregularity` for `Δt > 1.5m`; this is diagnostic only. Define `imu_hard_gap` for `Δt > 4m`, strictly greater. No scored observation, stationary certification, or 2 s conditioning interval may cross an `imu_hard_gap`. After one, the full 2 s conditioning starts anew. A 2 s conditioning interval is half-open `[start,end)` and must contain at least 95% of `N_expected = ceil((end_ns-start_ns)/m_ns)` actual IMU messages with headers inside the interval. `m_ns` is an exact integer or half-integer ns median of integer deltas; the implementation uses exact doubled-median integer arithmetic. The pass comparison is `100 × N_observed >= 95 × N_expected`; equality passes. Record expected/observed/ratio and irregularity/hard-gap counts in evaluator-only data. These checks do not certify stationarity: the unchanged reference A/B and gyro/accelerometer criteria still apply.

**V, unchanged:** LiDAR continues to mark `lidar_gap` at `CustomMsg.header.stamp Δt > 1.5 ×` its own median. It has 114/151 such gaps in eco/hospital, often about 200–300 ms instead of 100 ms; the same events appear in storage timing. They represent missing *published frames* or equivalent output discontinuities; the cause at raw packet level is **NV-P**. The new eco03 index has 72 LiDAR gaps. Smaller LiDAR timing variation remains in the indexed timestamps and is not an automatic gap. Reference interpolation remains bounded by 0.20 s with no extrapolation, hold-last or nearest substitution. The 0.30 m/8° acceptance limits, truth firewall, independent 5 m blocks, 35 s guard, split, S1/S2/S3 definitions, and minimum 30 independent blocks per cell are unchanged. No threshold is adjusted from candidate results.

**V, implementation:** `tools/nav_bench/window_inventory.py` version 1.1/config schema 2 records both IMU classes, checks 4-times hard gaps and 95% conditioning coverage, and keeps LiDAR/reference rules. Its evaluator-only CSV columns include `evaluator_only_imu_timing_irregularity_count` (over the certified support), `evaluator_only_imu_hard_gap_count`, `evaluator_only_imu_expected_samples`, `evaluator_only_imu_observed_samples`, and `evaluator_only_imu_coverage_ratio` (over conditioning only). Rejected anchors retain their first failing reason in the private exclusion log; counts below are not simultaneous-fault counts. The method contract still accepts only raw sensor packets and fixed public scenario configuration. No private CSV/JSON is a candidate input. No FAST-LIO, AMCL or relocalizer was run.

## New inventory: three evaluated input sets

**V:** the two archived sensor indexes were reused byte-identically; eco03 was indexed read-only from its MCAP bag using the existing `socialtech:mapper-verify` image (`sha256:88380b4b1b720a2e5f3b65fd0091ba7d9229fb409d87ce96f04184b981ed330c`), rosbags 0.11.5 and the same sensor-index reader. Source map/reference files were read unchanged. The raw uncompressed index SHA-256 values are eco `fff95dd7af58fde3cf484005ee19d5d43adc1b7049c51750aa814babdf284d8e`, hospital `b45e4711ddd6f1a5290854cb9ef574b94a26b714b7ee0bad38bd90273e086aad`, eco03 `7749f5c47f339cc74bafe5698f7ab50ab410428fcc41397ad0ab00c63822d06e`. The exact small-file hashes and bag size/mtime are in `windows.metadata.json`; full MCAP hashes were not computed. Dataset-wide events are:

| Dataset | LiDAR frames / gap flags | IMU samples / 1.5-times irregularities / 4-times hard gaps | Reference brackets >0.20 s |
|---|---:|---:|---:|
| eco01_1x | 5,110 / 114 | 104,507 / 8,297 / 1 | 68 |
| hospital_02_r05b | 6,188 / 151 | 126,851 / 8,183 / 1 | 83 |
| eco03 | 2,630 / 72 | 54,101 / 4,195 / 0 | 34 |

**V:** all cells select B through the predeclared A→B ladder; both levels produce zero windows and zero independent spatial/temporal blocks. The table gives **selected-B first-cause exclusions** as `IMU / LiDAR / reference`, including outside-span reasons; a reference failure may also be present in an anchor listed under LiDAR. `S3` means *preparable* input only, not a runtime FAST-LIO result.

| Dataset | Scenario | Valid / blocks | First-cause exclusions IMU / LiDAR / reference | Split |
|---|---|---:|---:|---|
| eco01_1x | S1 | 0 / 0 | 267 / 1,955 / 2,888 | 0 tuning, 0 final |
| eco01_1x | S2 1 s | 0 / 0 | 239 / 2,501 / 2,370 | 0 tuning, 0 final |
| eco01_1x | S2 3 s | 0 / 0 | 176 / 3,474 / 1,460 | 0 tuning, 0 final |
| eco01_1x | S2 5 s | 0 / 0 | 134 / 4,092 / 884 | 0 tuning, 0 final |
| eco01_1x | S3 cold | 0 / 0 | 273 / 1,833 / 3,004 | 0 tuning, 0 final |
| hospital_02_r05b | S1 | 0 / 0 | 334 / 2,592 / 3,262 | 0 heldout |
| hospital_02_r05b | S2 1 s | 0 / 0 | 194 / 3,264 / 2,730 | 0 heldout |
| hospital_02_r05b | S2 3 s | 0 / 0 | 99 / 4,341 / 1,748 | 0 heldout |
| hospital_02_r05b | S2 5 s | 0 / 0 | 78 / 5,067 / 1,043 | 0 heldout |
| hospital_02_r05b | S3 cold | 0 / 0 | 357 / 2,475 / 3,356 | 0 heldout |
| eco03 | S1 | 0 / 0 | 39 / 1,177 / 1,414 | 0 heldout |
| eco03 | S2 1 s | 0 / 0 | 35 / 1,453 / 1,142 | 0 heldout |
| eco03 | S2 3 s | 0 / 0 | 49 / 1,906 / 675 | 0 heldout |
| eco03 | S2 5 s | 0 / 0 | 69 / 2,193 / 368 | 0 heldout |
| eco03 | S3 cold | 0 / 0 | 41 / 1,125 / 1,464 | 0 heldout |

**V, independent second blocker:** after LiDAR/hard-gap/reference-B filtering, 226/311/18 S1 anchors remain in eco/hospital/eco03, but **none** passes the already documented IMU gyro p95 `<=0.02 rad/s`. The smallest p95 in these sets is 0.02847/0.02549/0.02371 rad/s. For S2 3 s, the respective post-filter counts are 108/49/0, also zero IMU passes. Those gyro/variance screens were explicitly marked *proposed and unvalidated* in point 4, then used unchanged by point 5. This run does not revise them based on the observed counts. **NV-P:** whether fixed mount vibration, gyro bias, a units/configuration error, or physical movement explains each failure. Future data with the same sensor/setup may fail again; new recordings alone are not a proven fix.

## Additional-dataset audit

**V:** eco03's bag `custom/mapping_eco_-1_03_custom` exists (270.522 s, CustomMsg 2,630; IMU 54,101). `nav_data/eco_-1_03/final.{pgm,yaml}` and `alidarState.txt` exist; reference timestamps 1781004806.673302–1781005075.973884 lie in the bag interval. The map has 0.05 m cells and a `T_map_world` block. It has no corresponding 3D cloud in the inspected set. An independent translation-only cross-correlation of occupied PGM pixels against eco01's own PGM peaks at (+1,−1) pixels: 25,687 coincident occupied pixels, 0.670 occupied IoU and 92.98% agreement among 130,955 cells known in both maps. **I:** this strongly supports the same physical area in a different session; it does not establish a fixed inter-map transform for scored cross-session localization. The name/timestamps support the bag↔reference/map association but no immutable build manifest proves it. Use eco03's *own* map/reference for a separately labelled 2D inventory; keep it held out, do not pool it with eco01 tuning/final, and do not claim 3D-candidate comparability. Classification: `APPROVED_FOR_INVENTORY` for that restricted purpose. Reference-only upper screening found at most 3 distinct 5 m groups satisfying B for 2 s, one for 3 s, and **zero for 5 s**, before IMU/LiDAR checks; the generated valid count is zero.

**V:** `benchmarks/m3dgr_Dynamic01_custom` exists (175.158 s), with topics `/livox/mid360/lidar` and `/livox/mid360/imu`, not the two topics in the current reader. `nav_data/dynamic01/final.{pgm,yaml}` and `alidarState.txt` exist; reference times 1732437229.741190–1732437403.641630 lie within the bag. The reference path covers roughly 6.2 by 2.9 m; a reference-only scan of every reference-row start found **no** 2 s stationary A or B start, even before IMU/LiDAR checks. A translation-only comparison with eco01 gives occupied IoU 0.174, insufficient to establish a shared map; rotations or other transforms were not searched. Scene dynamics are not annotated in a way that certifies static-map correspondence. Bag↔map provenance remains **I** without a build manifest. Classification: `REJECTED` for this stationary-start protocol, with 0 potential start blocks; no reader/topic adaptation or official inventory was run.

**I, limits:** reference-only candidate counts are upper screens, not valid windows, and 5 m components from different maps/sessions cannot be added as independent sites without physical correspondence. Hospital bag↔r05b provenance remains **I** as in point 5. No dataset is promoted to a verified between-session scoring pair solely from map resemblance.

## Reproduction and point-6 gate

**V:** the amended private archive is `/home/jetson/nav_out/global-reloc-continuity-v2-20261007/`: `inventory/{windows.csv,windows.metadata.json,exclusions.jsonl.gz}` plus `eco03_index/sensor-index.json.gz`. The original archive `/home/jetson/nav_out/global-reloc-point5-20261007/` was confirmed present and untouched. Two fresh local runs with identical inputs produced byte-identical files:

| Artifact | SHA-256 |
|---|---|
| `windows.csv` | `ef0ac38489083ebc29c8b7850af7a8f6058d537c2fffa495feaed3e6a7b9a9e4` |
| `windows.metadata.json` | `4950fe214065fdecde2334cbf999fde28daec4a88c981182bf13557164bff034` |
| `exclusions.jsonl.gz` | `6a381fc59398121133ab17483370de2939f4110e37909fcd12556ca3cf58493f` |

The empty CSV has a complete version-2 schema. Its lack of rows does not make a benchmark ready. Dataset input folders use archived compact indexes and small `map.pgm`, `map.yaml`, `alidarState.txt`; eco03's `final.*` is copied under those schema names without changing its bytes. Run `python3 -B tools/nav_bench/window_inventory.py --input-root <prepared-input> --output <new-empty-directory>`. The exact input hashes, generator/config hashes, runtime versions and first-cause exclusions are in the archive metadata. No full bag/nube hashes were made.

**Gate: `NOT_READY_FOR_POINT_6`.** Actual valid windows/blocks are **0/0**, versus at least **30 independent blocks per dataset/scenario cell** for conclusions. The 100-start target, A/B ladder, grouping and split are unchanged. A point-6 smoke test would still be noncompliant because it lacks even one currently eligible input; do not relabel a failed eligibility check as a method failure.

## Acquisition and calibration plan if the gate stays closed

**I — criterion for a future, separately approved collection:** survey the mapped indoor floor first. Plan **about 60 physically distinct eco sites** (30% chronological tuning leaves about 42 final before exclusions) and **about 40 held-out sites** in a separate approved session/map. Every pair intended to count separately must be at least 5 m apart *in physical map coordinates*; repeated headings, durations, visits or scans at one site count once. If those numbers do not fit the available mapped footprint, extend the mapped area or use additional traceable indoor sites; do not manufacture blocks from revisits. Aim above the minimum because some sites will be excluded. Keep a 35 s tuning/final temporal guard.

At each site record a genuinely stationary **10–12 s** interval: at least 2 s fresh IMU conditioning plus the main 3 s S2 exposure and 5 s ablation, with margin for frame boundaries and rejected data. Then, for a separately labelled S3 start, remain nearly stopped for its 2 s cold IMU initialization and make a short trajectory with operational stop at 2.8 m local-odometry radius, physical radius <=3 m, path <=6 m and <=30 s; return/travel to the next *new* site only after ending that trial. Record one raw continuous session or clearly linked bags with `/livox/lidar` `CustomMsg` and `/livox/imu` `Imu` (and any calibration/TF needed for later base_link conversion). Check live received rates near 10/200 Hz **and** rolling header-delta histograms, 2 s sample counts, duplicates/regressions and LiDAR 200–300 ms frame gaps; `ros2 topic hz` alone measures arrival, not header continuity. Preserve bag metadata, map PGM/YAML, cloud where 3D methods are planned, same-run Voxel-SLAM `alidarState.txt`, sensor calibration, transform and an immutable run manifest naming exact raw bag, processing commands/tool commits, map/reference output directory and small-file/index hashes. Suggested names: `global_reloc_eco_sites_01`, `global_reloc_hospital_sites_01`, with site IDs in a separate evaluator-only log; no true pose/ID enters method input.

**Required before relying on that collection:** determine and freeze an IMU stationarity treatment that distinguishes a fixed gyro bias from actual motion, using independent stationary calibration or source evidence, **before** scoring relocalizers. The observed p95 floor already exceeds the current proposed 0.02 rad/s threshold at the few reference-stationary anchors; merely recording more data may leave zero windows. Any change needs its own prospective protocol note, new versioned inventory and preserved negative results. A short acquisition pilot can validate the sensor/IMU-quality and traceability checks without being counted as formal benchmark evidence.

## Sources

- [D1] Livox driver pinned `21445540f0d100dc86a7e6df312dd70bbdb4afdf`, [IMU/CustomMsg publication](https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/src/lddc.cpp), [packet timestamp selection](https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/src/comm/pub_handler.cpp).
- [F1] FAST-LIO ROS 2 source pinned `a4743b095409588842a5b30ddfa27e29d2f99164`, [IMU initialization and timestamp integration](https://github.com/hku-mars/FAST_LIO/blob/a4743b095409588842a5b30ddfa27e29d2f99164/src/IMU_Processing.hpp).
- Local evidence: original commit `1c8aad6055a5001d5cb8d04ab201d20081bdd558`; archived indexes and manifests at the two paths above; `tools/nav_bench/window_sensor_index.py`, `window_inventory.py`, `test_window_inventory.py`, and `window_contract.py` at the new local commit.
