# Point 5 — Real sensor inventory and window result

Date: 2026-10-07. **V** = measured/read in the stated artifacts or pinned source; **I** = interpretation or delegated design choice; **NV-P** = not established. Point 4's numerical rules were not changed. Point 5 generated a reproducible **empty eligible-window inventory** for the two approved datasets. This is a measured negative result, not evidence that the robot never stops. Under the decided IMU inter-message gap definition, no uninterrupted 2.0 s conditioning interval exists in either bag. No localization candidate, FAST-LIO or AMCL was run.

## Inputs, readers and time semantics

**V:** the raw files are the existing MCAP bags `custom/mapping_eco_-1_01_custom` and `custom/hospital_02_custom`. The corresponding small `alidarState.txt`, `map.yaml`, `map.pgm` and exact hashes are listed in [03-generador-posiciones.md](03-generador-posiciones.md). The raw-message index was produced once per bag with `tools/nav_bench/window_sensor_index.py`, rosbags 0.11.5 and the existing `socialtech:mapper-verify` image (`sha256:88380b4b1b720a2e5f3b65fd0091ba7d9229fb409d87ce96f04184b981ed330c`). The bag and tools mounts were read-only, the container had no network and ran as the ordinary data owner. The compressed indexes are 4,055,458 / 4,328,353 bytes; their uncompressed JSON SHA-256 values are `fff95dd7af58fde3cf484005ee19d5d43adc1b7049c51750aa814babdf284d8e` / `b45e4711ddd6f1a5290854cb9ef574b94a26b714b7ee0bad38bd90273e086aad`. Full MCAP hashes were deliberately not calculated. Source-size/mtime and small-file hashes are in `windows.metadata.json`; they identify this snapshot but do not cryptographically prove unchanged large bag bytes.

**V, inspected code:** `livox_ros_driver2` pinned `21445540f0d100dc86a7e6df312dd70bbdb4afdf`, `src/lddc.cpp:480-495`, publishes IMU packet gyro/acceleration unchanged. The installed Voxel-SLAM revision `4d06f3e3150c0a1b224b234328ec8e0d9a0377f7`, `feature_point.hpp:176-205` uses CustomMsg `header.stamp` as scan start and `offset_time / 1e9` as relative seconds; `voxelslam.hpp:150` computes the end from the last retained filtered point. `voxelslam.cpp:246-268` writes reference times to six decimal places. The bag conversion `pc2_to_custom.py` constructs `timebase` from the minimum original per-point epoch nanosecond timestamp, stores relative `offset_time`, retains the original PointCloud2 `header.stamp`, and copies IMU CDR unchanged. The index fast CDR decoder was cross-checked against rosbags' schema decoder for the first message of both topics. The installed driver and Voxel-SLAM file hashes can be re-read from the NX; no source file was changed.

**V / I, units:** Livox's [IMU protocol](https://github.com/Livox-SDK/Livox-SDK/wiki/Livox-SDK-Communication-Protocol) specifies gyro in rad/s and acceleration in g. The installed driver does no SI conversion; the map config has `imu_acc_unit_is_g=true`; median raw acceleration norms are 1.00043 g (eco) and 0.99728 g (hospital). Together these support treating recorded acceleration as g. **I — implementation convention:** multiply acceleration by 9.80665 before the protocol's SI variance test; retain gyro in rad/s. The 9.80665 factor is the standard-gravity unit conversion, not a tuned stationarity threshold. The protocol's IMU p95/max/variance thresholds are unmodified and remain unvalidated for this mount.

| V: sensor header observations | eco01_1x | hospital_02_r05b |
|---|---:|---:|
| LiDAR CustomMsg / IMU counts | 5,110 / 104,507 | 6,188 / 126,851 |
| LiDAR first / last header epoch ns | 1780999549738891781 / 1781000072139338191 | 1781693277939433240 / 1781693912039150008 |
| IMU first / last header epoch ns | 1780999549694325117 / 1781000072279720947 | 1781693277948282200 / 1781693912231814584 |
| LiDAR / IMU median header delta | 100.000019 / 4.7858075 ms | 99.991904 / 4.812144 ms |
| LiDAR / IMU maximum header delta | 301.382 / 20.939 ms | 302.146 / 19.655 ms |
| LiDAR / IMU duplicate, regressive stamps | 0 / 0 | 0 / 0 |
| LiDAR / IMU gaps, delta >1.5 x topic median | 114 / 8,297 | 151 / 8,183 |
| Longest uninterrupted LiDAR / IMU span | 26.803 / **0.653 s** | 35.400 / **0.943 s** |
| One-frame median first-to-last point span | 100.244 ms | 100.208 ms |
| LiDAR bag storage minus header, median | 103.723 ms | 101.430 ms |
| IMU bag storage minus header, median | 0.348 ms | 0.273 ms |
| CustomMsg header minus point `timebase`, range | -128 to +128 ns | -104 to +120 ns |

**V:** CustomMsg timebase and header agree to within about 0.1 microsecond, while storage time differs and cannot be used as the represented scan time. The apparent nanosecond variation is compatible with conversion through floating-point original per-point timestamps (**I**), and no offset was applied. The reference times share the sensor epoch scale and lie inside the sensor bag. Against the nearest **unfiltered** per-frame last point, the reference timestamp difference has median -9.944/-9.960 microseconds and range down to -8.039/-7.081 ms. Filtering and six-decimal export explain why exact equality is not expected (**I**). **NV-P:** a one-to-one reference-row↔raw-frame identity, the exact filtered scan end for every row, and any constant hardware offset have not been proved. The initial bag/reference delays of ~0.34/~0.29 s are not offset estimates. No timestamp shift was fitted to reference or candidate results. Exact reference samples are used directly; interpolation is confined to brackets <=0.20 s, with no extrapolation/hold/nearest substitute.

## Inventory and decisive limitation

**V:** metadata and raw headers establish that both input topics are monotonic, with no duplicate stamps. The gap rule is applied separately to each topic at its measured median cadence. A gap means a header delta *strictly greater* than 1.5 times that median; this is a protocol validity flag, not a claim that every such interval proves packet loss. In particular, jitter or acquisition scheduling could contribute. The longest IMU spans above are shorter than the **required 2.0 s continuous conditioning**. Consequently all real candidate anchors fail before stationarity A/B, IMU vibration certification, reference grouping, split or S3 runtime validation can determine eligibility. A change to the gap rule would be a protocol revision requiring a new version and regenerated inventory; this run did not make one.

| Dataset | Cell | Tried anchors per level | A windows / blocks | B windows / blocks | Chosen level | Selected windows / blocks | Split |
|---|---|---:|---:|---:|---|---:|---|
| eco01_1x | S1 | 5,110 | 0 / 0 | 0 / 0 | B | 0 / 0 | 0 tuning, 0 final |
| eco01_1x | S2 1 s | 5,110 | 0 / 0 | 0 / 0 | B | 0 / 0 | 0 tuning, 0 final |
| eco01_1x | S2 3 s | 5,110 | 0 / 0 | 0 / 0 | B | 0 / 0 | 0 tuning, 0 final |
| eco01_1x | S2 5 s | 5,110 | 0 / 0 | 0 / 0 | B | 0 / 0 | 0 tuning, 0 final |
| eco01_1x | S3 cold | 5,110 | 0 / 0 | 0 / 0 | B | 0 / 0 | 0 tuning, 0 final |
| hospital_02_r05b | S1 | 6,188 | 0 / 0 | 0 / 0 | B | 0 / 0 | 0 heldout |
| hospital_02_r05b | S2 1 s | 6,188 | 0 / 0 | 0 / 0 | B | 0 / 0 | 0 heldout |
| hospital_02_r05b | S2 3 s | 6,188 | 0 / 0 | 0 / 0 | B | 0 / 0 | 0 heldout |
| hospital_02_r05b | S2 5 s | 6,188 | 0 / 0 | 0 / 0 | B | 0 / 0 | 0 heldout |
| hospital_02_r05b | S3 cold | 6,188 | 0 / 0 | 0 / 0 | B | 0 / 0 | 0 heldout |

**V:** the fixed ladder selects B because the A method-blind inventory is below 30 blocks; B also has 0. Every cell has `insufficient_independent_coverage`. No train/final/heldout evidence exists and there is no valid temporal distribution to report; attempted anchors cover the bag's sensor-time extent but all are excluded. The archived exclusion log records one first failing reason per anchor/level/cell, usually an IMU or LiDAR gap; it cannot be interpreted as counts of independent simultaneous faults. For example, the B S1 exclusions are eco: 3,135 `imu_gap`, 1,954 `lidar_gap`, 20 `imu_outside`, 1 `lidar_outside`; hospital: 3,575 `imu_gap`, 2,592 `lidar_gap`, 21 `imu_outside`. The complete counts for S2 variants and S3 are in the JSON manifest. The earlier 5A reference-only stationary counts of 2–7 intervals were explicitly pre-gap/pre-IMU estimates and remain valid only as that limited estimate.

**NV-P:** S3 online stop at 2.8 m, cold FAST-LIO readiness, local path budget, physical <=3 m/6 m post-check and `S3_continuous_rebased` availability require a future permitted sensor-only odometry execution. The generator stores only private feasibility diagnostics and fixed budgets; it never turns reference radius/path/prefix into EOF or an online stop. There was no eligible S3 start to prepare under the current gap rule. The continuous variant is recorded as unavailable because no certified sensor-only odometry artifact exists, not silently pooled with cold starts.

## Reproduction, isolation and tests

**V:** `window_sensor_index.py` reads only raw sensor messages. `window_inventory.py` runs on compact indexes and small map/reference files; it emits private `windows.csv`, `windows.metadata.json` and `exclusions.jsonl.gz`. Its versioned config records the A/B thresholds, IMU conversion, gap factor, time budgets, grouping and split rules, `seed=null`, protocol commit and source hashes. Empty inventories retain the full CSV schema and a zero row count. The input/output directories are explicit; no bag, PLY or full hash is added to Git. The private output archive is `/home/jetson/nav_out/global-reloc-point5-20261007/local-inventory/`; the two compact sensor indexes remain in sibling dataset directories. That archive is evaluator data and must be mounted inaccessible to future method processes.

Run the index command for each bag with the existing read-only `socialtech:mapper-verify` image, using a fresh output directory; copy its compact index plus each dataset's small `alidarState.txt`, `map.yaml` and `map.pgm` into the matching dataset input directory. Then run:

```sh
python3 -B tools/nav_bench/window_inventory.py --input-root <prepared-input> --output <new-output>
```

**V:** two invocations on identical input bytes produced identical ordered CSV, metadata JSON and gzip exclusions. SHA-256: CSV `c0746f257b85da593fd33bac48b4a09ba53583d06f23cd003e3c24c7afe1ba28`; metadata JSON `53f5bb6373a64dbfd4fc0b889658ba7a522f6c38fb1f6de95df6cffaf41ee42e`; exclusions gzip `2a23b2db9c6dd352354bafd6e7ab4c8f1d2c86180874554b4d0084518d5468e7`. The SHA values refer to the final code run, not the earlier exploratory pass.

**V:** `test_window_inventory.py` exercises timestamp conversion, exact/0.20 s reference brackets, no extrapolation, gap equality, reconditioning, IMU SI conversion and variance, group/split/guard boundaries, empty schema, deterministic positive synthetic windows, and the narrow method packet contract. The directly relevant non-ROS `tools/nav_bench/test_*.py` suite passed. An attempted run of the unrelated ROS node test did not initialize because the local ROS log directory is read-only; it is not evidence of a point-5 code failure. No candidate or AMCL test was run.

**I, firewall contract:** the private generator output is evaluator-owned. `window_contract.py` supplies only fixed scenario configuration and raw serialized sensor packets with their original timestamp; its validator rejects any extra field, including reference/split/prefix hints. The repository has no point-5 candidate process, so this verifies the interface schema and static source boundary, not filesystem isolation of a future candidate process. A later runner must keep the private archive inaccessible to that process. `run_bench_a.sh`, `pose_eval.py` query initializers/levelling, `--reference-csv`, saved poses and map_visibility reference joining are absent from the generator's query preparation.

**I — criterio delegado, revisable por Narcís:** AGENTS.md's arbitrary free-cell/headings request remains unresolved and unsatisfied. This point inventories real observed query windows; it creates neither synthetic scans nor true-pose seeds. It could instead refer to candidate hypotheses, but that is unproved. The hospital bag↔r05b association remains **I**, supported by run logs/timestamps but without an immutable raw-bag provenance manifest. No conclusion about method performance follows from this zero-window inventory.

**Consequence:** point-5 tooling and the requested inventory are complete, including the negative result. The point-6 AMCL baseline cannot produce a valid protocol comparison on these two bags under the currently frozen input-continuity rule. Resolving this requires an explicit protocol/data decision or different approved recordings; no threshold, independence minimum or data were silently changed here.
