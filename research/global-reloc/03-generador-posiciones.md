# Point 5 — Initial-window generator: design and data reconnaissance

Date: 2026-10-07. **5A design closed; 5B implemented and inventoried.** This document preserves the 5A reconnaissance and design as written before raw-message analysis. The measured point-5 result, including zero eligible windows under the fixed input-continuity rule, is in [04-inventario-ventanas.md](04-inventario-ventanas.md). Governing protocol: [02-protocolo-evaluacion.md](02-protocolo-evaluacion.md), commit `31cd64ff0034072a76699546ab47281a47359871`. Point-3 ranking: `17220b3`. AGENTS.md read in full, including its `e4a34b1` version. **V** = inspected source/file or confirmed instruction; **I** = inference/design proposal; **NV-P** = not verified/pending. Adopted point-4 decisions remain unchanged.

## 1. Scope conflicts and boundaries

- **V, scope conflict:** AGENTS.md asks for random free PGM cells and headings as starting poses; protocol B2 explicitly limits real-data queries to observed bag poses and prohibits reference-based query joining/initialization. Arbitrary free-cell poses have no corresponding observed scan. **I — criterio delegado, revisable por Narcís:** provisionally, point 5 delivers the reproducible manifest of real observed windows required by the protocol. AGENTS's free-cell/headings requirement remains an additional requirement pending clarification, not satisfied by this manifest. It might concern candidate hypotheses/poses rather than fabricated LiDAR queries; do not assume synthetic scans are required. No guessed/true pose seeds a global method, and neither AGENTS nor the protocol is rewritten here.
- **V:** acceptance limits agree: XY <=0.30 m AND yaw <=8 degrees; generator has no acceptance decision. Protocol B3 scores the first committed localization event (with wrong acceptance absorbing); the existing reference-based 5 s convergence hold belongs to B7, not initial acceptance.
- **V, scope update:** AGENTS's 2026-10-03 table says hospital lacks outputs; the r05b files inspected below now exist. This is stale inventory, not grounds to exclude the approved dataset or reopen its prior audit.
- **V / I:** protocol uses a local odometry stop at 2.8 m and hard reference radius <=3 m. Reference-only prefixes at 2.8 m are private offline feasibility diagnostics, never execution cutoffs. The executor uses permitted trial odometry to stop at 2.8 m; reference checks the actual <=3 m radius and <=6 m path afterwards. Neither a reference-based prefix nor a 0.2 m margin proves a drifted live controller obeys the limit.
- **V:** only small references/YAML/metadata/logs were copied locally. No bag message, ROS command, Docker command, candidate or replay was executed. No NX output directory/file was created. Names below are verified filesystem paths, not remote access details.

## 2. Dataset inventory — V, observed 2026-10-07

Roots used below: `M=/home/jetson/map_out/<dataset_id>`; `B` is the actual raw-bag directory in the table. All timestamps in this report are UTC; sizes are bytes. This reconnaissance resolves paths and hashes for small files only; it does not revise B1 in the approved protocol.

| Dataset | Raw bag directory B | Container / encoding | Duration s | LiDAR topic/type; messages; count/duration Hz | IMU topic/type; messages; count/duration Hz |
|---|---|---|---:|---|---|
| eco01_1x | `/home/jetson/rosbags/custom/mapping_eco_-1_01_custom` | MCAP / CDR | 522.585325877 | `/livox/lidar`, `livox_ros_driver2/msg/CustomMsg`; 5110; 9.778307 | `/livox/imu`, `sensor_msgs/msg/Imu`; 104507; 199.980740 |
| hospital_02_r05b | `/home/jetson/rosbags/custom/hospital_02_custom` | MCAP / CDR | 634.284032288 | `/livox/lidar`, `livox_ros_driver2/msg/CustomMsg`; 6188; 9.755882 | `/livox/imu`, `sensor_msgs/msg/Imu`; 126851; 199.990846 |

**V:** both metadata.yaml files use version 8, storage_identifier=mcap, ros_distro=rosbags, empty compression fields and only these two topic entries. LiDAR is Livox CustomMsg. **I:** count/duration is a whole-bag mean, not a cadence or proof of gap-free input. Conversion/storage time is not necessarily sensor header time. No odometry topic is listed; required FAST-LIO odometry would be computed later, not read from a listed raw-bag topic.

### File paths, sizes, modification dates

| Dataset | Path (relative to verified M or B) | Bytes | mtime UTC |
|---|---|---:|---|
| eco01_1x | `M/map.pgm` | 351444 | 2026-10-03T00:39:34.376839+00:00 |
| eco01_1x | `M/map.yaml` | 423 | 2026-10-03T00:39:34.376839+00:00 |
| eco01_1x | `M/map_clean.ply` | 4301868 | 2026-10-03T00:39:34.412841+00:00 |
| eco01_1x | `M/vs/run/alidarState.txt` | 1421693 | 2026-10-03T00:31:58.646807+00:00 |
| eco01_1x | `M/world.ply` | 10920408 | 2026-10-03T00:33:08.281582+00:00 |
| eco01_1x | `B/mapping_eco_-1_01_custom.mcap` | 2081619412 | 2026-10-01T18:37:19.000643+00:00 |
| eco01_1x | `B/metadata.yaml` | 1128 | 2026-10-01T18:37:19.004644+00:00 |
| hospital_02_r05b | `M/map.pgm` | 551912 | 2026-10-05T09:57:59.331659+00:00 |
| hospital_02_r05b | `M/map.yaml` | 439 | 2026-10-05T09:57:59.331659+00:00 |
| hospital_02_r05b | `M/map_clean.ply` | 4570116 | 2026-10-05T09:57:59.367660+00:00 |
| hospital_02_r05b | `M/vs/run/alidarState.txt` | 1706543 | 2026-10-05T07:46:42.790685+00:00 |
| hospital_02_r05b | `M/world.ply` | 12150445 | 2026-10-05T09:44:54.325013+00:00 |
| hospital_02_r05b | `B/hospital_02_custom.mcap` | 2519955422 | 2026-10-01T18:37:00.290462+00:00 |
| hospital_02_r05b | `B/metadata.yaml` | 1116 | 2026-10-01T18:36:38.023867+00:00 |

**V:** `M/map.yaml` is also the YAML containing the four-row `# T_map_world` comment block. Both maps have 0.05 m resolution and image=map.pgm. Eco origin is [-30.3,-12.8,0]; hospital origin [-25.9,-16.6,0]. These transforms are prior-map metadata usable by the evaluator; do not use reference poses to initialize a query frame. map_clean/world contents were not opened or geometrically revalidated here.

### SHA-256 — V, small files read

| Dataset | File | SHA-256 |
|---|---|---|
| eco01_1x | `map.pgm` | `864459602b54688caa50daa6751e69771315bba40407f9f4800ea3e3f627950a` |
| eco01_1x | `map.yaml` | `c1114899d0c12ab911533c4cd95118d9206f59735250cb4cbd96bb7d80d67226` |
| eco01_1x | `alidarState.txt` | `00d84f850d749e71d50d4a5a0fcfa9deea78dbc67a0b6baab7074259c6fb55be` |
| eco01_1x | `metadata.yaml` | `8278eb2a8ab97c208e14e9e40209fd95892f776b011407e6147fb10e75ae01b3` |
| hospital_02_r05b | `map.pgm` | `9097cd2d5b67d4bcc2844113ba6056282ebb9a17010f9cc9a54f569c015ac424` |
| hospital_02_r05b | `map.yaml` | `518db17b5bdfa2f22d072accb3f991ca38df178ff8e96ee506cb07b9cf11d717` |
| hospital_02_r05b | `alidarState.txt` | `fd3aa6a1f1e925e78e4bc3ae486661c5a0c0f533b00890cf2b1d9301eb2085fe` |
| hospital_02_r05b | `metadata.yaml` | `be0cf16e1e959e6d8a952c5738275e8c7cd649e01ec93fa222bcc28740804f2f` |

**NV-P:** bag and PLY hashes were not calculated. **I, cost proposal only:** hashing both bags reads 4,601,574,834 bytes once; at hypothetical 50–200 MB/s this is about 23–92 s plus storage/cache/CPU overhead. Those rates are not measured NX throughput. A future streaming timestamp pass may compute the bag digest concurrently if approved, avoiding an extra pass; no such read is authorized in 5A. PLY digests can be added later without geometric decoding (about 32 MB total).

### Hospital map/run relationship

**V, existing r05b files:** voxelslam.yaml says save_path=/out/hospital_02_r05b/vs/, bagname=run, previous_map empty, is_save_map=1, imu_acc_unit_is_g=true, and the two topics above. node.log reports session run, no previous sessions, first IMU timestamp 1781693277.948282 and final pose-graph optimization. play.log records rate 0.5. vs_to_ply.log reports 6,171 frames -> 1,012,527 points; report.json also reports 6,171 poses. ply_to_map.log records floor levelling/wall rotation and the obstacle band; final map_visibility.log records the later final grid. Raw recording time is June 17; mapping/postprocessing mtimes are October 5.

**I:** the named output set, matching reference time range, first-IMU log and frame counts support the association hospital_02_custom <-> hospital_02_r05b as one mapping session followed by postprocessing. The rate 0.5 log supports an offline replay run; the r05b directory name alone does not prove a different recording or any timestamp rescaling. This association remains I, not V. **NV-P:** no immutable command manifest naming the raw bag and every tool/image digest was found among these inspected top-level files; voxelslam_run.log is empty. This reconnaissance supplies the documented linkage, not cryptographic proof or a renewed audit. It is sufficient for the approved scope; preserve this provenance limit.

## 3. Reference time axis and synchronization

**V:** each reference has 26 columns on every data row. Pose is the first eight (`t x y z qx qy qz qw`); extra columns are not treated as an independent covariance. This agrees with read_poses() in tools/nav_bench/pose_eval.py. Both reference timestamps are strictly increasing. Median and largest deltas below are descriptive reference statistics, not LiDAR/IMU gap certification.

| Dataset | Rows | First / last reference epoch s | Median delta s | Largest delta s (bracket) |
|---|---:|---|---:|---|
| eco01_1x | 5099 | 1780999550.039071 / 1781000071.338798 | 0.099998951 | 0.299618006: 1780999925.439755–1780999925.739373 |
| hospital_02_r05b | 6171 | 1781693278.238954 / 1781693911.239516 | 0.099992990 | 0.300687075: 1781693582.839180–1781693583.139867 |

| Dataset | Bag start UTC / epoch s | Bag end epoch s | Reference start minus bag start s | Bag end minus reference end s |
|---|---|---:|---:|---:|
| eco01_1x | 2026-06-09T10:05:49.694612+00:00 / 1780999549.694612 | 1781000072.279938 | 0.344459 | 0.941140 |
| hospital_02_r05b | 2026-06-17T10:47:57.948574+00:00 / 1781693277.948574 | 1781693912.232606 | 0.290380 | 0.993090 |

**V:** references use Unix-epoch-scale seconds, not relative seconds, and lie inside the metadata bag intervals. Existing node.log first-IMU values (eco 1780999549.694325; hospital 1781693277.948282) also use this scale and precede storage starts by about 0.000287/0.000292 s. **I:** missing reference coverage at beginning/end is compatible with initialization/trailing data; it does not establish a constant time offset and must not be subtracted to force alignment.

**NV-P:** exact reference-to-CustomMsg/header/timebase/last-point and IMU-header correspondence is NOT conclusive from metadata/logs. Bag bounds summarize storage timestamps over both topics, not each message's represented sensor time. Keep protocol reference offset 0 unless independent evidence establishes otherwise. In 5B, inspect header/timebase/point-span timing at beginning/middle/end, match reference samples to scan-end times, and distinguish timebase conventions from storage/header delay. Never estimate a correction by minimizing candidate errors. Reference gaps near 0.30 s exceed the approved 0.20 s interpolation bracket; future windows crossing them must be flagged/excluded by the evaluator. No eligibility gap mask was applied to the estimate below.

## 4. Reference-only stationary estimate — preliminary, not certified windows

**V, computation performed locally:** only copied alidarState.txt and its prior-map YAML were used. No IMU samples, bag messages, gap checks, split calculation or candidate results entered this estimate. **I, precisely defined estimator:** scan rows in increasing timestamp order. For each possible start row, extend until any sampled 3D displacement from that anchor exceeds the level bound or the quaternion geodesic angle exceeds the orientation bound. Normalize quaternions; angular distance is `2*acos(clamp(abs(dot(q_start,q)),0,1))`. If duration >=5.0 s, retain the maximal anchored interval and restart strictly after its last row; otherwise advance the start by one row. Thus retained intervals do not overlap; this greedy estimate is not a claim to enumerate all maximal possible stationary intervals. Counts can change with actual sensor boundaries, IMU certification and gap exclusions.

**I, 5 m grouping convention for this estimate:** transform anchor positions by each YAML T_map_world; join anchors <5 m apart in an undirected graph; count connected components. This conservatively merges chains and avoids calling nearby revisits independent. These are provisional spatial groups, not 30/73 certified independent trials or grid-cell counts; only anchors were grouped. The final design checks temporal input overlap; it introduces no undefined geometric footprint-merging rule.

| Dataset | Level | Distinct retained intervals >=5 s | Total retained duration s | 5 m anchor groups |
|---|---|---:|---:|---:|
| eco01_1x | A | 2 | 28.600065 | 2 |
| eco01_1x | B | 3 | 33.999940 | 3 |
| hospital_02_r05b | A | 7 | 69.296570 | 2 |
| hospital_02_r05b | B | 7 | 70.493580 | 2 |

Individual retained intervals below are offsets from the METADATA bag start, for readability only. Generator outputs must use sensor epoch time, not these offsets. Durations include the complete 2 s conditioning + >=3 s S2 exposure; total duration is not available exposure alone.

| Dataset | Level | Offset interval s | Duration s | Provisional spatial group |
|---|---|---|---:|---:|
| eco01_1x | A | 0.344459–5.544617 | 5.200158 | 0 |
| eco01_1x | A | 99.044609–122.444516 | 23.399907 | 1 |
| eco01_1x | B | 0.344459–5.644588 | 5.300129 | 0 |
| eco01_1x | B | 98.844688–122.444516 | 23.599828 | 1 |
| eco01_1x | B | 305.044764–310.144747 | 5.099983 | 2 |
| hospital_02_r05b | A | 208.590415–220.086677 | 11.496263 | 0 |
| hospital_02_r05b | A | 244.291075–253.792229 | 9.501154 | 0 |
| hospital_02_r05b | A | 355.290563–374.090497 | 18.799934 | 0 |
| hospital_02_r05b | A | 405.590788–411.990480 | 6.399692 | 0 |
| hospital_02_r05b | A | 437.890532–447.090643 | 9.200111 | 0 |
| hospital_02_r05b | A | 479.090812–485.490628 | 6.399816 | 0 |
| hospital_02_r05b | A | 559.890989–567.390589 | 7.499600 | 1 |
| hospital_02_r05b | B | 208.490559–220.086677 | 11.596118 | 0 |
| hospital_02_r05b | B | 244.191150–253.792229 | 9.601079 | 0 |
| hospital_02_r05b | B | 355.290563–374.190511 | 18.899948 | 0 |
| hospital_02_r05b | B | 405.390858–412.190558 | 6.799700 | 0 |
| hospital_02_r05b | B | 437.690391–446.990673 | 9.300282 | 0 |
| hospital_02_r05b | B | 478.990817–485.490628 | 6.499811 | 0 |
| hospital_02_r05b | B | 559.593947–567.390589 | 7.796642 | 1 |

**I:** neither A nor B supplies evidence of 30 independent S2 blocks in either dataset under this estimate; repeating starts within long pauses cannot repair that. This supports an anticipated coverage shortfall, not a finalized inventory, an automatic switch to B, a changed sample target or a claim about S1/S3 counts. The actual method-blind inventory must apply timing/IMU/gap checks and split losses before deciding the fixed A/B level. Keep failed coverage visible and request additional data only in a later authorized step.

Estimator identity for traceability: disposable local Python/stdlib script SHA-256 `6f5615bac102acaf02329354ec585b30ecf5a366975c9f69cbc031607fd842be`. It is not generator code and is not added to Git; its exact selection/grouping semantics are specified above.

## 5. Proposed inputs, outputs and information boundary

**I — design specification; not implemented:** inputs are dataset descriptors for the verified raw bag, metadata.yaml, reference and map YAML/PGM; frozen protocol/config; a per-message timing/IMU/gap index to be produced in 5B. Prior clouds belong to candidate inputs but need not be decoded by this generator. Read the first eight reference columns only as evaluator poses; reject malformed/non-finite rows, invalid quaternions and timestamp regressions. No implicit identity transform, guessed time offset or reference-derived uncertainty.

### Three isolated domains

| Domain | Permitted information and role | Boundary |
|---|---|---|
| evaluator_only | alidarState.txt/truth for offline trial selection, true-pose labeling, real stationarity, spatial grouping/independence, split, retrospective physical S3 limits and evaluability | No reference file/lookup, pose, orientation, reference timestamp, selection criterion, true radius/path, group/split or prior evaluator result is passed to the algorithm or online stop logic |
| executor | Orchestrates the selected trial with supplied raw LiDAR/IMU and original timestamps, permitted FAST-LIO, fixed scenario parameters and clock/timeout | It starts from the first supplied legitimate sensor input, counts permitted exposure/conditioning and uses sensor-only odometry. It does not read the evaluator manifest/reference or use truth-derived cutoff/stop reasons |
| method | Candidate global relocalization algorithm; receives actual sensor messages with their original timestamps, permitted sensor-only odometry, static calibration, prior map and predeclared scenario parameters | No selected-window bounds as truth hints, reference timeline, pose seed, saved initial pose, spatial group, split, true-motion feedback or evaluator history. Acceptance uses only candidate observables |

**I:** admin is private bookkeeping, not a fourth localization domain. The offline scheduler binds a selected input source to an opaque trial token; it does not send its reference-derived selection rationale or precomputed S3 ending to executor/method. Selecting an experiment offline with reference is permitted; using that knowledge online to decide when to stop or accept is not. The executor receives the selected raw stream, not a truth-conditioned stop command. The method cannot inspect the source bag/reference archive or a timestamp-to-pose lookup. Original ROS headers within selected observations remain legitimate sensor timing, even though experiment selection was performed offline.

### Timestamp provenance

- **A, legitimate:** original ROS header.stamp and CustomMsg timebase/point offsets, read from actual messages. No timestamp is replaced with a reference timestamp; their exact timing convention remains NV-P until 5B.
- **B, legitimate:** fixed conditioning/exposure/path/radius/time budgets from protocol/config; execution boundaries derive from the actual first permitted sensor input plus these budgets, clock and permitted odometry, never from truth. A budget is not a location-dependent adaptive hint.
- **C, evaluator_only:** window bounds selected using reference stationarity, reference-derived feasibility prefixes, true-pose interpolation times, actual physical checks and split boundaries. Store them privately, even if their numeric values coincide with an original sensor timestamp. Renaming such a bound cannot make its provenance legitimate for online control.

**I — outputs:** private `windows.csv` and `windows.metadata.json`, plus an exclusion log. No generator CSV column is directly exposed to method. Executor execution records use the separate legitimate provenance below; their future generation is not performed in point 5. Metadata records schema/generator/protocol/config/input identities, all A/B inventory counts, gap/units conventions and exclusions. No raw sensor payload belongs in the CSV.

| Field(s) | Domain | Provenance / justification |
|---|---|---|
| window_id, dataset_id, scenario, variant_id, exposure_config_id, input_scan_ids_hash | admin | Private reproducible inventory identity; not algorithm hints. Executor receives only an opaque trial token and the fixed scenario configuration |
| t_imu_start, t_input_start, t_input_end, t_imu_end, t_conditioning_end, t_fit_end, t_verify_start; corresponding named *_ns fields | evaluator_only | Offline selected/planned bounds in sensor coordinates, potentially chosen with reference; never online cutoff signals. Implemented S3 `t_input_end` is null pending actual execution; its private feasible prefix has a separate `evaluator_only_prefix_end_ns` field |
| stationarity_level, imu_certified, imu_check_status, lidar_gap_flag, imu_gap_flag, reference_gap_flag, timing_verified, eligibility_status, exclusion_reason | evaluator_only | Offline certification/validity; yes/no plus unchecked/pass/fail avoids equating untested with measured failure |
| spatial_block_id, temporal_block_id, effective_block_id, split, independent_block_count, duplicate_family_id | evaluator_only | Reference-linked grouping and split; no candidate access |
| evaluator_only_max_radius_m, evaluator_only_path_m, evaluator_only_duration_s, evaluator_only_stop_reason, evaluator_only_prefix_end_s, evaluator_only_prefix_end_ns | evaluator_only | Offline reference feasibility/post-check evidence, not operational stopping information |
| evaluator_only_ref_x_m, evaluator_only_ref_y_m, evaluator_only_ref_z_m, evaluator_only_ref_qx, evaluator_only_ref_qy, evaluator_only_ref_qz, evaluator_only_ref_qw, evaluator_only_ref_pose_time_s, evaluator_only_ref_pose_time_ns | evaluator_only | Truth pose/time solely for selection/scoring |
| evaluator_only_ref_stationarity_displacement_m, evaluator_only_ref_stationarity_angle_deg, evaluator_only_ref_bracket_gap_s | evaluator_only | Reference certification and scoring validity |
| evaluator_only_gyro_norm_p95_rad_s, evaluator_only_gyro_norm_max_rad_s, evaluator_only_accel_variance_sum_m2_s4, evaluator_only_imu_count | evaluator_only | Offline IMU certificate, not extra method input; raw IMU remains legitimate |
| imu_units_source_hash | admin | Private provenance of verified units/conversion; actual static conversion is fixed configuration |
| planning_radius_m, hard_radius_m | evaluator_only | Private reference feasibility cap 2.8 m and physical validity cap 3 m; neither is an adaptive algorithm input |
| scenario_config_sha256, source_manifest_sha256, generator_version, schema_version, protocol_commit, seed | admin | Versioned reproduction metadata; seed=null when selection is nonrandom |
| executor_input_start_ns, executor_input_end_ns, executor_imu_start_ns, executor_conditioning_end_ns, executor_stop_reason | executor | Separate future execution records: actual consumed original sensor times and fixed-budget/clock/sensor-odometry stopping, never copied from evaluator bounds |

**I:** method receives messages/headers and approved fixed configuration through its input API, not any row of either private manifest. Offline fit/check bounds remain private inventory metadata; executor/internal method partitioning derives the same fixed exposure fractions from sensor input and approved configuration. Reference-derived bounds are not provided as a timestamp-only candidate view. Registration queries cannot inherit reference XY/yaw, reference levelling, saved pose, --reference-csv or preceding evaluator results. Reinitialize candidate state between trials as mandated by point 4.

**I:** inventory rows are not independent attempts. Shared observations/variants/seeds remain one family/block. No generated pose goes to /initialpose; an accepted candidate's later AMCL handoff is point 11.

## 6. Algorithms — design only

### Common preparation and certification (requires 5B sensor indexing)

1. **I:** validate reference order/schema, map T_map_world and static calibration provenance. Preserve raw timestamp strings/ns. Check reference/storage scale as above, then establish sensor-header/scan-end correspondence using actual messages; do not translate all reference timestamps by their first-row delay.
2. **I, protocol-derived:** form continuous eligible timelines from valid reference brackets <=0.20 s and marked raw LiDAR/IMU gaps. Gap definition is delta >1.5 times each topic's REAL median header cadence. Storage cadence and whole-bag mean rate do not substitute. Preserve duplicate/regressing topic stamps as explicit invalidity; do not count duplicates toward cadence or certification. A new candidate window after a gap needs fresh 2 s continuous conditioning.
**I, protocol-derived interpolation rule:** an exact valid reference sample is used directly. For t_a < t < t_b, interpolate only when t_b-t_a <=200,000,000 ns (0.20 s); equality passes. An oversized bracket makes the affected query excluded/not_evaluable, with reference_gap_flag and recorded reason. No extrapolation, last-pose hold or nearest-sample substitution. A window crossing the missing interior cannot be continuously certified even if its endpoints are exact valid samples. An evaluator may describe a valid S3 prefix before that gap, never concatenate later data. That private prefix must not become an executor/method stop signal: subsequent actual consumed data are post-checked for evaluability. Exact reference-to-CustomMsg correspondence remains NV-P pending 5B message inspection.

3. **I:** inventory A and B before candidate runs, using maximum 3D reference displacement and quaternion orientation change from interval start, plus the prescribed IMU certificate. No yaw-only shortcut, cumulative-drift chaining, smoothing that hides motion or threshold changes based on localization results. Reject interpolation crossing bad brackets. The reference-only 5A estimate does not implement this preparation.
4. **I, existing unvalidated thresholds from B2:** raw gyro norm p95 <=0.02 rad/s, maximum <=0.05 rad/s; sum of per-axis sample acceleration variances <=0.04 (m/s²)², N-1 denominator. Verify units independently before conversion. Existing mapping config says imu_acc_unit_is_g=true and node.log reports mean_acc_norm around 0.996; this is evidence for checking g units, not permission to assume every recorded Imu value is SI. If in g, convert acceleration to SI before computing variance (variance scales by g²). Require finite samples and actual coverage/bracketing; no reference-derived bias subtraction. Fix quantile implementation in the future schema (propose linear interpolation), and log raw statistics/sample counts.
5. **I:** use the same frozen certification and validity masks for all compared methods. Choose A for each dataset/scenario/exposure cell; use B only if A's method-blind inventory after split/guards supplies <30 independent blocks, as mandated. Build both inventories before outcomes, freeze and record the choice. If B remains <30, label insufficient_independent_coverage; do not fabricate new starts. A/B counts in section 4 are not this final decision.

### S1 / S2: stopped conditioning plus observation

**I, existing B2 decisions:** primary S1 is stopped, exactly one livox_ros_driver2/msg/CustomMsg, nominal frame duration approximately 0.10 s (actual point span recorded), no deskew in the principal case, and 2.0 s preceding IMU conditioning with no preceding LiDAR history. IMU-deskew/prewarmed variants remain separately named as in the protocol; no translational deskew enters the principal case. Use a fixed, seed-independent spatial/beam partition of that same frame: 70% fitting, 30% internal geometric checking. This is correlated, weak verification; S1 remains the limiting performance case/floor, not independent multi-scan evidence. Method/internal adapters implement that fixed partition later, not this inventory. Private S1 t_input_start is the frame first-point time, t_input_end is its exclusive last-point bound, and t_imu_start=t_input_start-2.0 s; include valid IMU bracketing through scan end. Private S2 t_input_start is the first permitted LiDAR frame start after conditioning; t_input_end bounds 3.0 s main exposure (1.0/5.0 s separately named ablations). Include only complete frames whose point spans lie inside the cap; log the actual exposure and any small boundary shortfall. Confirm reference/IMU stationarity over conditioning AND exposure. Do not treat the original sensor header as scan end without verification.

**I:** S2 fits the first 2 s and checks the final 1 s of the main exposure, preserving source scan identities. The 1 s ablation cannot satisfy the protocol's >=1 s distinct-batch confirmation span; keep it a diagnostic/native-policy variant. The generator identifies fit/check boundaries but never registers scans or scores acceptance. Strict S1 never consumes pre-window LiDAR; no hidden FAST-LIO warm cloud.

**I — proposed non-overlap rule:** enumerate possible starts at actual frame boundaries in chronological order, choose the earliest fully eligible start, and advance to the first start whose COMPLETE support interval (conditioning through last observation) is strictly after the selected interval. Repeat within long pauses. Thus the primary S2 3 s family consumes roughly 5 s per distinct non-overlapping selection, not one new independent attempt every 0.1 s. Align variants to a common anchor and reserve the largest family support interval (e.g. 2+5 s for S2 variants); count the family once and keep all variants in the same block/split. Apply the same principle to S1; shared-location non-overlapping scans are still correlated spatially. This deliberately trades start count for distinct evidence. Do not calculate this final catalogue in 5A.

### S3: cold stationary start and causal envelope

**I, protocol-derived:** t_imu_start=t_input_start is the first allowed input for a fresh FAST-LIO instance; initial 2 s must be nearly stationary under the selected A/B level and IMU certificate. Extend stopped initialization to actual readiness if required, up to 5 s. Readiness is runtime evidence, not a property inferable from a SLAM trajectory. The offline window inventory can declare an initialization opportunity, not that FAST-LIO succeeded. All consumed warm-up LiDAR, motion and time count inside the 30 s exposure cap.

**I — evaluator_only feasibility diagnostic:** from the offline selected start, compute reference XY path/radius and the first continuous feasible prefix under the requested planning caps (2.8 m, 6 m, 30 s) and valid brackets. Record its full private provenance and limiting cause. This is NOT a supplied-stream ending, executor timeout, runtime stop reason or acceptance cue. No truth-derived clipping, EOF or reference-gap cutoff is injected into method input. Never stitch later returns into the diagnostic. The executor instead consumes available legitimate input until its independently fixed budgets, clock or sensor-only odometry dictate stopping; the evaluator assesses what was actually consumed afterwards. No prefix calculation is performed now.

**I, protocol-derived:** executor operational stopping uses the permitted trial FAST-LIO XY radius at 2.8 m, local accumulated XY path at 6 m, the 30 s input budget and fixed wall timeout; it never reads alidarState.txt. Reference subsequently checks actual physical radius <=3 m and path <=6 m, including conditioning. Violations/non-evaluable truth coverage are logged without rewriting the input or disguising an online oracle as a prefix. Original sensor-message times stamp actual consumption. Private planned bounds and actual executor records remain distinct even when equal. No FAST-LIO/odometry execution is authorized in 5A/5B by this design; later front-end runs need explicit scope authorization.

**I:** `S3_continuous_rebased` is a separately named, less faithful boot variant. Its descriptor can reference an already produced sensor-only continuous odometry artifact with run/config identity; adjust the local gauge at the window start without reference. Do not manufacture that artifact or silently substitute it for cold S3 during this phase. A nearly stopped start remains the primary S3 eligibility rule. Without an existing continuous artifact, variant availability stays NV-P.

**I — non-overlap:** select cold-S3 starts chronologically, reserving the maximum possible complete sensor-support interval allowed by the fixed scenario budget before choosing the next, not an earlier truth-derived diagnostic ending. Overlap across scenario variants is allowed only as a declared shared family/block, never as additional independent evidence. Primary per-cell windows do not overlap in their consumed IMU/LiDAR support. Runtime timeout/failed readiness on intact input remains a method outcome, not a retroactive window exclusion.

### Spatial/temporal grouping and fixed split

**I — grouping proposal:** transform evaluator-only anchors to map XY. Across all candidate windows/variants of a dataset, link anchor pairs <5 m apart and take connected components; repeat visits/headings share a spatial_block_id. This conservative graph avoids 5 m grid-boundary artifacts and can merge a long chain into a large block: report the lost coverage, never claim a compact 5 m diameter or statistical independence merely because there are multiple IDs. Audit that distinct components' anchors are >=5 m apart. No additional physical-window-footprint rule is used: it is removed rather than left undefined. Temporal reuse is handled by the explicit message/support overlap rule below. Spatial separation alone is not proof of statistical independence.

**I — temporal grouping proposal:** connected components of overlapping complete sensor-support intervals define temporal_block_id, across scenarios/variants/seeds. Boundary-sharing frames/IMU samples count as overlap. If a temporal component links spatial components, merge their effective evaluation grouping for split/independence accounting. Publish both IDs and effective block IDs. Do not create 100 nominally distinct starts by changing headings, seeds, timestamps or repeatedly selecting the same pause.

**I — deterministic split detail:** build the union of A/B candidate families and all declared scenario exposures before choosing a level, so variants cannot migrate between splits. Assign each component the lexicographically smallest member window_id as its stable identity; sort spatial/effective groups by (earliest input_start_ns, stable identity); assign the first ceil(0.30*group_count) eco groups to tuning, remaining eligible groups to final. Any group touching tuning stays wholly tuning, including later revisits. All hospital groups are heldout (final evaluation use, no tuning). This specifies rounding only; the approved 30/70 group policy is unchanged and exact retained proportions can change with guards/exclusions.

**I:** enforce the decided 35 s temporal guard using complete support intervals: no final window may have any consumed input within 35 s of any tuning support interval, including later tuning-group revisits. Remove conflicting final windows and record the reason, then recount usable blocks; do not move them silently into tuning or rebalance using method outcomes. Other variants at the same anchor share split and block. Never use hospital outcome/geometry success to tune gates. The A/B inventory selection is outcome-blind; freeze it with grouping/split definitions before evaluation. If chain merging/guards remove an evaluation cell, report unavailable coverage, not an alternate split.

## 7. Invalidity risks and detection

| Risk | Detection / required action — I design |
|---|---|
| Storage and sensor time differ; wrong epoch/relative interpretation | Compare header/timebase/point spans/reference matching at several times, preserve storage stamps, document conventions. No first-row alignment guess or acceptance-error fitting |
| Reference reset/jump or wrong quaternion/frame | Strict order, finite normalized quaternion checks; record all abnormal pose deltas for review. A jump threshold would require separately justified criteria; do not invent a reference repair or discard difficult motions from candidate outcomes |
| Reference interpolation crosses sparse samples | Mark >0.20 s brackets; no extrapolation. Largest observed brackets above already warn of this risk |
| LiDAR/IMU gap or initialization after a gap | Real topic-median gap mask, IMU bracketing and fresh conditioning; scan point spans must also remain inside the valid timeline |
| Acceleration in g misread as SI | Verify units from messages/driver/config; record conversion. Compare raw acceleration norms only as supporting unit diagnostics, not a fabricated calibration |
| Duplicate frames/windows or overlapping conditioning | Stable message IDs including storage order; duplicate-family hash; interval overlap audit on full consumed input, not LiDAR exposure alone |
| Close starts split into distinct groups | All-pairs/group boundary distance audit and revisit linking; grid cells alone do not guarantee >=5 m separation |
| Tuning/final leakage through revisits/overlap | Union grouping before split; full-support 35 s guard; no shared frame/IMU IDs across tuning/final |
| Height inferred from reference used for query crop | Require independently documented fixed calibration; 0.532/0.534 m in existing mapping reports are reference-derived, not independent query calibration |
| A long stationary segment produces many misleading independent attempts | Publish starts, pauses, spatial/time/effective blocks separately; protocol's 30-block/73-start evidence bars remain unchanged |
| Generator cache changes across input revisions | Hash small inputs/configs and freeze snapshot IDs; bag hash pending remains explicit, not an empty digest mistaken for verification |

**I:** provenance/input failure is logged outside scored valid trials; a candidate's failure on certified intact input is not excluded. Final truth_condition labeling cannot be guessed from stationary movement or map free-space status.

## 8. Determinism, code location and future unit tests

**I — technical serialization/selection specification, not new scientific criteria:**

- Dataset IDs are exactly eco01_1x and hospital_02_r05b, in that declared order. scenario is S1/S2/S3; variant_id names primary, the declared S2 durations or S3_continuous_rebased, with the complete ordered variant list fixed in versioned config. Within a dataset, order rows by (input_start_ns, scenario, variant_id, window_id); sensor records by (original sensor time_ns, relative bag filename, stable storage/message ordinal). Duplicate/regressing stamps are invalid, not silently reordered into a repaired sensor stream.
- Selection/consumption intervals use [start_ns,end_ns). An observed frame's closed first/last point support is represented as [first_point_ns,last_point_ns+1). That added ns is a representation convention, not another physical sample or permission to exceed the duration cap. Budget comparisons use actual point times/span. Adjacent half-open intervals do not overlap unless they share a consumed/bracketing message ID; shared bracketing IMU samples count as reuse. The next primary selection is the earliest eligible frame boundary with disjoint complete sensor support and message IDs; no hidden history is dropped from this check.
- Integer nanosecond names are t_imu_start_ns, t_input_start_ns, t_input_end_ns, t_imu_end_ns, t_conditioning_end_ns, t_fit_end_ns, t_verify_start_ns, evaluator_only_prefix_end_ns and evaluator_only_ref_pose_time_ns, plus executor record names in section 5. Original ROS sec/nanosec conversion is exact. Parse reference decimal seconds exactly to integer ns where representable, retaining original text/precision; reject unsupported finer precision instead of silently rounding. *_s companions are exact decimal strings derived from ns (nine fractional digits), never independent binary-float clocks. No unknown driver offset-time unit is assumed.
- Inclusive scientific bounds pass at equality: stationarity A/B, 0.20 s interpolation bracket, 3 m physical radius, 6 m path and 30 s acquisition. A raw topic delta is a gap only when strictly >1.5*median. Spatial anchors link only when distance <5 m, so equality remains separated. Executor local stopping triggers when permitted odometry first reaches >=2.8 m or >=6 m; no additional input is requested after that detected crossing. Post-checks record overshoot rather than use truth to hide it. The temporal guard permits exactly 35 s separation; no final support may be nearer. Preserve full-support IMU bracketing at borders and record excluded boundary frames.
- For S1, the private inclusive last-point time is input_end_ns-1; pose geometry refers to that scan end, not its serialization-exclusive bound. For main S2, fitting/checking meet at first_input_ns+2,000,000,000: the boundary belongs to checking, never both. A frame spanning a fit/check boundary is assigned to neither partition and recorded as a boundary exclusion, without concealing exposure loss. Other fixed exposure fractions use the same integer/rational rule, with a nonintegral ns boundary rounded down and that convention logged. This does not change the 3 s or 1/5 s budgets.
- Spatial IDs are sp-<smallest member window_id>; temporal IDs tp-<smallest member window_id>; effective IDs ef-<smallest member window_id> after transitive union. Temporal edges require overlap of half-open full support intervals or shared consumed/bracketing message IDs. Component traversal order cannot change these IDs. The eco tuning count is ceil(3*group_count/10), before guard exclusions; ties use the IDs as above. Recount coverage after exclusions, without adjusting the split from method results.
- window_id = w-<full lowercase SHA-256 of canonical identity JSON>. Identity keys are dataset_id, scenario, variant_id, stationarity_level, input_scan_ids_hash, all seven private *_ns window bounds (null when inapplicable), source_manifest_sha256, scenario_config_sha256, generator_version, generator_source_sha256, schema_version and protocol_commit. The implemented extra level/source keys ensure an A and B row or a changed generator source cannot retain the same identity. Canonical JSON is UTF-8, sorted keys, compact separators, no whitespace/newline, no NaN, integers for ns and null for unavailable values; no pose/XY/yaw is embedded. IDs stay private despite their opaque form. input_scan_ids_hash covers the ordered stable source message-ID list in that same canonical encoding.
- source_manifest_sha256 covers declared dataset identity, ordered source filenames, sizes and original modification-time precision, metadata/reference/map digests, bag_digest_status and bag_sha256 (null while pending). Size/mtime/metadata are not proof of immutable bag payload. Config records all parsing, grouping, ordering, boundary and numerical-runtime conventions; freeze generator version/revision (including dirty-source digest), schema_version, protocol_commit and config digest. No change to thresholds/minima is implied. Exact regeneration requires the same input bytes/config and pinned numerical runtime, not just identical human-readable paths.
- No randomness is used for the proposed chronological selection; seed is null and random selection is disabled. Only a separately declared future random component would require a recorded seed and RNG/version; it cannot increase independence. This replaces the unnecessary proposed seed 0.
- CSV is UTF-8 with fixed versioned column order from section 5 (expanded names), LF newlines, decimal integer ns, exact *_s strings, booleans true/false and empty nullable cells. Private reference XYZ/quaternion/statistics use a versioned round-trip numeric format; non-finite values are invalid. JSON uses sorted keys, canonical identity encoding and typed null/integers; it includes windows_csv_sha256, row count and input/config identities, computed after serializing CSV. It excludes run wall-clock timestamps/hostnames from canonical inventory bytes; execution logs are separate. CSV rows and JSON config must share schema/version/input hashes. Any byte/hash disagreement is a validation failure, not silently repaired.

**I:** the future generator must return eligibility_status=insufficient_independent_coverage for affected cells and retain descriptive counts. Thirty-block minimum, A/B levels and statistical criteria remain unchanged; long pauses/variants/seeds cannot manufacture independence.

**V, implemented after 5A:** `tools/nav_bench/window_inventory.py` and `tools/nav_bench/test_window_inventory.py` use the proposed location; `window_sensor_index.py` reads raw timing and IMU, while `window_contract.py` defines a narrow future method packet/config contract. The generator remains evaluator-side and separate from pose_eval CLI. No generic authorization exists to import legacy reference helpers into a query frontend. This paragraph records point-5 implementation; the design-only statements above remain the prior 5A specification.

### Explicit conceptual whitelist for legacy helpers

**V, source inspected / I, domain restriction:** wrap and quaternion/matrix conversions (quat_to_mat, mat_to_quat) are pure argument-based mathematics; read_map_yaml is neutral prior-map metadata reading. They may be reused only with legitimate sensor/calibration/prior-map arguments for method preparation, never truth-derived orientation or levelling.

**I — evaluator only:** read_poses on reference; planar/cells operating on reference; truth interpolation with the explicit <=0.20 s bracket/exact-sample rules above. Results cannot enter executor stop logic or method preprocessing. The legacy interp() has no internal gap check and is not admissible unchanged; a gap-aware evaluator wrapper/replacement is required later, not implemented now.

**V / I — forbidden query preprocessing:** run_bench_a.sh; initial_pose; level_quat; reference_map; reference-derived init/initpose/level/check flows; --reference-csv; saved initial pose; any truth initialization/height/orientation/levelling; map_visibility.bag_scans() reference joining. No direct or indirect call through another adapter makes these paths admissible. Reference-map transformations for offline evaluation remain separate from preparing sensor-only query geometry.

**I — 5B unit test plan, before implementation:**

- Strict schema/order/finite checks; 26-column input with first-eight pose semantics; quaternion sign equivalence and full 3D angular thresholds at equality.
- A/B anchored displacement versus slow cumulative drift; exactly 5 s, just-short, interrupted and long pauses; no adaptive level choice using method outcomes.
- Distinct scan timing versus header/storage/timebase; epoch ns and offset confusion; reference bracketing limit and no extrapolation.
- IMU SI/g conversion (variance g²), gyro norm/quantile and sample variance; empty/non-finite/insufficient coverage; unchecked versus failed certification.
- Gap factor equality/boundary, topic-specific medians, duplicate/regressing stamps and conditioning after a gap.
- S1 exactly one frame/no hidden LiDAR warm-up; S2 complete conditioning/exposure/fit/check intervals and named 1 s diagnostic.
- S3 first radius/path/time crossing, no re-entry stitching, complete-frame cut, cold start versus explicitly rebased variant; no radius or path computed for a method from reference.
- Full-support non-overlap, shared boundary-message detection, duplicate families; 5 m group edge/boundary/chain/revisit behavior; shared variants remain one block.
- First 30% group rounding, revisit-group ownership, full 35 s guard, all hospital heldout, zero eligible final groups retained as a valid shortfall report.
- Byte-stable IDs/order/manifests under identical inputs; changed hashes/configs alter identity. Method receives no manifest columns or selected-window cutoffs; only original sensor timing and approved fixed inputs. Executor stopping never consults evaluator truth.

## 9. What 5B would read, costs, and what remains outside point 5

**NV-P / I — 5B work proposal, not executed:** header/timebase/offset timing and IMU values cannot be certified from metadata. A streaming bag pass would inspect the 5,110/6,188 LiDAR frames and 104,507/126,851 IMU messages, preserve only small timestamp/IMU-statistic indices, and read scan point-time offsets only to establish each frame span. It would not replay ROS, register clouds, execute FAST-LIO or copy bags to the laptop. Depending on MCAP indexes/chunk layout, reading only small timing fields may still decompress/read whole chunks; metadata has no reliable throughput forecast. Total raw I/O upper planning volume is about 4.60 GB, once per full pass; propose an initial wall-time allowance of 1–5 minutes per bag on the NX (I, unmeasured), plus any decoding cost. Report actual time later; no performance claim now. Use already available readers only after a read-only availability check in the authorized next phase; missing readers are a dependency decision, not permission to install or launch Docker.

**I:** smaller beginning/middle/end timing probes may establish conventions first, but whole-topic gap/certification inventories still require the complete timestamp/IMU stream. Bag hashing adds a full read unless combined with the pass; warn/obtain scope approval before that work. No SHA-256 over the large MCAP files was run in 5A.

**V, protocol prohibition:** generator is evaluator-only. It cannot reuse run_bench_a.sh or pose_eval.py init/initpose/level paths that derive initial pose/levelling from the reference as query preprocessing. It cannot use map_visibility.bag_scans() reference joining to assemble method submaps. Registration or FAST-LIO must build query geometry from actual sensor-only input in later authorized stages. A true reference pose is for selection/scoring only, never a candidate seed, saved pose or database-tile hint.

**V, out of point 5:** truth_condition labeling (positive_unique / positive_aliased / negative_no_match), negative-control construction and testing belong to point 9. They require independent physical coverage/location/session knowledge, fixed frame correspondences where maps overlap, alias annotations and genuine outside-map observations or explicitly labelled cropped-map controls. Do not derive these truth labels from candidate success/failure or invent an outside-area scan by moving points. The manifest can reserve these fields as unassigned evaluator metadata; no control/classification is produced here. Eco03/Dynamic01 between-session provenance remains the point-4 open question, not an audit in 5A.

## 10. Handoff and stop

**I — criterio delegado, revisable por Narcís:** point 5 provisionally delivers the protocol's real-window manifest. Arbitrary cells/headings remain an additional AGENTS requirement pending clarification, not fulfilled and not assumed to mean synthetic scans; it may refer to candidate hypotheses/poses. This is not a definitive resolution. Available stationary coverage appears insufficient for S2 independent-block conclusions; do not relax the decided thresholds/counts, synthesize poses or add datasets without a later scope decision. This is a data limitation, not a reason to implement or execute more work now.

**V, historical 5A reconnaissance operations:** remote commands were read-only filesystem listings and Python reads/hashes of small selected files, run over SSH without writing a script/file on the NX. The initial broad listing accidentally enumerated PCD filenames and was truncated; subsequent collection was restricted to target/top-level small files. No PCD/bag message contents were read. No remote temporaries, installs, host changes, ROS/Docker processes or replays were created. Local disposable files lived only under `/tmp/global-reloc-5a/`; inventory.json, estimates.json, estimate.py and write_report.py plus that directory were removed after report review; this Markdown is the sole intended repository change. Points 01/02 and all code remain unchanged. No commit/push; stop at 5A.

**V, historical 5A documentation correction:** only this Markdown was edited in that earlier correction; no NX access or generator implementation occurred then. Subsequent 5B source inspection and indexing are reported separately in the point-5 results document. The approved point-4 protocol was not amended.

### Read-only evidence and reproducibility of this reconnaissance

- **V, local:** AGENTS.md/current and e4a34b1; protocol 31cd64f; tools/nav_bench/README.md; pose_eval.py read_poses()/read_map_yaml(). No helper CLI or test suite was run.
- **V, remote:** target directories listed with ls/find; stat values from file metadata; metadata/map YAML/reference/top-level configs/logs read with Python standard library; hashlib.sha256 applied only to reference, final PGM/YAML and metadata files. No small-file identity is substituted for a full bag digest.
- **V / I, local estimator command:** `python3 -B /tmp/global-reloc-5a/estimate.py`; input hashes, row/stat summaries, exact interval bounds and estimator semantics/hash are preserved above. The disposable script is not committed; reconstruct it from the specified row-wise anchored scan and connected-component grouping if a future independent check is needed. No final window manifest or generator implementation is hidden in this estimate.
