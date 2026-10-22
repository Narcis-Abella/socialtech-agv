# Sensor-height estimation and global-AMCL diagnostic

**EXPLORATORY — NOT FINAL BENCHMARK**
Date: 2026-10-08. Labels: **V** verified from source/data/run; **I** inferred; **NV-P** pending. No Point 7 candidate was run.

## Scope and evidence firewall

The purpose was to find a truth-independent `/scan` route for the first global-AMCL diagnostic. The existing calibrated `base_footprint` route could not be reconstructed for the historical upward-facing sensor mount without an independent mechanical height/extrinsic. This experiment therefore uses an IMU-levelled sensor-origin proxy, a declared height hypothesis and a sensor-relative vertical band. It is strictly DEVELOPMENT/EXPLORATORY; any settings inspected against its outcomes consume these windows for future tuning. Formal evaluation requires new reserved data.

**V:** inputs to the estimator/projection and AMCL process were LiDAR/IMU messages, the PGM/YAML map, fixed AMCL parameters and the height hypothesis. The method container had no `alidarState.txt`, Voxel-SLAM trajectory, private inventory, truth pose, split or evaluator CSV mounted. The map YAML's `T_map_world` was not used to create scan geometry. No evaluator ran after AMCL because no pose was accepted. **NV-P:** reusable OS-level process isolation and a separate post-run evaluator remain to be implemented before a full benchmark runner.

## What needs an absolute height

| Operation | Absolute sensor-to-floor height needed? | Evidence/status |
|---|---|---|
| Decode/project raw points in the sensor frame | No | **V:** sensor point coordinates are already in the Livox message frame. |
| Estimate gravity direction and level roll/pitch | No | **V:** a stationary IMU prefix supplies a robust median acceleration direction; it gives no absolute yaw or height. |
| Detect a candidate plane below the sensor | No prior height | **V:** LiDAR geometry can estimate perpendicular distance to a supported plane. **NV-P:** plane identity as floor. |
| Select a vertical obstacle band intended to match the PGM | Yes, unless a sensor-relative band is independently shown equivalent | **V:** the existing converter's band is expressed in its target frame. **NV-P:** the historical sensor height and equivalence of a sensor-relative slice. |
| Generate a sensor-origin 2D scan for a development diagnostic | A hypothesis is sufficient to generate one, but not to certify its semantics | **V:** implemented and exercised. It is not a calibrated `base_footprint` scan. |
| AMCL 2D execution | It needs consistent scan/map/TF geometry, not a separately named height parameter | **V:** the sensor-proxy frame path ran. **NV-P:** physical base-frame correctness and validated map-band correspondence. |
| Protocol 3D query verification | Yes, fixed calibrated height is required by the protocol | **NV-P:** no independent calibration is available. |
| Post-run truth scoring | Not for a sensor-only run; reference is used only after the method ends | **V:** no truth was supplied to this run; no accepted pose required scoring. |

The launch uses `livox_level` as a levelled sensor-origin proxy and a static identity relation to `odom` for this stationary diagnostic. This does not assert that the sensor origin is physically `base_footprint`; no XY/yaw robot extrinsic was invented. The global AMCL service samples the map in this proxy frame, making this a deliberately limited diagnostic of the projection and global particle behavior, not a deployable TF chain.

## Plane estimator

`tools/nav_bench/estimate_sensor_height.py` reads only `/livox/lidar` and `/livox/imu` from the bag, selected by sensor header timestamps. It forms a gravity basis from the median accelerometer vector near each LiDAR frame, bins returns below the sensor into 0.25 m horizontal cells, and uses deterministic RANSAC (`seed=0`) over cell minima. Candidate planes must be within 10 degrees of horizontal, have at least 100 inliers, 10% support among below-sensor points, support across at least 8 cells and 0.5 m in both horizontal dimensions, and 3 cm residual tolerance. These are estimator guards, not changes to the stationarity protocol.

Frames/windows sampled included eco01 S2 3 s (`1780999653438871296` to `1780999656438871296`), hospital S2 3 s (`1781693840039264256` to `1781693843039264256`) and eco03 S1/S2-eligible intervals. Candidate medians fell around 0.08–0.13 m below the sensor origin, with p05–p95 spans around 0.04–0.05 m in the reported windows; median fitted tilt was below about 1.6 degrees, and inlier ratios were about 0.26–0.47. These are stable plane candidates, but the sensor-facing-up data do not prove that the plane is the floor. The small inferred distance makes an onboard/platform surface plausible. Therefore the estimator rejects promotion to a physical floor-height calibration.

**Conclusion:** `I — sensor height estimated from LiDAR + IMU, floor identity unverified`. The user-supplied “about 1 m or more” was not used as truth; it only motivated a broad diagnostic sweep. No common mount height was inferred across sessions.

## Sensor-only height sweep

For fixed eco01 and hospital S2 3 s windows, the obstacle band `[0.10, 1.80] m` above floor was translated into sensor-relative coordinates under height hypotheses 0.7–1.6 m. Scan projection used 0.5-degree bins, 0.5–60 m ranges and the frozen gravity estimate. No map matching or reference was used to choose a height.

| Dataset | Hypothesis heights (m) | Median finite beams, low→high | Azimuth span | Observation |
|---|---:|---:|---:|---|
| eco01 | 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.4, 1.6 | 713, 711, 709.5, 708.5, 708, 707, 701.5, 685 | ~359.5° | At 1.0 m, ±0.1 m changed only a few median beams; point count in the band changed more. |
| hospital | same | 705, 706, 706, 706, 705, 704, 703, 700.5 | ~359.5° | Finite-beam count was stable across this sweep. |

**I:** projected coverage is moderately insensitive to the tested height, but beam-count stability cannot establish that the vertical obstacle content corresponds to the PGM. No height was selected by localization error, truth, or map likelihood. The 1.0 m run value is an explicit uncalibrated DEVELOPMENT hypothesis, not a measured height.

## AMCL diagnostic run

| Field | Result |
|---|---|
| Dataset/window | eco01 S2 3 s, sensor-time interval `1780999653438871296`–`1780999656438871296` |
| Input | `/livox/lidar` `CustomMsg`, `/livox/imu` `sensor_msgs/Imu`; map PGM/YAML mounted read-only |
| Conditioning | 2 s IMU prefix; 400 observed / 418 expected = 95.69%; no 4× hard gap; gravity frozen before S2 input |
| Scan | Height hypothesis 1.0 m; `livox_level`; 29 scans published to AMCL; 703–715 finite beams per scan |
| Playback | 0.25× for this diagnostic because the 1× attempt accumulated/dropped scans downstream of the converter |
| AMCL | Existing 2,000-particle configuration; `/reinitialize_global_localization`; no `/initialpose`; no parameter tuning |
| Observer | 29 post-update batches; 2,000 particles visible; fixed gate evaluated with map, scan and particles only |
| Outcome | No acceptance; maximum particle concentration 0.481 vs 0.90 requirement. The run remained without acceptance through 60 s and is classified `false_reject_timeout` for this eligible positive. No `wrong_accept`. The explicit timeout JSONL terminal-event writer was added after this run and was not exercised here. |
| Truth scoring | Not performed: no pose was accepted and no truth was read online or afterward for this attempt. |

The slower replay demonstrates sensor/message/TF/service/particle-observer plumbing, not real-time performance. Its wall latency is not comparable with a 1× trial. The 1× input attempt lost scans before AMCL could consume them and is an operational integration issue, not a localization score. AMCL's failure to concentrate could reflect the uncalibrated sensor-relative scan/map band, the sensor-origin proxy frame, global ambiguity, or the fixed particle/configuration; this run does not distinguish those explanations.

## Interpretation and next steps

- **V:** no independent mechanical sensor-to-floor calibration or applicable mount record was found in the inspected repo/bag/map assets.
- **V:** the LiDAR+IMU plane estimator finds repeatable horizontal geometry but cannot identify its physical semantic role from these ceiling-facing data.
- **I:** the sensor-relative scan path is useful to test data plumbing and explore sensitivity. It is not sufficient evidence for formal AMCL performance.
- **NV-P:** physical floor height, sensor-to-robot-base transform, equivalence of the sensor-relative slice to the map's obstacle band, 1× scan throughput and a positive AMCL acceptance.
- Acquire a new, traceable session with the final mount, record a physical/drawing-based sensor origin height and extrinsic, then build its map/reference together. Keep new final-evaluation locations reserved; do not recycle these development windows after tuning.
- Before a repeat, resolve the 1× drop diagnosis with received-vs-published scan counts at each node and queue/processing timing. Do not alter acceptance thresholds or report a formal rate.

No Point 7 method, AMCL handoff, or formal benchmark was run.
