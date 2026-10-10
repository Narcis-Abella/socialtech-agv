# 17 — Mount calibration: audit, parameters, physical checklist

Date: 2026-10-10. Branch `feat/global-reloc`. Labels: **V** measured / read in code, **I** inferred, **NV** not validated.
**No physical measurement was made for this note. Calibration is NOT done.** Nothing here was run on the robot.

## A. Frames that exist today (V, read in code)

| Transform | Published by | Source of the numbers | Status |
|---|---|---|---|
| `camera_init -> body` | FAST-LIO2 (`/Odometry` + TF) | LIO estimate. `body` = Mid-360 IMU frame. `camera_init` = IMU frame at LIO start, **not gravity aligned** | estimated online |
| LiDAR -> IMU (inside FAST-LIO2) | `mid360.yaml`: `extrinsic_T [-0.011, -0.02329, 0.04412]`, `extrinsic_R = I`, `extrinsic_est_en: true` | Livox factory values shipped with FAST-LIO2; refined online | factory default, **not checked on our unit** |
| `odom -> camera_init` (static, levelling) | `reloc_node.publish_level_tf` | mean IMU specific force over the submap span, `lio_submap.gravity_up` + `level_rotation` (smallest rotation, adds no yaw) | estimated per attempt, truth-free |
| `odom -> base_footprint` | `reloc_node.publish_footprint` (`planar_odom.footprint` maths) | x, y, yaw of `body` in `odom`; `z = body z - sensor_height`; then the planar mount extrinsic | **PROVISIONAL**: `sensor_height = 0.53`, extrinsic = identity |
| `map -> odom` | Nav2 AMCL | particle filter, after `/initialpose` | estimated |
| `/initialpose` (`map`, pose of `base_footprint`) | `reloc_node.tick_handoff` | accepted `T_map_src` propagated with FAST-LIO odometry (`reloc_core.handoff_pose`), then the planar mount extrinsic; covariance from `init_std_m`, `init_yaw_std_deg` | pose **measured method output**; covariance **PROVISIONAL** (0.15 m, 5 deg) |
| `/scan` | `pointcloud_to_laserscan` on `/cloud_registered_body`, `target_frame: base_footprint`, band 0.10-1.80 m | band heights are metres above the floor only if `sensor_height` is right | depends on the provisional height |

What is measured, estimated, provisional:
- **Measured (V):** nothing about the mount. No tape/CAD value of height, offset or yaw exists in the repo.
- **Estimated from data (I):** gravity direction (IMU), LiDAR-IMU extrinsic refinement (FAST-LIO2), sensor height 0.53 m (bag value, origin: maintainer's notes; note 08 could not confirm it from LiDAR because the upward-facing sensor does not see the floor reliably).
- **Provisional (NV):** `sensor_height`, base-from-sensor x/y/yaw (zero), initial covariance, Livox factory LiDAR-IMU extrinsic on our unit, time offset LiDAR-IMU (0.0).

Consequences of a wrong value (I, from the maths, not measured):
- Height error `dh` shifts the 0.10-1.80 m scan band by `dh`: obstacles near the band edges appear/disappear in `/scan`; it does not move the 3D relocalization pose (the 3D search does not use the height).
- x/y offset error moves `/initialpose` and the AMCL base pose by the same vector rotated by the heading: a 0.20 m lever arm error alone consumes two thirds of the 0.30 m limit, and a yaw extrinsic error adds directly to the 8 deg limit.
- A tilted (downward) mount: `level_rotation` removes any tilt below 180 deg, so the pose maths holds, but the body x axis used for yaw becomes ill-defined as the tilt approaches 90 deg. **NV**: no tilted data exist.

## B. Parametrization (implemented, default = behaviour of note 15)

Node parameters of `reloc_node.py` (also launch arguments of `reloc_online.launch.py`):

| Parameter | Default | Meaning |
|---|---|---|
| `sensor_height` | 0.53 | IMU origin above the floor, metres |
| `base_from_body_xyyaw` | `[0, 0, 0]` | pose of `base_footprint` in the levelled body frame: x forward m, y left m, yaw rad (`reloc_core.compose_planar`, unit test `test_mount_extrinsic`) |
| `init_std_m`, `init_yaw_std_deg` | 0.15, 5.0 | `/initialpose` covariance |

Every `metrics.json` records the values used under `mount`, with `status: PROVISIONAL_not_measured`. Change that string only when section C is filled in.
With the zero default the published pose and TF are numerically those of note 15 (the composition is the identity).

## C. Physical measurements needed (to be done by a person, robot powered off)

Record each value with instrument, resolution, who, date, and a photo with a ruler in frame. Repeat each 3 times and keep all three.

| # | Quantity | How | Target resolution |
|---|---|---|---|
| C1 | Definition of `base_footprint` | Decide and write down: midpoint of the drive-wheel axle projected on the floor, x forward | decision, not a number |
| C2 | Sensor height: floor to the Mid-360 reference plane | Tape/calliper on flat floor, chair unloaded and loaded (patient weight changes suspension/tyres) | 5 mm |
| C3 | IMU origin above the reference plane | Livox Mid-360 mechanical drawing (do not guess; cite the document revision) | from drawing |
| C4 | x, y of the sensor axis relative to `base_footprint` | Plumb line from the sensor centre to the floor; measure to the axle midpoint mark | 5 mm |
| C5 | Mount yaw: sensor x axis vs robot forward axis | Square against the chassis, or C8 below | 0.5 deg |
| C6 | Mount tilt (pitch/roll) | Digital inclinometer on the sensor housing; cross-check with C7 | 0.2 deg |
| C7 | Rigidity | Push the mount by hand; note play. A foldable chair must return to the same pose after fold/unfold: repeat C2-C6 after 3 fold cycles | pass/fail + spread |
| C8 | Sensor orientation: up or down, cable direction | Photo | - |

## D. Checks with LiDAR/IMU data (scripts exist; none run on new data)

| Check | Data | Tool | Pass criterion (define before running) |
|---|---|---|---|
| D1 tilt | 30 s stationary on level floor | `tools/nav_bench/imu_static_calibration.py` (gravity vector in the IMU frame) | agrees with C6 within 0.5 deg |
| D2 height | same, sensor seeing the floor (downward mount) | `tools/nav_bench/estimate_sensor_height.py` (floor plane by RANSAC) | agrees with C2+C3 within 2 cm; with the upward mount this check is not possible (note 08) |
| D3 mount yaw | drive 5 m straight, supervised | direction of the FAST-LIO2 displacement in the body frame vs body x | agrees with C5 within 1 deg; **needs motion: supervised only** |
| D4 lever arm x/y | rotate in place 360 deg, supervised | the IMU origin describes a circle of radius `hypot(x, y)` in `/Odometry` | radius agrees with C4 within 2 cm |
| D5 LiDAR-IMU extrinsic | any 60 s run | FAST-LIO2 log of the refined extrinsic vs the factory value | within 1 cm / 0.5 deg, else investigate |
| D6 scan band | stationary next to a wall and a table of known height | `tools/nav_bench/scan_vs_map.py`, RViz overlay | table edge appears/disappears as the height predicts |
| D7 stream health | 5 min live | `tools/global_reloc_3d/bag_stream_integrity.py` on the recording, and `ros2 topic hz` live | see note 16: today's bags already miss 6-8 % of IMU samples |

## E. After the remount (sensor looking down)

The prior maps, the 0.53 m height and every result of notes 12-16 were obtained with the sensor facing the ceiling. After the remount, in this order:
1. Section C and D1, D2, D5, D7 (stationary, no motion needed).
2. Re-record one mapping session and rebuild the map with `make_map.sh`; check `map_clean.ply` against the PGM. The 3D prior used by the relocalizer (`map.xyz`) must come from the new mount or be shown to match: a ceiling-dominated prior may not overlap what a downward sensor sees. **NV**.
3. Re-run the unit tests and one replay per mode (`s2s3`, `s3`) on the new bag over its own map: plumbing only.
4. Re-run the stationary-S2 set and the negatives of note 16 on new windows. The acceptance gate stays frozen; if it now rejects everything, that is a result to report, not a threshold to move.
5. D3, D4 under supervision.
6. Only then the independent evaluation of note 18.

## F. Open items
- Who owns C1 (the `base_footprint` definition) and the Mid-360 drawing revision.
- Loaded vs unloaded height: which one the scan band should use.
- Whether the foldable frame keeps the sensor pose after folding (C7). If not, D1/D2 must run at every boot, and the height becomes a boot-time estimate instead of a constant.
