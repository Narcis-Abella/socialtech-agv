# level_corrector

Slow roll/pitch correction of FAST-LIO2's odom frame against the levelled 3D map. **Shadow mode only**: it computes and publishes the correction, nothing consumes it yet (`planar_odom` keeps its static `odom -> camera_init`).

Why: FAST-LIO2 starts with `camera_init` = the IMU frame at start and finds gravity from ~10 accelerometer samples, so the floor is tilted in `odom` by a constant start error plus slow drift (0.3-1.5 deg on real bags). A floor tilted by θ enters the costmap height band at R = 0.10 m / tan θ (0.57 deg: 10 m). The map is levelled by `make_map.sh`, so registering the cloud against it gives the true vertical.

| | |
|---|---|
| In | `/cloud_registered_body` (PointCloud2, FAST-LIO2, IMU frame), `/Odometry`, TF `map -> odom` (gn_loc: the seed), `<map_dir>/map.ply` + `map.yaml` (`T_map_world`, written by `ops/make_map.sh`) |
| Out | `~/correction` (QuaternionStamped, frame `odom`: C, so that L' = C * L), `~/status` (String, every tick) |
| Cycle | every `1/rate_hz` s (0.5 Hz): point-to-plane ICP of the latest cloud (voxel 0.25 m) against the map (voxel 0.1 m), seeded with gn_loc's planar pose and FAST-LIO2's attitude; two passes (0.5 m, 0.15 m) |
| Gates | `min_inliers` 300, `min_ratio` 0.9 (inliers / scan points), `max_rms` 0.03 m, `max_dxy` 0.3 m and `max_dyaw_deg` 3 against the seed. A scan that fails any is dropped (`state=<gate>` in the status) |
| Filter | first valid measurement (or first after a FAST-LIO2 restart: a gap > 1.5 s or a jump > 1 m in `/Odometry`) is taken whole; later ones move C with time constant `tau_s` (10 s) and at most `slew_deg_s` (0.05 deg/s); one further than `max_jump_deg` (2) from C is dropped (`state=jump`). With no valid measurement C stays where it was: today's behaviour |

`status` fields: `state n src rms dxy dyaw conv meas_pitch meas_roll C_pitch C_roll primed restart npts cpu_ms` (angles in deg; `meas` = this cycle's registration, `C` = the filtered correction).

Parameters: `map_dir` (/map), `level` ([qx,qy,qz,qw] of the static odom <- camera_init that `planar_odom` publishes: pass the same), `rate_hz`, `tau_s`, `slew_deg_s`, `max_jump_deg`, `scan_voxel`, `map_voxel`, the gates above.

## Measured (Nano, robot parked, 15.6 min, 470 cycles)

| | |
|---|---|
| Valid cycles | 468 of 470 (2 before `map -> odom` existed) |
| Registration | n 890 (p5 855), rms 0.026 m, dxy 0.044 m; measured roll -0.68 deg (p5..p95 -0.76..-0.62), pitch +0.07 deg: the start tilt FAST-LIO2 left in this boot is 0.68 deg, above the 0.57 deg limit of a 10 m band |
| CPU of one cycle | 8.6 ms median, 11 ms p95 (0.43 % of a core at 0.5 Hz) |
| CPU of the whole process | 1.66 % of one core (the rest is receiving the 10 Hz cloud it uses every 2 s) |
| Against the offline reference (numpy ICP, 198 scans of 20261031_162508, `notes/level_corrector_scripts/bench_registrar.cpp`) | gated: median 0.08 deg, p95 0.16 deg, max 1.26 deg to the reference; 14 of 198 dropped by the gates (ratio 0.68-0.78, rms 0.042-0.048: the fits that were > 0.5 deg off) |

Not measured: behaviour while the robot moves live (the offline simulation of the same filter gives p95 0.15 deg on 5 bags, N = 2949), and how often the gates drop scans in motion.

## Build and tests

small_gicp is not an apt package: the image builds the commit `register_v2` uses (Dockerfile, `SMALL_GICP_COMMIT`) into `/usr/local`. Built with the rest of `ros_loc` (tests off). With tests:

```
colcon build --packages-select level_corrector --cmake-args -DBUILD_TESTING=ON
./build/level_corrector/test_core ; ./build/level_corrector/test_map_io ; ./build/level_corrector/test_registration
python3 test/test_level_corrector_node.py        # the real node on a synthetic room, own DDS domain
```

16 gtests (the maths mirror `notes/orientacion_scripts/maptrack.py`; golden matrices come from it) and the node test (a floor tilted 1.22 deg comes back to 0.015 deg).
