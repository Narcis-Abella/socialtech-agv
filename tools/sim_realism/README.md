# Simulator realism (2026-10-30)

Models of what the real Tracer + Mid-360 do, calibrated on real bags, for the Nav2 simulation. Branch `feat/sim-realism`.

| Piece | Where | What |
|---|---|---|
| `docker/ros_nav/socialtech_simreal` | built into the `sim` image with the other `ros_nav` packages | four nodes: `sim_base_model`, `sim_wheel_odom`, `sim_scan_model`, `sim_imu_model`; the models are in `models.hpp` (no ROS) with gtests |
| `calibration.json` | here | every number with its source |
| `lidar_noise_from_scan.py`, `base_response_from_bag.py`, `slip_structure.py`, `caster_kick.py` | here | the estimation from real bags |
| `sim_selfcheck.py`, `scan_selfcheck.py` | here | check the running simulation against the calibration |
| `hospital-agv-sim-realism.patch` | here | hooks in the simulator repo (branch `sim-realism`): `SIM_REALISM=1` |

Use: `SIM_REALISM=1 SIM_ARGS="odom_source:=ground_truth" tools/nav_sim/run_trials.sh ...` (REALISM_SEED, REALISM_ARGS: `outlier_scale:=0 noise_scale:=1 kick:=0.05 deadband:=0.1`).
With `SIM_REALISM=1`: `/cmd_vel` (Nav2) goes through the base model to Gazebo (`/cmd_vel_gz`); `/odom` is the wheel measurement; the LiDAR-inertial stand-in (`noisy_odom`, odom -> base_footprint TF) publishes `/odom_lio`; `/scan` and `/imu/data` come from the models.

What the data says (see `calibration.json`):
- `/scan`: gaussian core 10-25 mm (the legacy RPLiDAR model gave 25 cm at 10 m), 2-27 % dropout by range, and surface jumps on a persistent subset of the beams (the closest point of a 0.10-1.80 m band flips between surfaces).
- Base: linear dead time 60 ms + lag 100 ms, gain 1.015. Angular: 120 ms + 50 ms, gain 0.97 moving; **in place a static-friction deadband of 0.03-0.18 rad/s** (per session) is the "slip".
- Wheel odometry: reports 0.965 x the linear speed and 0.92-1.0 x the yaw rate, quantised at 0.003 m/s and 0.018 rad/s.
- IMU: ICM-40609-D from a 12 h Allan variance (static: a lower bound).
- NOT modelled / weak: the caster kick after in-place turns (7 sequences, off by default), vibration, wheel slip on slopes, the transient linear gain > 1 after in-place turns (probably a FAST-LIO artefact).

## Validated in the running simulation (2026-10-30, AGX, `sim-nav2` image)
`run_selfcheck.sh`-style run: the sim as **root** (the headless Gazebo server does not start as a normal user in this image), `world:=empty.sdf`, all times in simulation time (real-time factor 0.95):
whole chain /cmd_vel -> base model -> Gazebo: linear dead time 0.060 s, lag 0.100 s, gain 1.015; angular moving gain 0.967, in-place deadband as drawn; wheel odometry linear 0.956 / in place 0.930 of the true twist;
Gazebo alone (/cmd_vel_gz): dead time 0.02 s, gain 1.000, so everything above comes from the models. `scan_selfcheck.py` in the hospital world: core sigma, dropout and jump rate per range bin within the real values.
Pitfalls found: a test in wall time against a simulator under real time; a command profile that drives into the walls of the hospital world.

## Livox Mid-360 (non-repetitive, `docker/ros_nav/socialtech_livox_sim`)
`livox_mid360_sim` ray casts a measured per-unit pattern table on the world triangles (Embree) and publishes `livox_ros_driver2/CustomMsg` (`/livox/lidar`) and, optionally, the driver's PointCloud2 (`xfer_format 0`, PointXYZRTLT) with the real per-point time. Every point is cast from the ground-truth pose (`/odom_gt_raw`, world frame) at the point's own time, so the output has the real motion distortion (FAST-LIO undistorts it with `offset_time`). Slots without a return are zero points, as in the driver.

| step | tool |
|---|---|
| pattern table of a unit (20 s = 200 frames, 36 MB, not in git) | `livox_build_pattern.py BAG TOPIC T0 SECONDS F_Z OUT.pat` (f_z from `livox_fit_pattern.py`) |
| world to triangles (visuals; boxes, plane, OBJ meshes) | `world_to_tris.py loyola.sdf MODELS_DIR loyola.tris` |
| run | `ros2 run socialtech_livox_sim livox_mid360_sim --ros-args -p pattern_file:=... -p scene_file:=... [-p pc2_topic:=/livox/pc2] [-p static_pose:="[x,y,yaw]"]` |
| output check vs table and real frames | `livox_sim_check.py` |
| pattern statistics vs real units and the official CSV | `livox_pattern_compare.py` |
| geometry vs Gazebo gpu_lidar, still and turning (inside the sim image) | `livox_sim_geom.py` |

Why a table: the azimuth law is not recoverable in closed form (see notes/sim_realism/calibracion_resultados.md), so the node replays what the unit did. The table wraps every 20 s.
Known limits: no moving objects in the scene (the world has none), tag always 0 (a real unit flags ~1.3 % of points), reflectivity constant, range noise 2 cm is the datasheet value (not measured), the robot body does not occlude.
