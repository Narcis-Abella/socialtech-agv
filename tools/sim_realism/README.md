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
