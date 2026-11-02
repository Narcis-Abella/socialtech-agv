# nav_sim: the simulated Tracer

Gazebo Harmonic world (`hospital-agv-sim`, branch `sim-realism`) + the stack under test, run inside the `sim` image on a board (AGX), results analysed offline. Realism models and their calibration: `../sim_realism/README.md`.
All `*.sh` that run inside the image take the same mounts: this `tools/` at `/tools`, the simulator checkout at `$SIM_SRC`, the output dir at `/out`. The laptop-side wrappers (`agx_*.sh`) ship `tools/` and the simulator commit to the board.

## Two simulations

| | 2D (legacy) | full chain (`SIM_LIO=1`) |
|---|---|---|
| sensors | Gazebo 2D lidar `/scan` + wheel/ground-truth odometry | simulated Mid-360 (`socialtech_livox_sim`, non-repetitive) + ICM-40609 IMU 200 Hz in g |
| localization input | the simulator's `/scan` and `odom -> base_footprint` | real FAST-LIO2 (the robot's parameter edits) -> `planar_odom` -> `scan_median` -> `/scan`, plus `SCAN_EXTRA` dropout and surface jumps |
| entry points | `run_trials.sh`, `run_eval.sh`, `agx_run.sh`, `agx_queue.sh`, `agx_live.sh` | `lio_sim.sh`, `fid_sim.sh`, `sup_sim.sh` (`sim_up.sh` is their common start) |
| Nav2 | yes (`socialtech_nav`, AMCL on the PGM) | not yet: `sup_sim.sh` runs the robot's own `ops/nav_live.launch.py` (gn_loc + gn_supervisor) without Nav2 |

## Full chain
- `lio_sim.sh OUT [leg v w]`: drives out and back, `lio_eval.py` reports FAST-LIO2 against the Gazebo pose (8.7 m, two half turns: xy median 2.1 cm, max 7.7 cm on the AGX).
- `fid_sim.sh OUT [s]`: robot still; the `/scan` of the whole chain against `../sim_realism/calibration.json` (`scan_selfcheck.py`) and the parked drift.
- `sup_sim.sh OUT [cover_s] [--push-after S | --move-covered M]`: start-up localization by the supervisor, lidar covered (`ros2 param set /livox_mid360_sim cover true`), uncovered; `sup_cover.py` times each step (AGX, 20 s cover: stop 4.0-4.5 s after the cover, back to MONITOR 7.0 s after the uncover). `--push-after`: the robot is pushed during the exposure (the pose must be refused); `--move-covered`: it is moved while blind (GN disagrees, the relocalizer's pose is confirmed at scan rate).
- Needs on the board: `~/livox_pat/{ours.lvm,loyola.tris}` (the unit's pattern model and the world's triangles, `../sim_realism/world_to_tris.py`) and `~/greloc/build/{register_v2,libsmall_gicp.so}` for the supervisor.

## 2D / Nav2
`run_trials.sh scenario.json OUT [reps]` runs a scenario (`scenarios/*.json`, from `make_scenario.py`, `make_mission.py`, `make_cross.py`) against Nav2; `trials.py` records raw series, `analyze.py` / `stats.py` / `metrics.py` summarise and compare paired runs; `params/*.yaml` are overlays for `params_merge.py` / `params_variant.py`; `bt/` behaviour trees; `worlds*` (`worlds.py`, `ext_worlds.py`) test worlds. Obstacles that move: `dyn*.py`, `ghost.py`. Planner benchmark: `plan_bench.py` + `plan_analysis.py`. Goals: `eval_goals.py` (`goals.yaml`). Replay in rviz2 on the laptop: `replay_rviz.py`, `rviz_laptop.sh`. Mapping the world from known poses: `run_gtmap.sh`, `gt_mapper.py`, `gt_map.py`.
AMCL on recorded real inputs (no simulator): `record_real_inputs.sh`, `real_sweeps.sh`, `real_factorial.sh`, `agx_real.sh`, `synth_scan.py`, `scan_quality.py`, `odom_error.py`.

## Tests
with ROS sourced (`source /opt/ros/jazzy/setup.bash`): `python3 -m pytest tools/nav_sim` (`test_trials.py` and `test_eval_goals.py` import rclpy / geometry_msgs; without ROS 193 of the others pass).
