# nav_sim: the simulated Tracer

Gazebo Harmonic world (`hospital-agv-sim`, branch `sim-realism`) + the stack under test, run inside the `sim` image on a board (AGX), results analysed offline. Realism models and their calibration: `../sim_realism/README.md`.
All `*.sh` that run inside the image take the same mounts: this `tools/` at `/tools`, the simulator checkout at `$SIM_SRC`, the output dir at `/out`. The laptop-side wrapper (`agx_full.sh`) ships `tools/` and the simulator commit to the board.

## The simulation: full chain
Simulated Mid-360 (`socialtech_livox_sim`, non-repetitive) + ICM-40609 IMU 200 Hz in g -> the real FAST-LIO2 (the robot's parameter edits) -> `planar_odom` -> `pointcloud_to_laserscan` -> `/scan` (optional `SCAN_EXTRA=1`: calibrated dropout and surface jumps, a stress test). Entry points: `lio_sim.sh`, `fid_sim.sh`, `sup_sim.sh`, `lat_sim.sh`, `clock_sim.sh` (`sim_up.sh` is their common start); `nav_full_sim.sh`: the robot's own `ops/nav_robot.launch.py` (gn_loc, RPP, the real overrides) + `trials.py`; `sup_sim.sh`: gn_loc + gn_supervisor without Nav2. The 2D Gazebo simulation with AMCL (run_trials/run_eval/agx_run, worlds, params overlays, scenarios) was removed on 2026-11-08; it is in the git history before that commit.

## Full chain
- `lio_sim.sh OUT [leg v w]`: drives out and back, `lio_eval.py` reports FAST-LIO2 against the Gazebo pose (8.7 m, two half turns: xy median 2.1 cm, max 7.7 cm on the AGX).
- `fid_sim.sh OUT [s]`: robot still; the `/scan` of the whole chain against `../sim_realism/calibration.json` (`scan_selfcheck.py`) and the parked drift.
- `sup_sim.sh OUT [cover_s] [--push-after S | --move-covered M]`: start-up localization by the supervisor, lidar covered (`ros2 param set /livox_mid360_sim cover true`), uncovered; `sup_cover.py` times each step (AGX, 20 s cover: stop 4.0-4.5 s after the cover, back to MONITOR 7.0 s after the uncover). `--push-after`: the robot is pushed during the exposure (the pose must be refused); `--move-covered`: it is moved while blind (GN disagrees, the relocalizer's pose is confirmed at scan rate).
- `PROD=1` (sup): the robot as deployed (`ops/nav_live.sh PROD=1`): `viz:=false`, no rviz aid. DEV (default) keeps `live_paths.py`; `PROD=0` is off, as on the robot.
- `PROCMON=1` (sup; `SETTLE=s` lengthens the MONITOR phase): `tools/nav_bench/procmon.py` (the one CPU/RAM sampler of the repo) samples every process of the container at 1 Hz; the report (`procmon.txt`, or `procmon.py report A.csv B.csv -s scenario.json`) gives avg / p95 / max CPU (% of one core) and RSS per stack process and for the stack as a whole, per phase, with the simulator left out. Numbers are of the board that ran it: the AGX is not the Nano.
- `lat_sim.sh OUT [reps] [idle_s]` (`agx_full.sh start lat`, `CPUSET=0-5` limits the container's cores; the search input is `~/lat_input` on the board, a `runs/gn_supervisor/reloc*/input` of an earlier sup run): does a search disturb FAST-LIO? `lat_ab.py` runs the worker by hand (plain and under `nice -n 10`) with the stack in MONITOR and reports the latency and gaps of `/Odometry`. The simulator shares the container (and the cores) with the stack.
- `clock_sim.sh OUT [s]` (`agx_full.sh start clock`): what the simulated clock costs. Gazebo publishes `/clock` at every physics step (~395 Hz on the AGX with `loyola.sdf`); an idle rclpy node with `use_sim_time` took 21.4 % of a core against 0.02 % without it. Every sim-time node of the stack pays its share (a C++ node ~4 %: `map_server` idle 3.7 % in the sim, 0.9-1.5 % in the bag bench at 40 Hz), so CPU figures of the sim runs are an upper bound; the robot has no `/clock`.
- Needs on the board: `~/livox_pat/{ours.lvm,loyola.tris}` (the unit's pattern model and the world's triangles, `../sim_realism/world_to_tris.py`) and `~/greloc/build/{register_v2,libsmall_gicp.so,libcorr_search.so}` for the supervisor.

## Nav2 scenarios
`nav_full_sim.sh auto:N` (or a `scenarios.py` json made by `make_scenario.py` / `make_mission.py`) runs start/goal pairs against Nav2; `trials.py` records raw series, `analyze.py` / `stats.py` / `metrics.py` summarise and compare paired runs. `NAV2_OVERLAY=<file in tools/nav_sim/params/>` merges one extra parameter file over the robot's (no overlay is kept; write one for an experiment and delete it after).

## Tests
with the laptop's ROS sourced: `python3 -m pytest tools/nav_sim --ignore=tools/nav_sim/test_trials.py` (95 pass); `test_trials.py` needs `nav2_msgs`, so run it in the `sim` image.

## Results (AGX, 2026-11-02)
`nav_full_sim.sh auto:4` (4 start/goal pairs on the chain's own map, clearance 0.6 m): 4 of 4 reached, 25-52 s each, 0 recoveries, localization error 2-3 cm at the start. With `SCAN_EXTRA=1`: 0 of 4. Start/goal pairs 0.45 m from a wall fail instead: the footprint's corners (0.43 m) cannot turn in place there.
