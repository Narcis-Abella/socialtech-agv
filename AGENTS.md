# AGENTS.md

Context for the AI agent (Claude Code, Codex, any) helping a team member on this repo. Read it fully before touching anything. The maintainer deletes this file and `CLAUDE.md` when the work merges.

**This file is incomplete on purpose.** Anything it does not state, or states as unverified, you find out yourself: read the source, run it, read the upstream docs and issue trackers, search the web with several broad queries. Do not fill gaps from memory. Say what you verified and what you did not.

## Project

University robotics team (IQS, Barcelona) for the SocialTech Challenge: an autonomous AGV that carries a patient through a hospital. Base: AgileX Tracer. Sensor: Livox Mid-360 (LiDAR + 6-axis IMU at 200 Hz). Computers: Jetson Orin NX 16 GB (production), Orin Nano 8 GB (candidate), AGX Orin (development). ROS 2 Jazzy in Docker on JetPack 7.2.1. Indoor, flat floor, about 0.34 m/s, no GNSS.

## Where things run

The agent and the editor run on your human's laptop, in a checkout of branch `feat/global-reloc`. The experiments run on the Orin NX over SSH. Your human gives you the SSH host name. Code reaches the NX with `git push` to your branch and `git pull` in the NX clone (`~/socialtech-agv`), or with `rsync` for quick iterations of `tools/`. Run the Python tests on the laptop in a venv with numpy. The bench itself runs in the Docker image on the NX. You may build your own image from this repo's Dockerfile with an extra layer (for example `socialtech:reloc`), install pip packages in a venv and apt packages inside containers. Do not change the `robot` stage.

## The task

Today the robot learns its starting pose from an operator, who clicks it in RViz (`/initialpose`). We want the robot to find its own initial pose in the map, 2D (the PGM) or 3D (the point cloud), with no operator, at boot. The method is open: 2D, 3D, or a mix.

Your job, with the person you work with: find out which approaches exist, pick candidates, and evaluate them on real data. We want measured results, not claims: how reliable each one is, how accurate, how long it takes, and what it costs in CPU, GPU and RAM on the Orin NX. The working hypothesis from earlier research is KISS-Matcher (global) followed by small_gicp (fine), then hand the pose to AMCL. It was never tested. Treat it as one candidate, not the answer. Verified licenses: both MIT, compatible with this repo's GPL-2.0-only. Not verified: that either builds on Jazzy and aarch64.

Other starting points, none verified here, and the list is not closed. Search for more.
- AMCL's own global localization on the PGM (`/reinitialize_global_localization`). It is a 2D method, it costs nothing, and it is the baseline to beat.
- Other global registration or place recognition: Quatro, Scan Context family, BTC (the descriptor Voxel-SLAM uses for loop closure; it also has a `previous_map` multi-session mode).
- 3D localizers that publish `map -> odom`: FAST-LIO-Localization-QN, hdl_localization, lidar_localization_ros2, als_ros, prism_loc.
- No-LiDAR options: a fixed dock pose, the last saved pose, a fiducial marker (ArUco/AprilTag) seen by the Orbbec camera.

What good looks like: from random starting poses anywhere in the mapped area (not only the 1 m / 20 degree offsets the bench uses today), the share of runs that end at the right pose, the error, the time to a pose, and how often it accepts a wrong pose. Wrong acceptance matters most. AMCL's reported covariance did not warn when it was lost in Corridor01, so the robot would drive on a wrong pose. Test it also against the wrong map and outside the mapped area. Then check that AMCL, started from the result, converges end to end.

- There is no random start-pose generator yet. `pose_eval.py init` only pushes the reference pose along its heading by `--dist` and turns it by `--dyaw`. Write a generator: sample free cells of the PGM and headings, and take the true pose at the query time from the reference.
- Define "accepted" and "wrong acceptance" before you measure. Accepted is the rule the method uses to say it is localized (residual, inlier ratio, covariance). Wrong acceptance is accepted with an error over 0.30 m or 8 degrees, the thresholds of `pose_eval.py`.
- Registration methods can run offline on scans read straight from the bag (`map_visibility.py` shows how). You need a ROS replay only for the end-to-end AMCL step. To replay from a time, use `ros2 bag play --start-offset <s>` (the flag exists in the image) and `PLAY_ARGS` of `run_bench_a.sh`. `pose_eval.py initpose --at-time` takes the header stamp in epoch seconds, not the offset into the bag.
- There is no fixed budget. Your human decides when to stop. A sensible order: a survey of options, the AMCL baseline, then the shortlist.

## Stack

```
Livox CustomMsg -> FAST-LIO2 (/Odometry, deskewed /cloud_registered_body)
  -> planar_odom (levelled odom -> base_footprint TF)
  -> pointcloud_to_laserscan (z band 0.10-1.80 m above the floor) -> Nav2 AMCL on a PGM map
```

The map is built offline: Voxel-SLAM (chosen SLAM) -> `ply_to_map.py` -> PGM + YAML. `make_map.sh` (PR #13) does the whole chain and also leaves `map_clean.ply`, the 3D cloud the PGM came from. Check yourself that it is in the map frame.

Frames, learned the hard way:
- FAST-LIO2's world frame is the IMU frame at start. It is not gravity-aligned. `pose_eval.py level` computes the rotation to the levelled `odom` frame the PGM uses.
- The map YAML carries a `# T_map_world` comment block. It moves the Voxel-SLAM world frame into the map frame.
- Ground truth in the bench is the Voxel-SLAM trajectory of the same bag (`alidarState.txt`: `t x y z qx qy qz qw`). It gives the true pose at any time in the bag, so you can start a test anywhere along it. Voxel-SLAM has its own error, never measured.
- Our bags were recorded with the Mid-360 facing up, so it sees mostly ceiling, at 0.53 m height. The final mount is elevated and looks down. A method that leans on the ceiling needs a recheck after the remount.

## Tools in this repo (it carries the branch of PR #12, `feat/nav-bench-a`)

`tools/nav_bench/`, read its README first. `run_bench_a.sh` replays a bag over its own map with FAST-LIO2 + AMCL and scores it. `pose_eval.py` has the frame maths, the metrics and the initial-pose helpers (`init`, `initpose`, `level`, `check`, `report`). `cost_sampler.py` samples CPU and RSS per process from `/proc`. Tests: `cd tools/nav_bench && for t in test_*.py; do python3 -B $t || break; done`.

`tools/slam_bench/` has `ply_to_map.py`, `map_visibility.py` (it reads `CustomMsg` bags without ROS, straight from the bytes) and `make_map.sh`. They are not in this branch. They live in PR #13's branch. In the NX clone: `git fetch origin feat/make-map && git worktree add ~/work/make-map origin/feat/make-map`. Its README explains the usage.

The `robot` image has numpy and no scipy. The `mapper` stage in PR #13 adds scipy, `rosbags` and GTSAM. Neither has Open3D, matplotlib or any registration library. A full `docker build --target robot` takes about 11 minutes on the NX. Do not change the `robot` stage lightly. A research image or overlay is fine.

## Data on the NX (checked 2026-10-03)

Bags are `livox_ros_driver2/CustomMsg` mcap bags (`/livox/lidar`, `/livox/imu`) under `/home/jetson/rosbags`. Maps and references are under `/home/jetson/nav_data` and `/home/jetson/map_out`. List them yourself. 

| Bag | On the NX | Map PGM + YAML | 3D cloud | Reference poses (`alidarState.txt`) |
|---|---|---|---|---|
| `custom/mapping_eco_-1_01_custom` (523 s, out and back, 177 m) | yes | `map_out/eco01_1x/map.{pgm,yaml}` | `map_out/eco01_1x/{map_clean,world}.ply` | `map_out/eco01_1x/vs/run/alidarState.txt` |
| `custom/mapping_eco_-1_03_custom` (271 s) | yes | `nav_data/eco_-1_03/final.{pgm,yaml}` | no | `nav_data/eco_-1_03/alidarState.txt` |
| `benchmarks/m3dgr_Corridor01_custom` (featureless corridor, other robot, sensor height 0.77 m) | yes | `nav_data/corridor01/final.{pgm,yaml}` | no | `nav_data/corridor01/alidarState.txt` |
| `custom/hospital_02_custom` (634 s) | yes | no | no | no |
| `custom/mapping_real_01_custom` | yes, but invalid: a 72 s LiDAR gap | | | |
| `benchmarks/m3dgr_Elevator01_custom` | yes, but multi-level, which `ply_to_map` does not support | | | |

`hospital_02` is unusable until a `make_map.sh` run gives it a map, a cloud and a reference. Use a map, a cloud and a reference that come from the same Voxel-SLAM run. `nav_data/eco_-1_01/map.pgm` differs from `map_out/eco01_1x/map.pgm`, so use the `map_out` set for eco01. The eco01 reference in both places is identical.

Two 3D clouds exist per run. `map_clean.ply` is the obstacle band only (0.10 to 1.80 m above the floor) in the map frame, with no floor and no ceiling. It was checked against the PGM: all points fall on occupied cells. `world.ply` is the whole Voxel-SLAM cloud, 5 cm voxels, with floor and ceiling, in the un-levelled Voxel-SLAM world frame. Apply `T_map_world` from the YAML to move it into the map frame. It has far outliers (z up to 34 m on eco01). The Mid-360 faces up, so the ceiling is most of what the sensor sees. A method that registers scans against `map_clean.ply` has no ceiling to match.

Generate what is missing with `make_map.sh` in the `socialtech:mapper-verify` image (it exists on the NX), one bag at a time on an idle board. It runs Voxel-SLAM at 1x, so it takes about as long as the bag. It writes the PGM, YAML, both PLYs and the reference in one output directory:

```
docker run --rm --network host -v /home/jetson/rosbags:/bags:ro -v ~/work/maps:/out -v ~/work/make-map/tools/slam_bench:/tools:ro \
  socialtech:mapper-verify /tools/make_map.sh /bags/custom/<bag_dir> /out/<name>
```

Other bags (`seat_01`, `zona_*`, Corridor02, TIERS) are not on the NX. The AGX, the Nano or the maintainer's laptop may have them. If you need one, ask the human.

## What we already measured

Everything in this section comes from the maintainer's private notes, which you cannot see. Treat each number as a hypothesis until a measurement of yours backs it. Numbers for the current localization stack, to compare against. Orin NX and AGX, `jetson_clocks` on, 1x replay:
- FAST-LIO2 + AMCL (tuned) + scan conversion: about 23-34 % of one core, about 300 MB RSS in total. With the default CPU governor the same stack used double.
- AMCL tuning: `alpha1-4` 0.01, 2000 fixed particles, `update_min_d/a` 0.05, 180 beams. It was tuned for offsets up to 2 m and 40 degrees. Large offsets are untested. With a lower `alpha`, AMCL converged late and erratically from large offsets (100-243 s).
- Convergence from a 1 m / 20 degree offset: 3-28 s. Position error median 3-5 cm, p95 7-11 cm, 0 losses on the own bags.
- Memory budget on the Nano without a desktop: about 6.4 GB left after the nav stack, for the rest of Nav2, the planned AI layer (person following, voice) and CUDA. A CUDA context costs about 90 MB of RAM per process. GPU memory is shared RAM and cannot go to swap.
- Hard cases: Corridor01 and Corridor02 (M3DGR) are long featureless corridors. Position along the axis is ambiguous there, the registration between two maps was off by about 3 m, and 3D ICP degenerates in them. FAST-LIO2 is not repeatable on degenerate bags: two identical runs differed by up to 14 m. Outdoor is out of scope.
- A hint, not a result: an earlier side experiment registered one 2D map against another (levelled maps, FFT correlation over 4 yaws, then 2D ICP, 9 maps). The notes record 5 pairs accepted at 1.8-3.5 cm RMS, 120 of 120 correct with full overlap, and failures with partial overlap: 4-5 of 16 with a 30 m window, 0 of 16 with 15 m. The setup is not documented in detail. It was map against map, not a live scan against a map. A robot at boot sees only part of the map, so partial overlap may matter for us too. Verify it.
- AMCL does not recover from being lost: `recovery_alpha_*` are 0, so it injects no random particles. With the pose initialised through `/initialpose` and an explicit covariance it behaves like an RViz click. With the `set_initial_pose` parameter the covariance is zero and every particle starts on one pose.
- Stage B (a bag replayed over another bag's map) worked 10 of 10 where the areas overlap. Corridor01 over Corridor02's partial map lost the pose when it left the map and found it again about 100 s later, when it came back.

## The standard: how we measure

Every decision on this project (the SLAM, the AMCL fix, FAST-LIO2 as odometry) came from measurements, not opinions. The standard binds the human as much as the agent. The agent drafts and runs. The human reads the raw numbers, not only the agent's summary, reruns a result they do not trust, and does not accept "it works" without a distribution behind it.

What it looked like, three times:
- **SLAM choice.** Five methods measured on 6 public sequences and 3 own bags. A criteria table written before the results. 3 to 5 runs for the finalists. Error against motion-capture ground truth with a time-offset sweep, and against an ArUco end pose. Determinism checked by trajectory md5. Seven invalid runs listed with their cause. The conditions that would reverse the decision written down.
- **AMCL fix.** Parameters changed one at a time. The code read (`amcl_node.cpp`, `pf.c`) to explain each effect. The 16 worst events replayed in isolation. A candidate validated on 2 bags and 3 start offsets, then its real-time cost sampled. Wrong hypotheses ("AMCL reacts late in turns") rejected with data and written down.
- **FAST-LIO2 as odometry.** iG-LIO was built and run. It diverged on every bag, in 4 configurations. We recorded "our build diverges, cause unknown, not evidence about the algorithm", not "bad algorithm". The 3D localizers were dismissed from research only, never built. The IMU-noise change that fixed Corridor01 (3 of 3 starts survive, against 0 of 3) was checked on 8 other bags and 21 runs with 0 losses. The mechanism was not proven, and we wrote that. FAST-LIO2's RAM was broken down with `smaps`, cut from about 155 MB to 52-73 MB, found a bug (indexing past `size()` after `clear()`), and the odometry stayed identical byte for byte on 4 bags.

The rules that follow:
1. **Distributions, never one number.** For every quantity, per run and across runs, report the mean or median, the p95 and the worst case. That covers position and yaw error, time to a pose, CPU and RAM. State N: runs, bags, start poses.
2. **Define a metric before you measure it**, with its threshold, and keep it fixed. Existing ones in `pose_eval.py` (thresholds provisional): converged = error under 0.30 m and 8 degrees held for 5 s. Loss = error over 0.5 m for at least 3 s. Correction at the robot per update, not the `map -> odom` step, which grows with the distance to the odom origin. Write down what you expect before a run.
3. **Know the noise floor first.** Repeat identical runs. If they differ, that spread is your resolution, and a difference inside it is not a result. Three runs only catch big effects. Separating close configurations would need about 10. AMCL is deterministic for identical inputs, so vary the start conditions, not the repeat count. Registration methods with random sampling need repeats and recorded seeds.
4. **Success rates need confidence intervals.** 0 failures in N runs is not zero: 0 of 4 is compatible with up to about 50 % at 95 % confidence. Report Wilson or Clopper-Pearson bounds, above all for wrong acceptance. Plan enough starts: 0 errors in 50 starts still leaves a 95 % upper bound of about 7 % (Wilson).
5. **One change at a time, with controls.** Use a negative control (the wrong map, a start outside the mapped area, the opposite sign). Check that your override reached the running config: a ROS node ignores parameters it does not know, silently.
6. **Same input for every method, on the same board.** Do not tune on the data you score on. Hold out bags or start poses, or leave one bag out. Say which results are best-case (a bag over its own map).
7. **Know your ground truth.** The reference is a Voxel-SLAM trajectory with its own error, never measured, and the map comes from the same run. Our AMCL position error already sits on that floor, so do not claim precision below it.
8. **Find the cause before you fix.** Evidence, then hypotheses, then reject them with data. Reproduce a failure in isolation. Read the upstream source and cite file and line. Docs, blogs and AI summaries were wrong more than once here.
9. **Label every claim** as measured, read in code, or unverified. Keep invalid runs in the log (lost messages, a shared DDS domain, a wrong time offset) with the reason. Rerun them and never put them in a table. Keep a list of your own mistakes and their corrections.
10. **Do not over-conclude.** A broken build of your own is not evidence about the algorithm. One run on one bag is a hint.
11. **Every number points to its data and its command.** Someone else must be able to reproduce it. Write findings as you go in `research/global-reloc/`.

Infrastructure, each learned from a bad run:
- Same input for every method: same random start poses, same bags, same map.
- Check that a run lost no messages. `ros2 bag play` publishes best-effort, so a loaded board drops messages and the filters diverge. Replay at 1x on an idle board.
- Pick your own `ROS_DOMAIN_ID`, one per run: the bench defaults to 79, GLIM uses 11, FAST-LIO2 and iG-LIO 77, Voxel-SLAM 78. Set `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` on shared networks. A second ROS process with the same topic names corrupts the first one.
- On the NX, check `jetson_clocks` is on (`systemctl is-active jetson-clocks`). The default governor doubles the CPU numbers.
- RSS overcounts shared libraries. Prefer PSS from `/proc/<pid>/smaps_rollup`. Use `tegrastats` for GPU and power. `cost_sampler.py` uses RSS only.
- A relocalizer that runs once at boot has a peak and a duration. One that stays alive has a steady cost. Report both, and measure on the Nano too if the candidate could fit there.
- Make plots (matplotlib in a venv, not in the image). Overlay the scan on the map at the estimated pose. Narcís judges results visually, in RViz2 or CloudCompare, and his verdict beats your metric. Commit the key plots as small PNGs in `research/global-reloc/figures/`. Keep raw data out of git and name its path on the NX in your report.

## How to work

- Explain your reasoning to the person you work with before you change any code. Be direct and critical. If a plan is weak, say so first.
- Explore the repo before asking a question the repo answers.
- Verify before you claim. Run the code, read the upstream source (cite file and line), read the official docs and issue trackers. Search the web broadly, with several abstract queries, not one narrow guess.
- Simple, correct code over code built to scale. Write a test first for behavior changes. Small tests like the existing `test_*.py` are enough.
- Findings go in `research/global-reloc/` on your branch: English, dense, tables and fragments, no filler.
- Report outcomes as they are. A negative result is a result.

## Repo rules

- `main` is protected. Work on a branch `feat/global-reloc` and open a PR with base `feat/nav-bench-a` (PR #12). The bench you need is not in `main` yet. Never push to another branch, never merge. Only the maintainer has write access to this repo. If your human has not been invited, fork it and open the PR from the fork.
- Code, comments, commits and docs are in English. This repo is public: no keys, IPs, hostnames or passwords in it.
- `tools/` is internal research tooling. Fix correctness bugs there and stop. Do not harden it for deployment. The Dockerfile, `host/` and anything that runs on the robot get the strict standard: a clean root-cause fix that holds on every Jetson, third-party behavior copied from the upstream source, no speculative additions.
- Our own configuration never goes into patches of third-party code. Read "Where changes go" in `README.md`.

## The NX board

A dedicated Orin NX (JetPack 7.2.1) is shared with nobody. If you break it, Narcís reflashes it. Still treat it like a real robot. You log in as the board's main account, `jetson`, with an SSH key your human installs with `ssh-copy-id`. Your human has the hostname and the password. You do not have the password.
- Work in the repo clone `~/socialtech-agv`, on your branch: `git fetch origin && git switch feat/global-reloc`. Put scratch files and outputs in `~/work/`. The bags also exist on the AGX, the Nano and the maintainer's laptop, so a lost file or a reflash of the NX costs little. Still tell your human before you delete something you did not create.
- You have no root. When a step needs `sudo`, give the human the exact command and a one-line reason. They run it in a separate terminal, then paste the result. Claude Code's `!` prefix cannot run `sudo`.
- Work in containers and in your home. Do not touch host setup, networking, Tailscale, power mode, `jetson_clocks` or sysctl. If the board needs a host change, ask the human.
- Docker access is root-equivalent. Do not run `--privileged`, mount `/` or `/etc`, or mount more than you need.
- Docker writes files as root. Use a new output directory for each run.
- Kill processes by PID, not `pkill -f` (it kills your own shell). Start long remote jobs with `ssh -f -n <host> 'setsid nohup <cmd> &'`. Never `kill -KILL` a `ros2 launch`: it leaves orphan nodes.
- Data: see "Data on the NX". Per bag you need the map PGM and YAML, `alidarState.txt` of the Voxel-SLAM run that made the map, and the sensor height (`pose_eval.py check` prints it). The clone at `~/socialtech-agv` sits on `main`, clean.
- Free disk is about 130 GB of 233 GB. The images are 6-7 GB each. Delete your own outputs when done.

## Second task, if there is time: review the maintainer's PRs

Review open PRs #4 to #13 of this repo in numeric order. Leave comments for the maintainer to read. Do not change code, push or merge.
- One pass per PR. Do not loop fix and review.
- Post one `gh pr review <n> --comment` per PR, starting with `[AI-review]`. List findings by severity (blocking, minor, question) with `file:line` and the concrete input that breaks. Add inline comments with `gh api` where it helps.
- Stacked PRs (#5-#7 and #9-#11 on top of each other, #13 on #9): review only the diff against the PR's base.
- Check the PR description against the code. Where code copies a third-party behavior, read the upstream source.
- Apply the repo rules above. In `tools/`, report correctness bugs only. If nothing is critical, say so.
- Context per PR is in its description. The base of each PR: #4 `main`, #5 `feat/glim-ply-export`, #6 `feat/glim-eval`, #7 `feat/slam-bench`, #8 `main`, #9, #10 and #11 `feat/ply-to-map`, #12 `main`, #13 `feat/map-visibility`.
