# Point 7 — first real 3D global relocalization

Date: 2026-10-09. Status: **FIRST_3D_GLOBAL_RELOCALIZATION_SUCCESS**, exploratory only. Labels: **V** measured or read directly in pinned source; **I** inference/technical judgment; **NV-P** unverified/pending. This is one location on its own same-run map, not a reliability estimate or a deployable localization service.

## Frozen contract and architecture

- **V, source:** `FAST_LIO/src/laserMapping.cpp:546-561` publishes deskewed `/cloud_registered_body` in `body` at `lidar_end_time`; `/Odometry` gives the local body pose. `lio_submap.py` joins by header stamp (exact or valid <=0.20 s interpolation), transforms every body cloud into the first cloud's IMU/body gauge, splits the first 2/last 1 s of S2 3 s, and voxelizes at 0.20 m. No global pose is supplied. The fixed Mid-360 LiDAR-to-IMU translation in the installed `mid360.yaml` is `[-0.011,-0.02329,0.04412]` m, rotation identity. The evaluator scores the same IMU-origin convention as the Voxel-SLAM trajectory.
- **V, map:** `world.ply` is the global 3D Voxel-SLAM cloud in unlevelled `world`; YAML `T_map_world` transforms it to levelled `map`. `prepare_map.py` keeps map-frame `-0.5 <= z <= 5.0` m and voxels at 0.20 m. eco01: 910,024 raw points, 899,834 after crop, 55,071 voxel centroids; hospital: 1,012,527 raw points and 66,509 voxel centroids in the first preparation. `map_clean.ply` has only the 0.10–1.80 m obstacle band (358,479 eco01 / 380,833 hospital raw points); it excludes much of the upward-looking Mid-360 ceiling and was not the registration target. eco01 world has outliers up to z=34.75 m after `T_map_world`; hospital world z maximum is 4.01 m. These are measured artifact properties, not semantic ceiling labels.
- **V, source/API:** KISS-Matcher at `8b58b118e00c7a5a22448741f8d956d638a44ae2`, `KISSMatcher.hpp:118-178` / `KISSMatcher.cpp:122-148`, calls `estimate(source,target)` without any initial transform. The caller searches the whole prior plus every fixed 15 m tile on a 10 m grid, independent of query pose; it stores every trial and up to K=5 spatially distinct hypotheses. Its 0.30 m feature voxel is distinct from the 0.20 m input voxel. This is descriptor/robust-solver global search; no ICP truth seed is present. small_gicp at `fa0cfc983417c044b6036f71dfa448038bb9c820`, `registration_helper.hpp:66-92`, receives each KISS `T_target_source` as `init_T`, runs GICP with 1.0 m max correspondence distance and two threads. `register.cpp` logs matches, KISS rotation/translation inliers, GICP convergence/iterations/inliers/error and both transforms. Both libraries built and ran on aarch64/Jazzy in `socialtech:robot-nav`/`socialtech:mapper-verify`.
- **V, policy:** fit clouds never enter the checking geometry. The ranking is support fraction at <=0.15 m against the complete map, then matched RMSE, then KISS inliers; up to five distinct poses are retained. Acceptance requires valid KISS, converged GICP, >=0.70 checking support, <=0.10 m matched RMSE, >=0.10 absolute support lead over a searched distinct runner-up, an independently globally registered later batch agreeing within 0.20 m/5°, median batch separation >=0.5 s, total first-to-last span >=1 s, and a 60 s search deadline. No runner-up means abstention. These are the delegated point-4 initial gates, not thresholds fitted to the outcomes. **I:** the current nearest-neighbor check approximates “visible prior geometry”; ray/occlusion checking remains pending.
- **V, isolation:** `capture_fastlio.sh` receives only LiDAR+IMU bag and emits `/Odometry` plus `/cloud_registered_body`. In `isolated-v16`, map preparation mounted `world.ply` and `map.yaml` as individual read-only files; the joiner mounted only the captured LIO bag and prepared `map.xyz`. The final `isolated-v17` matcher mounted that `input`, truth-free code, `libsmall_gicp.so` and an empty output root. Its `search.json` checksum was printed **before** a separate evaluator container mounted `alidarState.txt`. The reference was neither a seed, tile selector, leveling source nor candidate ranker. The raw stationary prototype preceded the LIO run but is not the decisive FAST-LIO2 result.

The transformed output is `T_map_localfirstimu`. For a later represented time `t`, `T_map_imu_est(t) = T_map_localfirstimu * T_localfirstimu_imu(t)`. The evaluator alone computes `T_map_imu_ref(t) = T_map_world * T_world_imu_ref(t)` and compares XY/yaw at the same header stamp. No map-to-odom-to-AMCL publication was attempted.

## Results: DEVELOPMENT / EXPLORATORY, not statistically interpretable

Each result points to `search.json`, `evaluation.json`, all tile `result.json` files and `manifest.json` under `/home/jetson/work/global-reloc-3d-20261009/`. No original bag, map or reference was changed. eco01 own-map query is the predeclared `1780999653.438871296` header-time window; hospital is `1781693840.039264256`. The same-run Voxel-SLAM reference has unmeasured error and shares mapping observations with the query. The strict formal inventory in `04-inventario-ventanas.md` has **zero** eligible S2 windows under its IMU gap rule, so these are explicitly development diagnostics.

| Dataset / run suffix | Window | XY m / yaw ° after GICP | Method decision | Class after evaluation | Search s |
|---|---:|---:|---|---|---:|
| eco01, `isolated-v17/result` (final FAST-LIO2) | S2 3 s | **0.02051 / 0.08541** | accepted | correct_accept | 3.528 |
| eco01, `isolated-v16/result` (prior isolated repeat) | S2 3 s, same input | 0.02117 / 0.08552 | accepted | correct_accept | 3.374 |
| eco01, `point7-v1/eco01-lio-s2-5s-w01-search-v12` | S2 5 s, same place | 0.01802 / 0.03047 | accepted | correct_accept | 3.367 |
| eco01, `point7-v1/eco01-lio-s2-1s-w01-search-v10` | S2 1 s, same place | 0.01367 / 0.00227 | ambiguous_no_temporal_confirmation | diagnostic_correct_not_accepted | 2.880 |
| hospital, `point7-v1/hospital-lio-s2-w01-search-v6` | S2 3 s | 11.39794 / 0.42304 | reject_geometry | diagnostic_wrong_not_accepted | 4.321 |
| eco01 over hospital map, `point7-v1/eco01-lio-hospital-map-control-search-v7` | S2 3 s | NA (unrelated map frames) | reject_geometry | cross-map rejection | 3.721 |

**V, final eco01 raw evidence:** 30/30 selected LIO clouds joined to odometry; 5,982 fit and 5,264 check voxels. Fixed search examined 15 targets (full + 14 tiles), 8 gave KISS-valid and GICP-converged poses. The selected tile `tile_0_1` had 68 initial matches, 68 rotation inliers, 63 translation inliers, 1,521 GICP inliers and 2 iterations. On checking geometry, 5,262/5,264 points were <=0.15 m from the map (0.99962 support), matched RMSE 0.09196 m, p95 nearest distance 0.12698 m; the distinct runner-up support lead was 0.76463. The independently registered later batch agreed at 0.01114 m / 0.05605°; batch-median separation was 1.500 s. KISS alone scored 0.09519 m / 1.83147°; GICP refined to 0.02051 m / 0.08541°. The frozen `search.json` SHA-256 is `9cb8203dbab85b6cf334b7862dd814bea0faa0b89097b569a14e97bfd6959197`. [Overlay](figures/eco01-lio-s2-w01-overlay.png) plots an earlier accepted repeat of the same input; visual approval in RViz/CloudCompare is still **NV-P**.

**V, controls and failure mechanism:** hospital's selected LIO hypothesis had 7 KISS matches, 5 translation inliers, 0.18845 checking support, 0.01539 lead, and later-batch disagreement; GICP convergence alone would have been misleading. Its wrong pose was rejected before reference inspection. At the evaluator-only reference pose, hospital checking support is 0.92415 and RMSE 0.08665 m, so gross absence of map geometry is not the explanation. The true-location tile was searched (`tile_6_1`); KISS produced 7 matches, 3 translation inliers and no converged GICP. **I:** correspondence/descriptor ambiguity is the immediate failure; the deeper cause (viewpoint density, symmetric ceiling, FPFH scale, map cleaning or scene structure) is not proven. A planned KISS feature-scale sweep on this same development input gave 0/18 valid converged candidates at 0.20 m (`hospital-lio-s2-w01-kiss02-v14`) and rejection at 0.40 m (`...kiss04-v15`); neither rescues this window. At the eco query over hospital map, the best apparent hypothesis had 0.31383 support and 0.02793 lead and was rejected. Map disjointness was not independently certified, so call this a cross-map diagnostic rather than a formal negative-control rate.

**V, repeats and resources:** four identical-input eco01 3 s searches (`point7-v1` v3/v8/v9 and `isolated-v16`) all accepted the same tile. Maximum pairwise final-pose spread was 0.00115 m / 0.00101°. Their search times were mean 3.292, p95 3.373, worst 3.374 s (N=4 algorithm repeats, **one** physical start); the final timeout-corrected `isolated-v17` repeat also accepted at 3.528 s. The final search used 9.126 child CPU seconds, observed peak child RSS 76,284 KiB and orchestrator RSS 55,908 KiB; `jetson_clocks` was active. These are CPU/RSS process readings, not PSS or total frontend cost; GPU, power, Nano cost, FAST-LIO2 peak and complete boot-to-pose latency are **NV-P**. The captured 3 s exposure and 3 s pre-exposure replay occurred at 1×, but the capture, offline join, map load and search were separate processes, so their durations are not a measured integrated boot latency. No percentage reliability or confidence interval is inferred from correlated variants/repeats.

**V, baseline context:** the prior global-AMCL exploratory v36 used the same eco01 S2 point-time window and rejected after 29/29 scan updates; its reported last mode was 16.72 m / 10.89° away (`09-amcl-sweep-exploratorio.md`). It replayed at 0.25× with a 2D scan projection, whereas this 3D attempt acquired at 1×; this is a diagnostic comparison, not a matched 1× latency/resource benchmark. `10-amcl-cierre-exploratorio.md` remains `AMCL_WEAK_BASELINE`, not a universal AMCL failure claim.

## Reproduction of the decisive isolated run

Run on the NX with the existing `socialtech:robot-nav` and `socialtech:mapper-verify` images; create **new** versioned output paths and choose an unused ROS domain when repeating (181 is the historical run). `B=/home/jetson/work/global-reloc-3d-20261009`, `C=$B/isolated-v16` (map/input), `D=$B/isolated-v17` (final matcher). The pinned source builds and their build commands are in `11-punto7-arranque.md`. `register.cpp` was compiled against their existing aarch64 static/shared artifacts (compiler/link command is preserved in this point-7 session transcript); `test_register_synthetic.py` exercises the resulting binary. The final matcher code hash is `fe9feb9de367e55585980c0ced4ba7d4c899b0341b84314cf3eb9dbc0f296c56`; binary hash `4d51becba30ef2cd05444426db53d5c914f68252bf4d78ed5d7bc56712423f43`.

```bash
B=/home/jetson/work/global-reloc-3d-20261009
C=$B/isolated-v16
D=$B/isolated-v17
# Capture: no map or reference mount. Offset is seconds from bag start, not an epoch stamp.
docker run --rm --network host -e ROS_DOMAIN_ID=181 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -v /home/jetson/rosbags/custom/mapping_eco_-1_01_custom:/bag:ro -v "$B/point7-v1":/tool:ro -v "$B":/out \
  socialtech:robot-nav bash /tool/capture_fastlio.sh /bag 100.744259 7.3 /out/point7-v1/eco01-lio-w01-capture-v2
# Prior map, narrow mounts: world.ply and YAML only.
docker run --rm --network none -v /home/jetson/map_out/eco01_1x/world.ply:/data/world.ply:ro \
  -v /home/jetson/map_out/eco01_1x/map.yaml:/data/map.yaml:ro -v "$C/code":/tool:ro -v "$C":/out \
  socialtech:mapper-verify python3 /tool/prepare_map.py --world /data/world.ply --yaml /data/map.yaml --out /out/map --voxel 0.20
# Source submap: LIO capture and prepared prior only.
docker run --rm --network none -v "$B/point7-v1/eco01-lio-w01-capture-v2/lio_bag":/capture:ro \
  -v "$C/map/map.xyz":/map/map.xyz:ro -v "$C/code":/tool:ro -v "$C":/out \
  socialtech:mapper-verify python3 /tool/lio_submap.py --bag /capture --map-xyz /map/map.xyz \
  --start 1780999653.438871296 --duration 3 --out /out/input
# Method: no trajectory, evaluator code, or reference directory mounted.
docker run --rm --network none -v "$C/input":/input:ro -v "$D/code":/tool:ro \
  -v "$B/small_gicp-build/libsmall_gicp.so":/opt/libsmall_gicp.so:ro -v "$D":/out \
  socialtech:mapper-verify bash -lc 'export LD_LIBRARY_PATH=/opt:$LD_LIBRARY_PATH; python3 /tool/search_tiles.py --run /input --out /out/result --binary /tool/register'
sha256sum "$D/result/search.json"  # freeze before evaluator
# Evaluator is a separate process with individual reference files mounted only now.
docker run --rm --network none -v "$C/input":/input:ro -v "$D/result":/result -v "$C/evaluator":/evaluator:ro \
  -v /home/jetson/map_out/eco01_1x/vs/run/alidarState.txt:/reference/alidarState.txt:ro \
  -v /home/jetson/map_out/eco01_1x/map.yaml:/reference/map.yaml:ro socialtech:mapper-verify \
  python3 /evaluator/evaluate.py --run /input --search /result --reference /reference/alidarState.txt \
  --map-yaml /reference/map.yaml --out /result/evaluation.json
```

The caller binary was compiled in `socialtech:robot-nav` with `g++ -std=c++17 -O2 -fopenmp`, the KISS core and ROBIN/PMC static archives under `$B/kiss-build`, the `$B/small_gicp-build` shared library, include roots from `$B/kiss-matcher/cpp/kiss_matcher/core`, `$B/kiss-build/_deps/robin-{src,build}/include`, `$B/kiss-build/xenium-src`, `$B/small_gicp/include` and `/usr/include/eigen3`, and links `-lsmall_gicp -lflann_cpp -llz4 -ltbb`. The complete compiler invocation and exit 0 are preserved in the point-7 command transcript; the source and frozen binary hashes above identify the executed artifact. The images' Linux Python wheels were not used.

**V, tests:** laptop `cd tools/global_reloc_3d && for t in test_*.py; do python3 -B "$t" || break; done` passes all pure-Python tests (synthetic binary test skips without `REGISTER_BINARY`). On NX/aarch64, `REGISTER_BINARY=/src/point7-v1/register python3 -B /src/point7-v1/test_register_synthetic.py` passed with a known 33°/4,-3,0.5 m synthetic transform. Tests cover composition/inversion, quaternions and reference interpolation, frames, stamp guards, voxelization, registration, acceptance, false-position rejection and the truth-free method signature. The synthetic result does not count as a real localization.

## Limitations and next action

- **NV-P:** no different eco01 place, held-out physical start, independent mapping session, S3 moving submap, outside-map negative, strict formal sample or end-to-end AMCL initial-pose handoff was tested. The 3/5 s eco01 acceptances and four repeats share one location and one map. Physical chassis-center extrinsic and the final downward-facing mount remain uncalibrated; the metric is the IMU-origin footprint proxy. Same-run reference/map error floor is unknown; the 2 cm estimate does not prove physical 2 cm accuracy.
- **I:** hospital's strong oracle overlap but failed KISS features makes viewpoint-changing S3 accumulation or a second global candidate (e.g. a different descriptor/search engine) the next useful experiment. Keep the same frozen acceptance rule and test independent locations plus genuine wrong-map/outside-map controls before considering autonomous boot. Then implement an explicit accepted `map -> odom`/AMCL handoff and measure full 1× boot latency/PSS/GPU on NX and Nano. No physical robot motion or deployment change was made.
