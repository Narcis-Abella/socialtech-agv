# socialtech_bringup

What starts the robot: launch files, the defaults they use and the scripts that run the stack in the container. Installed in `/opt/socialtech` (`share/socialtech_bringup`, `lib/socialtech_bringup`).

| Path | What |
|---|---|
| `launch/nav_live.launch.py` | Localization: Livox driver, FAST-LIO2, `planar_odom`, `/scan`, `map_server`, `gn_loc`, the level corrector, the local-frame aligner, the supervisor (`SUPERVISOR`). `driver:=false sim_time:=true` is the simulation |
| `launch/nav_robot.launch.py` | The above + Nav2 (`navigation_launch.py`) + the Tracer base (`base:=true`) |
| `config/live_overrides.yaml` | gn_loc and map_server parameters of the robot; the default of the `overrides` argument |
| `config/nav2_robot_overrides.yaml` | What differs from `socialtech_nav`'s `nav2_params.yaml` on the real robot; merged over it by `merge_params.py` |
| `config/scan.yaml` | The `/scan` cut (0.10 - 1.80 m above the floor): the obstacle band of the map, in sync with `grid2d.rasterize` and the global costmap (`ops/test_constants_agree.py` checks it) |
| `scripts/run_live` | The container half of `ops/nav_live.sh`: the DDS URI, the map's level and sensor height, the FAST-LIO2 parameters, the recorder (DEV), `ros2 launch`. Started by absolute path, never `ros2 run` (the recorder needs the SIGINT) |
| `scripts/fastlio_config.sh` | `fastlio_yaml`: the robot's edits of FAST-LIO2's `mid360.yaml`, shared by `run_live` and the simulation |
| `scripts/flags.sh`, `fastlio_check.sh`, `livox_config.sh`, `merge_params.py`, `base_state.py` | Helpers of `run_live` that `ops/` also uses on the host (flags, the base's CAN state, the Livox driver config) |
| `scripts/map_frame.py` | `level`: `qx qy qz qw` of the rotation from FAST-LIO2's world frame to the floor-levelled, wall-aligned one (`run_live` calls it), and the helpers `live_paths.py` and the tests import |
| `scripts/local_frame_aligner.py`, `nav_start_when_localized.py`, `live_paths.py` | Nodes and helpers the launch files start |
| `test/` | Tests of all of the above; `planar_odom.py` is the Python reference of the C++ node (`test_planar_odom_node.py` compares them inside the image) |

## Frames
FAST-LIO2's world frame is the IMU frame at start, **not** gravity-aligned. `odom` is that frame rotated by `R_L` (`map_frame.py level`) so the PGM's floor is z = 0 and the axes follow the walls. The map YAML must carry the `# T_map_world` comment block written by `make_map.sh`.

## Tests
`pytest src/bringup/socialtech_bringup/test` (the ones that need ROS are left out of the collection without it; each is also a script: `python3 test/test_x.py`). `test_flags.sh` and `test_fastlio_check.sh` are plain bash. `scripts/test.sh` at the repository root runs them all with the rest.
