# 18 — New independent data and physical validation: plan and checklists

Date: 2026-10-10. Branch `feat/global-reloc`. **Plan only. No new data were recorded, the robot was not touched, nothing here is a result.**
It applies protocol `02-protocolo-evaluacion.md` unchanged (limits 0.30 m / 8 deg, 60 s from first permitted input, >=30 independent blocks >=5 m apart per cell for conclusions, >=73 independent zero-failure starts for the 5 % Wilson bar).

## 1. Why the existing data are not enough (V, counted in note 16)

- Stationary episodes long enough for a real S2 (>= 6.2 s still): 8 in two datasets, about 6 independent 5 m blocks. Target 30.
- Every positive so far used a map built from the same bag, or eco03 on the eco01 map (same building, same day, same mount).
- All of it was looked at while developing the gate and the node: it is tuning data now. None of it may be reported as final evaluation.
- Sensor facing up at 0.53 m; the final mount looks down.

## 2. Sessions to record

All with the **final mount**, after section C of note 17. One operator pushes or drives the chair under the usual manual control; the relocalizer is not running and commands nothing.

| Session | Purpose | Content | Use |
|---|---|---|---|
| M1 | prior map | one closed loop of the whole area at walking pace, start and end at the dock, 10 s still at both ends | builds PGM + `map.xyz` (`make_map.sh`). Never a query |
| M2 | second prior map, another day | same route | map-age control, reference cross-check |
| Q-dev | development queries | 12 stops, see section 3 | tuning allowed; never reported as evaluation |
| Q-eval-A, Q-eval-B | evaluation queries, two different days | >= 40 stops each, see section 3 | **frozen parameters, opened once** |
| N-out | negative: outside the map | >= 15 stops in rooms/corridors not covered by M1 | no-match control |
| N-changed | hard positive | 10 stops of Q-eval repeated with doors closed/open, chairs and beds moved, 2-3 people standing within 2 m | changed-scene control (positive: a correct pose exists) |

Query sessions are recorded on a different day than the map they are scored against (M1 for Q-eval-A, and again against M2).

## 3. A stop (one block)

Each stop yields one S2 trial and one S3 trial from the same boot-like start:
1. Arrive, brake, **stand still 12 s** (IMU gate 2 s + LIO + 5 s exposure + margin; lets the 1/3/5 s S2 variants share one stop).
2. Say the stop ID aloud / press the marker key (section 5).
3. S3 part: the operator moves the chair by hand, slowly (<= 0.35 m/s), up to 2.5 m from the stop along any free direction, with a turn if possible, for at most 25 s, then stops. Stay inside 3 m: the operator watches a 2.5 m tape on the floor. The software limit (2.8 m radius, 6 m path, 30 s) is checked afterwards from the method's own odometry.
4. Drive on >= 5 m to the next stop.

Stop placement, fixed on the floor plan **before** recording:
- >= 30 stops (target 40) pairwise >= 5 m apart per session; every room and corridor segment gets at least one.
- Heading: assign in advance from {0, 90, 180, 270 deg} relative to the corridor axis, balanced; plus 8 extra stops that repeat an earlier position with the opposite heading (reported as the same block, they add heading coverage, not independence).
- Repetitive geometry on purpose: mark on the plan every pair of places that look alike (parallel corridors, identical rooms, both ends of a long corridor, door niches at a regular pitch). At least 8 stops in such places, labelled `alias` beforehand with the ID of their look-alike. A long featureless corridor gets stops at 1/4, 1/2 and 3/4 of its length.
- Open areas (lobby, > 10 m to the nearest wall) and cluttered areas (beds, trolleys) labelled as such.

Independence rules:
- One block = one 5 m cell. Several windows, headings, exposures or seeds at one stop are **one** block.
- The same stop visited in Q-eval-A and Q-eval-B is one block for the 30-block rule, two sessions for day-to-day variation.
- A stop within 5 m of a Q-dev stop is development, whatever session it was recorded in.

## 4. Reference pose

The reference today is the Voxel-SLAM trajectory of the same bag, error never measured. For evaluation it has to be defensible on its own:
1. Voxel-SLAM on the query bag (`make_map.sh`), linked to the map frame with `map_link.py`: as now. Report the link residual per session.
2. **Independent check at surveyed stops:** >= 10 stops per session marked on the floor with tape crosses whose map coordinates are measured once with a tape/laser distance meter from two walls visible in the PGM (resolution 2 cm). At those stops the reference must agree with the survey within 0.10 m; otherwise the session's reference is not trusted and the error budget is reported, not hidden. Heading at surveyed stops: chair aligned to a taped line parallel to a wall (2 deg).
3. Loop closure residual at the dock (start vs end pose of each session).
4. If an ArUco/AprilTag board is available at the dock, its pose is a third check.
A stop where the reference has a gap is `unscorable`, counted and listed, never dropped silently.

## 5. Recording procedure

```
tools/global_reloc_3d/record_session.sh <session_id> <output_dir>     # NOT yet run on hardware
```
It records `/livox/lidar` and `/livox/imu` (mcap), writes `session.json` (fields below, it refuses to start if one is missing), and runs `bag_stream_integrity.py` on the result.
`session.json`: session ID, date, operator, map session it will be scored against, mount: orientation (up/down), `sensor_height_m`, `base_from_body_xyyaw`, tilt, measurement date and who (note 17 C2-C6), photo file names, firmware/driver commit, image digest, load on the chair, floor plan revision, list of planned stops (ID, x, y, heading, labels), deviations noted during the run.
Stop markers: a keypress in the script appends `(stop ID, wall time)` to `stops.csv`. The start time of each trial is then the first IMU-certified stationary instant after the marker (`imu_ready.py`, truth-free), not the keypress.
Before leaving: integrity report shows < 1 % missing IMU samples (today's bags: 6-8 %; fix that first, note 16), no LiDAR gap > 0.3 s, every planned stop has a marker.

## 6. Development / evaluation separation and freeze

1. Freeze **before** opening Q-eval: commit hash, image digest, `verify.py` gate, search parameters, S2 exposure (3 s main; 1 s and 5 s declared ablations), mount parameters, the stop list and labels, the evaluator. Write the hash in `session.json` of the evaluation run.
2. Q-eval bags are copied to a directory the method containers never mount until the freeze commit exists. The evaluator and the reference stay in a separate container (as now).
3. One run of the full set. A code or parameter change after that creates a new version and consumes the set: the next evaluation needs new stops.
4. The band gate of note 14: computed in diagnostic mode on Q-dev only. If it is to be proposed, its threshold is chosen on Q-dev and tested once on Q-eval, as a separate pre-declared configuration next to the frozen gate.
5. Report per cell (dataset x scenario S2/S3 x map session): counts, Wilson 95 % bounds, blocks and starts separately, errors and latencies with mean/p50/p90/p95/max, every wrong accept with its overlay plot.

Cells and minimum N (independent blocks): S2-3s positive 30 (target 73+), S3 positive 30 (target 73+), N-out 15 (descriptive only below 30), wrong-map negative: all Q-eval stops against a map of another building, alias positives >= 8 (descriptive).

## 7. Physical validation without autonomous motion (checklist, nothing executed)

Levels. Only the first is reached today.

| Level | Meaning | Status |
|---|---|---|
| `VALIDATED_IN_REPLAY` | node + AMCL on recorded bags at 1x on the NX | reached for development windows (notes 15, 16) |
| `VALIDATED_ON_HARDWARE_STATIONARY` | live sensors, robot still, motors off | **not done** |
| `VALIDATED_WITH_SUPERVISED_MOTION` | S3 with a person moving the chair | **not done** |
| `FORMALLY_BENCHMARKED` | section 6 executed on Q-eval | **not done** |

Preconditions: motors disabled or drive disconnected (the relocalizer publishes no velocity; this is belt and braces), Nav2 controllers not launched, a person at the chair, note 17 C done.

H1 stationary (each: expected outcome written first; record `metrics.json`, `launch.log`, a screen capture of RViz):

| # | Test | Expected |
|---|---|---|
| H1.1 | TF tree after boot (`ros2 run tf2_tools view_frames`) | `map -> odom -> base_footprint`, `odom -> camera_init -> body`; no duplicate publisher of `odom -> base_footprint` |
| H1.2 | Stream health live 5 min | IMU ~200 Hz, LiDAR 10 Hz, missing < 1 %; compare with the 6-8 % of the old bags |
| H1.3 | Cold start at the dock, robot still, mode `s2s3`, no supervisor grant | `LOCALIZED` from S2, or `S3_SUPERVISION_REQUIRED` then `TIMEOUT s3_not_authorized_in_budget`. No pose without `ACCEPTED` |
| H1.4 | Pose check | RViz: scan on the walls; tape-measured position of the stop vs `/amcl_pose` within 0.30 m / 8 deg |
| H1.5 | 10 different stops, 3 boots each | distribution of outcome and time; boots at one stop are repeats, not blocks |
| H1.6 | Wrong map loaded | no pose published, `REJECTED` or `TIMEOUT` |
| H1.7 | LiDAR cable unplugged at start / IMU topic absent | `TIMEOUT sensors_unavailable` at 60 s |
| H1.8 | LiDAR unplugged during accumulation | `ERROR lio_stalled` or `fastlio_exited`, no pose |
| H1.9 | Sensor covered (cardboard box over it) | `REJECTED`/`TIMEOUT`, no pose |
| H1.10 | Chair shaken by hand during the IMU gate | gate waits; `TIMEOUT imu_not_stationary` if it never settles |
| H1.11 | AMCL not started | `HANDOFF_FAILED amcl_not_available`, pose not published |
| H1.12 | Node killed (SIGKILL) and restarted | no orphan `fastlio_mapping`; new attempt from clean state |
| H1.13 | Clocks: power button -> login -> ROS stack up -> first sensor message -> `ACCEPTED` -> `LOCALIZED`, stopwatch plus logs, 5 cold boots | clock A (cold boot), reported next to the protocol clock; the protocol 60 s starts at the first sensor message |
| H1.14 | Localization lost after handoff: sensor covered for 20 s while AMCL runs | recorded behaviour of AMCL and of the node (today the node does not supervise AMCL after `LOCALIZED`: known gap) |

H2 supervised motion (a person moves the chair; software only asks):

| # | Test | Expected |
|---|---|---|
| H2.1 | Stop where S2 is insufficient; supervisor grants on `/global_reloc/s3_authorize`, moves 2 m | `S3_ACCUMULATING` -> `S3_SEARCHING` -> outcome; time of each step; whether the human reaction fits in the 60 s (note 16 expects it does not always) |
| H2.2 | Supervisor moves beyond 2.8 m | acquisition stops, state message shows `supervisor_stop_required: true`; the software stops nothing physically |
| H2.3 | Grant sent before it was requested | ignored |
| H2.4 | Odometry check: 5 m straight and 360 deg in place | notes 17 D3, D4 |
| H2.5 | After `LOCALIZED`, chair pushed 10 m by hand | AMCL keeps the pose within limits against taped stops |

Not covered by this plan, on purpose: autonomous driving, Nav2 goals, any command to the motors.
