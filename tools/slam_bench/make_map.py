"""The whole post-SLAM pipeline of make_map.sh in one process: the world cloud of a Voxel-SLAM run (or a map.ply and its poses) -> level the floor once -> remove the ghosts
(beams that go through them) -> map_clean.ply, the whole height -> the 2D map from the band 0.10-ceil of that cloud (+ the unknown cells the beams crossed) -> report.json.
  make_map.py (--run vs/run | --cloud map.ply --poses alidarState.txt) --bag BAG --out OUT [--res 0.025] [--voxel RES] [--ceil 1.80] [--topic /livox/lidar] [--node-log OUT/node.log]
Outputs in OUT: map.ply (the raw SLAM cloud), map_clean.ply (ghost-free, whole height, map frame, floor z = 0), map.pgm, map.yaml, report.json."""
import argparse
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

import cloud_io
import floor_level
import ghosts
import grid2d
import map_report

LOST_RATIO = 0.98       # poses / scans in the bag under which Voxel-SLAM is said to have lost data (smoke bag: 1x 0.996, 3.5x 0.989, 4x 0.968)
RULE = {"ghost_vox_m": ghosts.RATIO_VOX, "ghost_margin_m": ghosts.RATIO_MARGIN, "ghost_min_obs": ghosts.MIN_OBS, "ghost_min_ratio": ghosts.MIN_RATIO, "halo": list(grid2d.HALO),
        "island_cells_5cm": grid2d.ISLAND_CELLS_5CM, "close_m": grid2d.CLOSE_M, "fill_heights_m": list(grid2d.HEIGHTS)}


def build(cloud, poses, bag, out_dir, *, topic="/livox/lidar", res=0.025, ceil=1.80, ghost_band=(-np.inf, np.inf), traj_check=True, count=None):
    """Write the four cloud and map files into out_dir and return what the report needs. cloud: (n, 3) SLAM cloud (float32 values); poses: (m, 8) t x y z qx qy qz qw of the IMU at the
    end of each scan; count(C, margin) -> (hit, passed): the beam counter (default: the C++ engine over `bag`). ghost_band: heights (m above the floor) the ghost test judges.
    ValueError (nothing written) when the floor is not found or does not fit the trajectory."""
    Q, info = floor_level.level_to_floor(cloud, traj=(poses[:, 1:4], poses[:, 4:8]) if traj_check else None)
    if count is None:
        scans = ghosts.BagScans(bag, poses, topic)
        count = lambda C, margin: ghosts.count_beams(C, scans, margin)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    ghost = ghosts.ghost_points(cloud, Q, count, ghost_band)
    clean = Q[~ghost]
    cloud_io.write_ply(out / "map.ply", cloud)
    cloud_io.write_ply(out / "map_clean.ply", clean)
    grid, origin = grid2d.rasterize(clean, res, (floor_level.OBST_LO, ceil))
    grid = grid2d.fill_unknown(grid, res, origin, info["T"], poses[:, 1:4], count)
    grid2d.write_map(out / "map", grid, res, origin, info["T"])
    vote = info["traj"]["position"] if isinstance(info["traj"], dict) else None
    return {"floor_tilt_deg": round(info["tilt_deg"], 2), "sensor_height_m": round(float(vote["height"]), 3) if isinstance(vote, dict) else None,
            "ghost_points": int(ghost.sum()), "points": len(cloud), "ghost_band": [None if np.isinf(v) else v for v in ghost_band]}


def check_cloud_has_the_trajectory(cloud, poses, margin=1.0):
    """ValueError when the path of `poses` leaves the bounding box of `cloud` (+ margin m): a map.ply of another run next to these alidarState.txt."""
    lo, hi = cloud.min(0) - margin, cloud.max(0) + margin
    p = poses[:, 1:4]
    if ((p < lo) | (p > hi)).any():
        raise ValueError("the cloud does not belong to these poses: the trajectory leaves its bounding box (a map.ply of another run?)")


def scan_warning(poses, scans, play_rate):
    """The warning text when too few scans of the bag got a pose, else None. At a replay faster than real time it recommends relaunching at 1x."""
    if not scans or poses / scans >= LOST_RATIO:
        return None
    head = f"WARNING: only {poses} of {scans} scans of the bag got a pose ({100 * poses / scans:.1f} %): Voxel-SLAM lost data."
    if play_rate > 1:
        return head + f" The bag was replayed at {play_rate:g}x: relaunch in real time, PLAY_RATE=1 ops/make_map.sh BAG_NAME MAP_NAME."
    return head + " The replay was already real time: the bag itself lost scans (tools/slam_bench/bag_check.py)."


def commit():
    """Short git hash of the tools that made the map: MAKE_MAP_COMMIT (the image mounts the tools without .git), else git, else 'unknown'."""
    if os.environ.get("MAKE_MAP_COMMIT"):
        return os.environ["MAKE_MAP_COMMIT"]
    try:
        out = subprocess.run(["git", "-C", str(Path(__file__).resolve().parent), "rev-parse", "--short", "HEAD"], capture_output=True, text=True, timeout=5).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        out = ""
    return out or "unknown"


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--run", help="Voxel-SLAM run dir (alidarState.txt and one <i>.pcd per frame)")
    src.add_argument("--cloud", help="map.ply of an earlier run (needs --poses): redo everything after the SLAM without replaying it")
    ap.add_argument("--poses", help="alidarState.txt that goes with --cloud")
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--res", type=float, default=0.025, help="m, cell size of the map")
    ap.add_argument("--voxel", type=float, help="m, thinning of the cloud assembled from --run (default: --res)")
    ap.add_argument("--ceil", type=float, default=floor_level.OBST_HI, help="m above the floor: obstacle band ceiling (30 for an outdoor map)")
    ap.add_argument("--topic", default="/livox/lidar")
    ap.add_argument("--node-log", help="node.log of the SLAM run (default: <out>/node.log when it exists)")
    ap.add_argument("--play-rate", type=float, default=float(os.environ.get("PLAY_RATE", "1")))
    a = ap.parse_args(argv)
    out = Path(a.out)
    if a.run:
        poses_file = Path(a.run) / "alidarState.txt"
        poses, cloud = cloud_io.assemble_world_cloud(a.run, a.voxel or a.res)
    else:
        if not a.poses:
            ap.error("--cloud needs --poses")
        poses_file, cloud = Path(a.poses), cloud_io.read_ply(a.cloud)
        poses = np.loadtxt(poses_file, usecols=range(8), ndmin=2)
    try:
        if a.cloud:
            check_cloud_has_the_trajectory(cloud, poses)
        data = build(cloud, poses, a.bag, out, topic=a.topic, res=a.res, ceil=a.ceil)
    except ValueError as e:
        print(f"make_map: {e}", file=sys.stderr)
        return 1
    scans = map_report.scans_in_bag(a.bag, a.topic)
    warning = scan_warning(len(poses), scans, a.play_rate)
    if warning:
        print(warning, file=sys.stderr)
    node_log = Path(a.node_log) if a.node_log else out / "node.log"
    map_report.write_report(out, poses_file=poses_file, node_log=node_log, extra={
        "pipeline": 2, "commit": commit(), "play_rate": a.play_rate, "scans_in_bag": scans, "lost_scan_ratio": round(1 - len(poses) / scans, 4) if scans else None, "warning": warning,
        "floor_tilt_deg": data["floor_tilt_deg"], "sensor_height_m": data["sensor_height_m"], "ghost_band": data["ghost_band"], "rule": RULE})
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
