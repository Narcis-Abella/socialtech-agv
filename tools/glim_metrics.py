#!/usr/bin/env python3
"""Map-quality metrics for GLIM dumps that need no ground truth.

Per dump:
- revisit: where the robot passes a place again (submaps >= REVISIT_GAP_S apart in time), distance
  from each later point to the nearest earlier point, both voxelized to VOXEL_M so configs with
  different densities compare equally. Only distances below REVISIT_CAP_M count (overlap region).
  A consistent map gives sampling-limited values (a few cm); a missed loop closure shows as doubled
  walls/floor, i.e. a larger median. revisit_fraction: share of later points that found a match.
- mme: Mean Map Entropy over the whole map voxelized to VOXEL_M: mean over points of
  0.5*ln(det(2*pi*e*cov)) of the neighbors within MME_RADIUS_M (points with fewer than
  MME_MIN_NEIGHBORS, or a degenerate covariance, are skipped). Lower = crisper; catches blur within a
  single pass (noise, deskew, odometry jitter), which the revisit metric does not see.
- trajectory (traj_lidar.txt): the robot drives on a flat floor, so z, roll and pitch should barely
  change; their ranges measure drift. end_to_start_m is informative only (the bag may not end
  where it started).
- submaps, matching_factors: from graph.txt.

Across dumps of the same bag (repeated runs): per timestamp, the largest distance between the
runs' positions; max and mean over the trajectory. All runs share the same gravity-aligned world
frame at the first frame, so no alignment is applied.

Usage: glim_metrics.py <dump_dir>...
Needs numpy and scipy (apt: python3-numpy python3-scipy).
"""
import itertools
import math
import pathlib
import sys

import numpy as np
from scipy.spatial import cKDTree

from glim_dump_to_ply import load_submap, read_matrix_after

VOXEL_M = 0.1
REVISIT_GAP_S = 30.0
REVISIT_CAP_M = 1.0
MME_RADIUS_M = 0.3
MME_MIN_NEIGHBORS = 5


def voxelize(points, size):
    """One point per occupied voxel (the first one), like a voxel-grid filter without averaging."""
    keys = np.floor(points / size).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(first)]


def submap_stamp(path):
    """Mean stamp of the submap's frames (data.txt 'stamp:' entries)."""
    tokens = (path / "data.txt").read_text(encoding="utf-8").split()
    stamps = [float(tokens[i + 1]) for i, t in enumerate(tokens) if t == "stamp:"]
    if not stamps:
        sys.exit(f"error: {path}/data.txt has no frame stamps")
    return sum(stamps) / len(stamps)


def load_world_submaps(dump, num_submaps):
    """[(mean stamp, voxelized world points Nx3)] sorted by stamp."""
    submaps = []
    for i in range(num_submaps):
        path = dump / f"{i:06d}"
        T, points, _ = load_submap(path)
        world = (T @ points.T).T[:, :3]
        submaps.append((submap_stamp(path), voxelize(world, VOXEL_M)))
    return sorted(submaps, key=lambda s: s[0])


def mme(points):
    """Mean Map Entropy of `points` (Nx3) after voxelization; None if no point has enough neighbors."""
    points = voxelize(points, VOXEL_M)
    neighbors = cKDTree(points).query_ball_point(points, MME_RADIUS_M)
    counts = np.array([len(n) for n in neighbors])
    center = np.repeat(np.arange(len(points)), counts)
    diff = points[np.concatenate(neighbors).astype(np.int64)] - points[center]  # local coords: no cancellation
    n = len(points)
    mean = np.stack([np.bincount(center, diff[:, a], n) for a in range(3)], axis=1) / counts[:, None]
    outer = np.stack([np.bincount(center, diff[:, a] * diff[:, b], n) for a in range(3) for b in range(3)], axis=1)
    cov = outer.reshape(n, 3, 3) / counts[:, None, None] - mean[:, :, None] * mean[:, None, :]
    det = np.linalg.det(2 * np.pi * np.e * cov)
    ok = (counts >= MME_MIN_NEIGHBORS) & (det > 0)
    return float(np.mean(0.5 * np.log(det[ok]))) if ok.any() else None


def revisit(submaps):
    distances, queried, tree, tree_size = [], 0, None, 0
    for stamp, points in submaps:
        past = [p for t, p in submaps if t < stamp - REVISIT_GAP_S]
        if not past:
            continue
        if len(past) != tree_size:  # past only grows along the sorted submaps
            tree, tree_size = cKDTree(np.vstack(past)), len(past)
        d, _ = tree.query(points, distance_upper_bound=REVISIT_CAP_M)
        distances.append(d[np.isfinite(d)])
        queried += len(points)
    if not queried:
        return {"revisit_median_m": None, "revisit_p90_m": None, "revisit_fraction": None}
    d = np.concatenate(distances)
    return {"revisit_median_m": float(np.median(d)) if len(d) else None,
            "revisit_p90_m": float(np.percentile(d, 90)) if len(d) else None,
            "revisit_fraction": len(d) / queried}


def read_traj(dump):
    """{stamp string: (x, y, z, qx, qy, qz, qw)} from traj_lidar.txt; stamps kept as text to match runs exactly."""
    path = dump / "traj_lidar.txt"
    try:
        lines = path.read_text(encoding="utf-8").split("\n")
    except OSError as e:
        sys.exit(f"error: cannot read {path}: {e}")
    traj = {}
    for line in lines:
        cols = line.split()
        if len(cols) == 8:
            traj[cols[0]] = np.array(cols[1:], dtype=np.float64)
    if not traj:
        sys.exit(f"error: {path} has no poses")
    return traj


def trajectory(dump):
    poses = np.array(list(read_traj(dump).values()))
    xyz, (qx, qy, qz, qw) = poses[:, :3], poses[:, 3:].T
    roll = np.unwrap(np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy)))
    pitch = np.arcsin(np.clip(2 * (qw * qy - qz * qx), -1, 1))
    return {"path_m": float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum()),
            "end_to_start_m": float(np.linalg.norm(xyz[-1] - xyz[0])),
            "z_range_m": float(np.ptp(xyz[:, 2])),
            "roll_range_deg": math.degrees(np.ptp(roll)),
            "pitch_range_deg": math.degrees(np.ptp(pitch))}


def dump_metrics(dump):
    dump = pathlib.Path(dump)
    if not (dump / "graph.txt").exists():
        sys.exit(f"error: {dump} is not a GLIM dump (no graph.txt)")
    num_submaps = int(read_matrix_after(dump / "graph.txt", "num_submaps:", 1, 1)[0, 0])
    factors = int(read_matrix_after(dump / "graph.txt", "num_matching_cost_factors:", 1, 1)[0, 0])
    submaps = load_world_submaps(dump, num_submaps)
    world = np.vstack([p for _, p in submaps]) if submaps else np.empty((0, 3))
    return {"submaps": num_submaps, "matching_factors": factors, **trajectory(dump), **revisit(submaps),
            "mme": mme(world) if len(world) else None}


def run_divergence(dumps):
    trajs = [read_traj(pathlib.Path(d)) for d in dumps]
    stamps = sorted(set.intersection(*(set(t) for t in trajs)), key=float)
    if not stamps:
        sys.exit("error: the runs share no trajectory timestamps (different bags?)")
    xyz = [np.array([t[s][:3] for s in stamps]) for t in trajs]
    worst = np.max([np.linalg.norm(a - b, axis=1) for a, b in itertools.combinations(xyz, 2)], axis=0)
    return {"max_m": float(worst.max()), "mean_m": float(worst.mean()), "common_poses": len(stamps)}


def fmt(v):
    return "-" if v is None else f"{v:.3f}" if isinstance(v, float) else str(v)


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: glim_metrics.py <dump_dir>...")
    rows = [(d, dump_metrics(d)) for d in sys.argv[1:]]
    keys = list(rows[0][1])
    print("\t".join(["dump", *keys]))
    for dump, m in rows:
        print("\t".join([dump, *(fmt(m[k]) for k in keys)]))
    if len(rows) > 1:
        div = run_divergence(sys.argv[1:])
        print(f"run divergence: max {div['max_m']:.3f} m, mean {div['mean_m']:.3f} m over {div['common_poses']} poses")


if __name__ == "__main__":
    main()
