"""2D map of a SLAM cloud cleaned with the raw scans: drops points beams pass through, and frees unknown cells beams crossed.

  map_visibility.py <bag> <poses.txt> <world.ply> <out_prefix> [--topic /livox/lidar] [--min-ratio 0.10] [--min-component 10] [--no-fill] [--ply-out cloud.ply]

Needs numpy + scipy (+ rosbags to read the bag). Inputs: the ROS 2 bag of the mapping run (sensor_msgs/PointCloud2), the SLAM poses (t x y z qx qy qz qw, the IMU pose at the END of each
scan, as Voxel-SLAM's alidarState.txt) and the SLAM cloud in the same world (x y z PLY). The cloud is levelled and aligned like ply_to_map does; the bag is read twice.
Every check is a beam test: with a range image per scan (1 deg bins, nearest return), a point is HIT when a beam ends within a margin (+1 % of the range) of it, and PASSED when the beam ends farther away.
1. Ghosts (--min-ratio): cloud points in the obstacle band (0.10-1.80 m) are grouped in 10 cm voxels; a voxel with >= 10 observations and hits / (hits + passes) < 0.10 is dropped (margin 15 cm):
   thin or dim things the beams mostly cross (walls measure ~0.75, isolated specks ~0.01 on eco_-1_01). Then the map is rasterised and occupied islands under --min-component cells are freed.
2. Unknown cells (unless --no-fill): each unknown cell within 8 m of the path, at three heights above the floor (0.3, 0.9, 1.5 m), becomes FREE when it is PASSED >= 3 times at every observed height (at least 2 heights),
   HITs are <= 2 % of the observations (margin 5 cm) and it connects to cells that were already free. Only unknown cells change. Not proof that nothing is there (overhangs, thin objects): it only replaces
   what would stay unknown, and Nav2's live sensors still see obstacles.
Measured on eco_-1_01: 27 387 cells (7.8 % of the map) freed by step 2. The lidar -> IMU translation is Voxel-SLAM's mid360.yaml (rotation = identity)."""
import argparse
import sys
from pathlib import Path

import numpy as np
from scipy.ndimage import label
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

import ply_to_map as pm

EXTR = (-0.011, -0.02329, 0.04412)      # m, lidar -> IMU translation (mid360.yaml)
RMAX = 15.0                             # m: beams farther than this from a point are not used
MIN_RANGE = 0.5                         # m, the nearest returns are not used (Voxel-SLAM's blind zone)
EL_LO, EL_HI = -7.0, 52.0               # deg, the Mid-360's vertical field of view: the range image covers it in 1 deg bins
NEL = int(EL_HI - EL_LO)
RATIO_VOX, RATIO_MARGIN, MIN_OBS, MIN_RATIO, MIN_COMPONENT = 0.10, 0.15, 10, 0.10, 10   # ghosts: voxel (m), HIT margin (m), observations needed, hit ratio under which a voxel is a ghost, cells of the smallest island kept
HEIGHTS = (0.3, 0.9, 1.5)               # m above the floor
NEAR, FILL_MARGIN = 8.0, 0.05           # m: unknown cells this close to the path are candidates; HIT margin (m)
N_PASS, MIN_HEIGHTS, MAX_HIT_RATIO = 3, 2, 0.02
MAX_GAP = 0.5                           # s, poses further apart than this are not interpolated
SCAN_MID = 0.05                         # s after the header stamp: the pose used for a 10 Hz Livox scan


def ghost_voxels(hit, passed, min_ratio=MIN_RATIO):
    """bool per voxel from its beam counts: enough observations and a hit ratio under min_ratio."""
    nobs = hit + passed
    return (nobs >= MIN_OBS) & (hit / np.maximum(nobs, 1) < min_ratio)


def free_cells(hit, passed):
    """(cells, heights) beam counts -> bool per cell: the evidence rule of step 2."""
    seen = hit + passed > 0
    enough = ((passed >= N_PASS) | ~seen).all(1) & (seen.sum(1) >= MIN_HEIGHTS)
    h, p = hit.sum(1), passed.sum(1)
    return enough & (h <= MAX_HIT_RATIO * (h + p))


def connected_to_free(mask, grid):
    """The cells of mask (8-connected) that touch known free space, directly or through other cells of mask."""
    lab, _ = label(mask | (grid == pm.FREE), structure=np.ones((3, 3)))
    return mask & np.isin(lab, np.unique(lab[grid == pm.FREE]))


def range_image(xyz, rng):
    """Nearest return per 1 deg azimuth/elevation bin (inf = nothing seen), flat [azimuth * NEL + elevation]."""
    az = (np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0])) + 180.0) % 360.0
    el = np.degrees(np.arcsin(xyz[:, 2] / rng))
    k = (el >= EL_LO) & (el < EL_HI)
    img = np.full(360 * NEL, np.inf)
    np.minimum.at(img, np.minimum(az[k].astype(int), 359) * NEL + (el[k] - EL_LO).astype(int), rng[k])
    return img


def count_beams(C, scans, extr=EXTR, margin=FILL_MARGIN):
    """(hit, passed) per candidate point C (n, 3, PLY frame): in how many scans a beam ended at the point (within margin + 1 % of its range) or went through it and ended farther.
    scans: iterable of (points in the sensor frame, R, t) with the IMU pose in the PLY frame."""
    extr = np.asarray(extr, float)
    tree = cKDTree(C)
    hit, passed = np.zeros(len(C), np.int32), np.zeros(len(C), np.int32)
    for xyz, R, t in scans:
        rng = np.linalg.norm(xyz, axis=1)
        far = rng > MIN_RANGE
        img = range_image(xyz[far], rng[far])
        idx = np.array(tree.query_ball_point(R @ extr + t, RMAX), dtype=np.int64)
        if len(idx) == 0:
            continue
        cl = (C[idx] - t) @ R - extr                                    # candidates in the sensor frame
        rc = np.linalg.norm(cl, axis=1)
        ac = (np.degrees(np.arctan2(cl[:, 1], cl[:, 0])) + 180.0) % 360.0
        ec = np.degrees(np.arcsin(np.clip(cl[:, 2] / np.maximum(rc, 1e-9), -1, 1)))
        vis = (rc > MIN_RANGE) & (ec > EL_LO) & (ec < EL_HI)
        idx, rc, ac, ec = idx[vis], rc[vis], ac[vis], ec[vis]
        rb = img[np.minimum(ac.astype(int), 359) * NEL + np.clip((ec - EL_LO).astype(int), 0, NEL - 1)]
        mg = margin + 0.01 * rc
        seen = np.isfinite(rb)
        hit[idx[seen & (np.abs(rc - rb) <= mg)]] += 1
        passed[idx[seen & (rc < rb - mg)]] += 1
    return hit, passed


def ghost_points(P, Q, scans, min_ratio=MIN_RATIO, extr=EXTR):
    """bool per cloud point: it lies in an obstacle-band voxel that is a ghost. P: points in the PLY frame, Q: the same points levelled and aligned (z = height above the floor)."""
    band = (Q[:, 2] > pm.OBST_LO) & (Q[:, 2] <= pm.OBST_HI)
    uk, inv = np.unique(np.floor(P[band] / RATIO_VOX).astype(np.int64), axis=0, return_inverse=True)
    hit, passed = count_beams((uk + 0.5) * RATIO_VOX, scans, extr, RATIO_MARGIN)
    out = np.zeros(len(P), bool)
    out[band] = ghost_voxels(hit, passed, min_ratio)[inv.ravel()]
    return out


def fill_unknown(grid, res, origin, T, scans, traj, extr=EXTR, heights=HEIGHTS):
    """New grid (same layout: row 0 = max y) with the unknown cells freed (step 2). origin = (x, y) of the lower-left corner; T: PLY -> map; traj: (n, 3) positions in the PLY frame."""
    Rm, tm = T[:3, :3], T[:3, 3]
    rows, cols = np.nonzero(grid == pm.UNK)
    cx, cy = origin[0] + (cols + .5) * res, origin[1] + (grid.shape[0] - 1 - rows + .5) * res
    near = cKDTree((np.asarray(traj) @ Rm.T + tm)[:, :2]).query(np.c_[cx, cy], distance_upper_bound=NEAR)[0] < np.inf
    rows, cols, cx, cy = rows[near], cols[near], cx[near], cy[near]
    n, nh = len(rows), len(heights)
    if n == 0:
        return grid.copy()
    C = (np.column_stack([np.repeat(cx, nh), np.repeat(cy, nh), np.tile(heights, n)]) - tm) @ Rm   # candidates in the PLY frame, cell-major
    hit, passed = count_beams(C, scans, extr, FILL_MARGIN)
    mask = np.zeros(grid.shape, bool)
    free = free_cells(hit.reshape(n, nh), passed.reshape(n, nh))
    mask[rows[free], cols[free]] = True
    out = grid.copy()
    out[connected_to_free(mask, grid)] = pm.FREE
    return out


def build_map(P, Q, T, scans, traj, res=0.05, min_ratio=MIN_RATIO, min_component=MIN_COMPONENT, fill=True, extr=EXTR):
    """(grid, origin, Q) of the cleaned map; Q is the cloud without the ghosts. P: SLAM cloud in the PLY frame; Q = the same points levelled and aligned by ply_to_map (T: P -> Q);
    scans: zero-argument function returning the scans (the bag is read twice); traj: (n, 3) path in the PLY frame. min_ratio 0 skips the ghost step."""
    if min_ratio > 0:
        Q = Q[~ghost_points(P, Q, scans(), min_ratio, extr)]
    grid, origin = pm.rasterize(Q, res, min_component=min_component)
    if fill:
        grid = fill_unknown(grid, res, origin, T, scans(), traj, extr)
    return grid, origin, Q


def occupied_points(Q, grid, origin, res=0.05):
    """The points of Q in the obstacle band that lie over an occupied cell of grid (row 0 = max y): the cloud the map shows."""
    Q = Q[(Q[:, 2] > pm.OBST_LO) & (Q[:, 2] <= pm.OBST_HI)]
    c = np.floor((Q[:, 0] - origin[0]) / res).astype(int)
    r = grid.shape[0] - 1 - np.floor((Q[:, 1] - origin[1]) / res).astype(int)
    ok = (r >= 0) & (r < grid.shape[0]) & (c >= 0) & (c < grid.shape[1])
    keep = np.zeros(len(Q), bool)
    keep[ok] = grid[r[ok], c[ok]] == pm.OCC
    return Q[keep]


def pose_at(poses, t):
    """(R, position) at time t from poses (t x y z qx qy qz qw): linear interpolation between the two around it (normalised quaternion average). None outside the file or across a gap."""
    pt = poses[:, 0]
    j = np.searchsorted(pt, t)
    if j == 0 or j == len(pt) or pt[j] - pt[j - 1] > MAX_GAP:
        return None
    f = (t - pt[j - 1]) / max(pt[j] - pt[j - 1], 1e-9)
    q0, q1 = poses[j - 1, 4:8], poses[j, 4:8]
    q1 = q1 if q0 @ q1 >= 0 else -q1
    return Rotation.from_quat((1 - f) * q0 + f * q1).as_matrix(), (1 - f) * poses[j - 1, 1:4] + f * poses[j, 1:4]


def bag_scans(bag, poses, topic="/livox/lidar"):
    """(points in the sensor frame, R, t) for each scan of a ROS 2 bag of sensor_msgs/PointCloud2, with the pose at its middle; scans without a pose are skipped. Needs rosbags."""
    from rosbags.highlevel import AnyReader
    with AnyReader([Path(bag)]) as r:
        for c, _, raw in r.messages(connections=[c for c in r.connections if c.topic == topic]):
            msg = r.deserialize(raw, c.msgtype)
            p = pose_at(poses, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 + SCAN_MID)
            if p is None:
                continue
            off = {f.name: f.offset for f in msg.fields}
            d = np.asarray(msg.data).reshape(-1, msg.point_step)
            xyz = np.column_stack([d[:, off[k]:off[k] + 4].copy().view("<f4")[:, 0] for k in "xyz"]).astype(float)
            yield xyz[np.isfinite(xyz).all(1)], p[0], p[1]


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("poses")
    ap.add_argument("world_ply")
    ap.add_argument("out_prefix")
    ap.add_argument("--topic", default="/livox/lidar")
    ap.add_argument("--min-ratio", type=float, default=MIN_RATIO, help="hit ratio under which a voxel of 10 cm is a ghost (0 = skip the step)")
    ap.add_argument("--min-component", type=int, default=MIN_COMPONENT, help="cells: occupied islands smaller than this are freed (0 = off)")
    ap.add_argument("--no-fill", action="store_true", help="skip freeing the unknown cells the beams crossed")
    ap.add_argument("--ply-out", help="also save the cloud the map shows (obstacle band, over occupied cells, in the map's frame), to inspect it")
    a = ap.parse_args(argv)
    P = pm.read_ply(a.world_ply)
    try:
        Q, info = pm.level_to_floor(P)
    except ValueError as e:
        sys.exit(f"error: {e}")
    poses = np.loadtxt(a.poses, usecols=range(8))
    grid, origin, Qk = build_map(P, Q, info["T"], lambda: bag_scans(a.bag, poses, a.topic), poses[:, 1:4], min_ratio=a.min_ratio, min_component=a.min_component, fill=not a.no_fill)
    pm.write_map(a.out_prefix, grid, 0.05, origin, info["T"])
    if a.ply_out:
        pm.write_ply(a.ply_out, occupied_points(Qk, grid, origin))
    print(f"{grid.shape[1]}x{grid.shape[0]} cells: {(grid == pm.OCC).mean():.1%} occupied, {(grid == pm.FREE).mean():.1%} free, {(grid == pm.UNK).mean():.1%} unknown")


if __name__ == "__main__":
    main(sys.argv[1:])
