"""Frees the UNKNOWN cells of a ply_to_map map that the LiDAR beams demonstrably crossed (the floor is seen sparsely, so reachable areas stay unknown).

  map_visibility.py <bag> <poses.txt> <map.yaml> <out_prefix> [--topic /livox/lidar]

Needs numpy + scipy (+ rosbags to read the bag). Inputs: the ROS 2 bag of the mapping run (sensor_msgs/PointCloud2), the SLAM poses (t x y z qx qy qz qw, the IMU pose
at the END of each scan, as Voxel-SLAM's alidarState.txt) and a map from ply_to_map (its YAML carries the PLY -> map transform). Writes a new PGM + YAML.
For every unknown cell within 8 m of the path, three heights above the floor (0.3, 0.9, 1.5 m) are checked in every raw scan with a range image (1 deg bins, nearest return):
  HIT    the beam ends within 5 cm (+1 % of the range) of the cell; PASSED the beam ends farther away (the cell is in the free space in front of the return).
A cell becomes FREE when it is PASSED >= 3 times at every height that was observed (at least 2 heights), HITs are <= 2 % of the observations, and it connects to cells that were already free.
Only unknown cells change: occupied and free cells are never touched. Not proof that nothing is there (overhangs, thin objects): it only replaces what would stay unknown,
and Nav2's live sensors still see obstacles. Measured on eco_-1_01: 27 387 cells (7.8 % of the map) freed, none changed otherwise. The lidar -> IMU translation is Voxel-SLAM's mid360.yaml (rotation = identity)."""
import argparse
from pathlib import Path

import numpy as np
from scipy.ndimage import label
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

import ply_to_map as pm

EXTR = (-0.011, -0.02329, 0.04412)      # m, lidar -> IMU translation (mid360.yaml)
HEIGHTS = (0.3, 0.9, 1.5)               # m above the floor
NEAR, RMAX, MARGIN = 8.0, 15.0, 0.05    # m: unknown cells this close to the path are candidates; beams farther than RMAX from a cell are not used; range margin of a HIT
N_PASS, MIN_HEIGHTS, MAX_HIT_RATIO = 3, 2, 0.02
MIN_RANGE = 0.5                         # m, the nearest returns are not used (Voxel-SLAM's blind zone)
EL_LO, EL_HI = -7.0, 52.0               # deg, the Mid-360's vertical field of view: the range image covers it in 1 deg bins
NEL = int(EL_HI - EL_LO)
MAX_GAP = 0.5                           # s, poses further apart than this are not interpolated
SCAN_MID = 0.05                         # s after the header stamp: the pose used for a 10 Hz Livox scan


def free_cells(hit, passed):
    """(cells, heights) beam counts -> bool per cell: the evidence rule described above."""
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


def fill_unknown(grid, res, origin, T, scans, traj, extr=EXTR, heights=HEIGHTS):
    """New grid (same layout: row 0 = max y) with the unknown cells freed. origin = (x, y) of the lower-left corner; T: PLY -> map; traj: (n, 3) positions in the PLY frame;
    scans: iterable of (points in the sensor frame, R, t) with the IMU pose in the PLY frame."""
    Rm, tm = T[:3, :3], T[:3, 3]
    extr = np.asarray(extr, float)
    rows, cols = np.nonzero(grid == pm.UNK)
    cx, cy = origin[0] + (cols + .5) * res, origin[1] + (grid.shape[0] - 1 - rows + .5) * res
    near = cKDTree((np.asarray(traj) @ Rm.T + tm)[:, :2]).query(np.c_[cx, cy], distance_upper_bound=NEAR)[0] < np.inf
    rows, cols, cx, cy = rows[near], cols[near], cx[near], cy[near]
    n, nh = len(rows), len(heights)
    if n == 0:
        return grid.copy()
    C = (np.column_stack([np.repeat(cx, nh), np.repeat(cy, nh), np.tile(heights, n)]) - tm) @ Rm   # candidates in the PLY frame, cell-major
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
        mg = MARGIN + 0.01 * rc
        seen = np.isfinite(rb)
        hit[idx[seen & (np.abs(rc - rb) <= mg)]] += 1
        passed[idx[seen & (rc < rb - mg)]] += 1
    mask = np.zeros(grid.shape, bool)
    free = free_cells(hit.reshape(n, nh), passed.reshape(n, nh))
    mask[rows[free], cols[free]] = True
    out = grid.copy()
    out[connected_to_free(mask, grid)] = pm.FREE
    return out


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


def read_map(yaml_path):
    """(grid, resolution, origin, T) of a ply_to_map map; T is its '# T_map_world' comment (p_map = T p_ply)."""
    yml = Path(yaml_path)
    lines = yml.read_text().splitlines()
    val = lambda key: next(l for l in lines if l.startswith(key)).split(":", 1)[1].strip()
    T = np.array([[float(x) for x in l[2:].strip().strip("[]").split(",")] for l in lines if l.startswith("# [")])
    if T.shape != (4, 4):
        raise ValueError(f"{yml}: no '# T_map_world' comment; make the map with ply_to_map")
    b = (yml.parent / val("image")).read_bytes()
    w, h = map(int, b.split(b"\n")[1].split())
    return np.frombuffer(b[-w * h:], np.uint8).reshape(h, w).copy(), float(val("resolution")), tuple(float(x) for x in val("origin").strip("[]").split(",")[:2]), T


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
    ap.add_argument("map_yaml")
    ap.add_argument("out_prefix")
    ap.add_argument("--topic", default="/livox/lidar")
    a = ap.parse_args(argv)
    grid, res, origin, T = read_map(a.map_yaml)
    poses = np.loadtxt(a.poses, usecols=range(8))
    out = fill_unknown(grid, res, origin, T, bag_scans(a.bag, poses, a.topic), poses[:, 1:4])
    pm.write_map(a.out_prefix, out, res, origin, T)
    print(f"freed {int(((grid == pm.UNK) & (out == pm.FREE)).sum())} cells: unknown {(grid == pm.UNK).mean():.1%} -> {(out == pm.UNK).mean():.1%}, free {(grid == pm.FREE).mean():.1%} -> {(out == pm.FREE).mean():.1%}")


if __name__ == "__main__":
    import sys
    main(sys.argv[1:])
