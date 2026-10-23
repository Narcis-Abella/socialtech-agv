"""How good is the /scan that AMCL and the costmaps get (pointcloud_to_laserscan on FAST-LIO2's deskewed cloud)? Offline, on the /scan recorded by the stage A bench (RECORD=1) and the Voxel-SLAM
reference: share of beams with data, scan rate, and how far the beam endpoints, placed at the REFERENCE pose, land from the nearest occupied map cell (by range band; the map and the reference carry
their own error of a few cm, so a floor of ~5 cm is expected).
usage: scan_quality.py <inputs bag dir> <map.yaml (with T_map_world)> <alidarState.txt> [--max-scans N]"""
import math
import os
import sys

import numpy as np

import metrics

BANDS = (0.5, 2.0, 5.0, 10.0, 20.0, 60.0)
WITHIN = (0.05, 0.10, 0.30)


def endpoints(ranges, angle_min, inc, pose, return_index=False):
    """Map-frame (x, y) of the finite beams of one scan taken at pose (x, y, yaw)."""
    r = np.asarray(ranges, float)
    idx = np.flatnonzero(np.isfinite(r))
    a = angle_min + inc * idx + pose[2]
    pts = np.column_stack([pose[0] + r[idx] * np.cos(a), pose[1] + r[idx] * np.sin(a)])
    return (pts, idx) if return_index else pts


def distance_map(occ, res):
    return metrics.distance_to_occupied(occ, res)


def endpoint_map_distance(ranges, angle_min, inc, pose, dist, res, origin):
    """(ranges, distance to the nearest occupied cell centre) of the finite beams whose endpoint falls on the map."""
    pts, idx = endpoints(ranges, angle_min, inc, pose, return_index=True)
    h, w = dist.shape
    col = np.floor((pts[:, 0] - origin[0]) / res).astype(int)
    row = h - 1 - np.floor((pts[:, 1] - origin[1]) / res).astype(int)
    ok = (col >= 0) & (col < w) & (row >= 0) & (row < h)
    return np.asarray(ranges, float)[idx][ok], dist[row[ok], col[ok]]


def band_stats(r, d, bands=BANDS, within=WITHIN):
    out = []
    for lo, hi in zip(bands[:-1], bands[1:]):
        m = (r >= lo) & (r < hi)
        if not m.any():
            out.append({"band": (lo, hi), "n": 0, "median": math.nan, "p95": math.nan, "within": [math.nan] * len(within)})
            continue
        dd = d[m]
        out.append({"band": (lo, hi), "n": int(m.sum()), "median": float(np.median(dd)), "p95": float(np.percentile(dd, 95)), "within": [float(np.mean(dd <= w)) for w in within]})
    return out


def static_pairs(poses, pos_thr=0.005, yaw_thr=0.0009):
    """Flags (N-1) of the consecutive scan pairs between which the reference pose did not move (displacement < pos_thr m and rotation < yaw_thr rad; NaN poses are never static)."""
    p = np.asarray(poses, float)
    d = np.hypot(*np.diff(p[:, :2], axis=0).T)
    dy = np.abs((np.diff(p[:, 2]) + np.pi) % (2 * np.pi) - np.pi)
    with np.errstate(invalid="ignore"):
        return (d < pos_thr) & (dy < yaw_thr)


def frame_jitter(scans, pair_mask, max_step=0.3):
    """Between consecutive scans of a still robot the same bin should give the same range. Returns (mean range, range difference) of the bins finite in both frames and closer than max_step
    (same surface), and the flicker: bins finite in exactly one of the two frames over bins finite in either."""
    s = np.asarray(scans, float)
    m = np.asarray(pair_mask, bool)
    a, b = s[:-1][m], s[1:][m]
    fa, fb = np.isfinite(a), np.isfinite(b)
    both = fa & fb
    d = b[both] - a[both]
    r = 0.5 * (a[both] + b[both])
    keep = np.abs(d) < max_step
    either = (fa | fb).sum()
    return r[keep], d[keep], (float((fa ^ fb).sum() / either) if either else math.nan)


def flicker_bands(scans, pair_mask, bands=BANDS):
    """Flicker (see frame_jitter) per range band, the range being that of the frame in which the bin has data."""
    s = np.asarray(scans, float)
    m = np.asarray(pair_mask, bool)
    a, b = s[:-1][m], s[1:][m]
    fa, fb = np.isfinite(a), np.isfinite(b)
    r = np.where(fa, a, b)
    out = []
    for lo, hi in zip(bands[:-1], bands[1:]):
        either = (fa | fb) & (r >= lo) & (r < hi)
        out.append(float(((fa ^ fb) & either).sum() / either.sum()) if either.sum() else math.nan)
    return out


def finite_share(scans):
    return np.mean(np.isfinite(np.asarray(scans, float)), axis=1)


def read_scans(bag_dir, topic="/scan"):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    t, rs, meta = [], [], None
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name == topic:
            m = deserialize_message(data, LaserScan)
            t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            rs.append(np.array(m.ranges, float))
            meta = (m.angle_min, m.angle_increment, m.range_min, m.range_max)
    return np.array(t), np.array(rs), meta


def main(bag, map_yaml, ref_path, max_scans=None):
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "nav_bench"))
    import pose_eval as pe
    t, scans, (amin, inc, rmin, rmax) = read_scans(bag)
    m = pe.read_map_yaml(map_yaml)
    grid = pe.read_pgm(m["image"])
    dist = distance_map(grid < 50, m["resolution"])
    t_ref, ref = pe.reference_map(ref_path, m["T_map_world"])
    poses = pe.interp(t_ref, ref, t)
    keep = np.flatnonzero(np.isfinite(poses[:, 0]))
    if max_scans:
        keep = keep[:: max(1, len(keep) // max_scans)]
    R, D = [], []
    for i in keep:
        r, d = endpoint_map_distance(scans[i], amin, inc, poses[i], dist, m["resolution"], m["origin"])
        R.append(r)
        D.append(d)
    R, D = np.concatenate(R), np.concatenate(D)
    fs = finite_share(scans)
    nb = scans.shape[1]
    print(f"{len(t)} scans of {nb} beams ({math.degrees(inc):.2f} deg), rate {1 / np.median(np.diff(t)):.1f} Hz, scans on the reference {len(keep)}")
    print(f"beams with data per scan: median {100 * np.median(fs):.0f} % (p5 {100 * np.percentile(fs, 5):.0f} %, p95 {100 * np.percentile(fs, 95):.0f} %) = {np.median(fs) * nb:.0f} of {nb}")
    print(f"endpoint to nearest occupied map cell (all ranges): median {100 * np.median(D):.1f} cm, p95 {100 * np.percentile(D, 95):.1f} cm; within 5 cm {100 * np.mean(D <= 0.05):.0f} %, 10 cm {100 * np.mean(D <= 0.10):.0f} %, 30 cm {100 * np.mean(D <= 0.30):.0f} %")
    print(f"{'range band (m)':>16s} {'beams':>8s} {'share':>6s} {'median cm':>10s} {'p95 cm':>8s} {'<=5cm':>6s} {'<=10cm':>7s} {'<=30cm':>7s}")
    for b in band_stats(R, D):
        if b["n"]:
            print(f"{b['band'][0]:>7.1f}-{b['band'][1]:<7.1f} {b['n']:8d} {100 * b['n'] / len(R):5.0f}% {100 * b['median']:10.1f} {100 * b['p95']:8.1f} {100 * b['within'][0]:5.0f}% {100 * b['within'][1]:6.0f}% {100 * b['within'][2]:6.0f}%")


if __name__ == "__main__":
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    n = int(sys.argv[sys.argv.index("--max-scans") + 1]) if "--max-scans" in sys.argv else None
    main(a[0], a[1], a[2], n)
