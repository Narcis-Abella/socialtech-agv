"""Test-only numpy beam counter: the reference the C++ engine (libsocialtech_mapping_beams.so) is checked against, and the counter the grid2d tests inject.
Same rules as the engine: a point is HIT when a beam of the range image (1 degree bins, nearest return) ends within margin + 1 % of its range, PASSED when it ends farther."""
import numpy as np
from scipy.spatial import cKDTree

EXTR = (-0.011, -0.02329, 0.04412)      # m, lidar -> IMU translation (mid360.yaml)
RMAX = 15.0                             # m: beams farther than this from a point are not used
MIN_RANGE = 0.1                         # m, the nearest returns are not used (the Mid-360 blind zone; Voxel-SLAM blind: 0.1 as well)
EL_LO, EL_HI = -7.0, 52.0               # deg, the Mid-360's vertical field of view: the range image covers it in 1 deg bins
NEL = int(EL_HI - EL_LO)
NEL = int(EL_HI - EL_LO)


def range_image(xyz, rng):
    """Nearest return per 1 deg azimuth/elevation bin (inf = nothing seen), flat [azimuth * NEL + elevation]."""
    az = (np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0])) + 180.0) % 360.0
    el = np.degrees(np.arcsin(xyz[:, 2] / rng))
    k = (el >= EL_LO) & (el < EL_HI)
    img = np.full(360 * NEL, np.inf)
    np.minimum.at(img, np.minimum(az[k].astype(int), 359) * NEL + (el[k] - EL_LO).astype(int), rng[k])
    return img


def count_beams(C, scans, extr=EXTR, margin=0.05):
    """(hit, passed) per candidate point C: scans is an iterable of (sensor-frame points, R, t); the reference the C++ engine is checked against."""
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
