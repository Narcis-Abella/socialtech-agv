"""The Python reference the C++ Binner is checked against (golden.txt): points of a frame -> 720 ranges, as the recorded /scan was reproduced from the raw cloud
(93-95 % of beams valid in both, median 16-36 mm). Copied from to_scan of notes/nav_real_scripts/scan_study.py of the team's workspace repository, which
imports other modules of it, so only this function and its constants live here. make_golden.py regenerates test/golden.txt from it."""
import numpy as np

A_MIN, INC, NB = -3.14159, 0.00873, 720
MIN_H, MAX_H = 0.10, 1.80


def to_scan(x, y, z, h, blind3d=0.5, rmin=0.5, rmax=60.0, every=3, how="min", thin_near=0.0):
    """x, y, z of a frame in the sensor frame (z up), h = sensor height above the floor -> 720 ranges (inf = no return)"""
    if every > 1: x, y, z = x[::every], y[::every], z[::every]
    ok = (x * x + y * y + z * z > blind3d ** 2); x, y, z = x[ok], y[ok], z[ok]; hh = z + h; r = np.hypot(x, y)
    ok = (hh > MIN_H) & (hh < MAX_H) & (r >= rmin) & (r < rmax); x, y, r = x[ok], y[ok], r[ok]
    b = np.floor((np.arctan2(y, x) - A_MIN) / INC).astype(int); keep = (b >= 0) & (b < NB); b, r = b[keep], r[keep]; out = np.full(NB, np.inf)
    q = {"min": 0.0, "median": 0.5}.get(how, how)   # how: "min", "median" or the quantile (0..1) of the ranges of the bin
    if len(b):
        o = np.lexsort((r, b)); b2, r2 = b[o], r[o]; u, first, cnt = np.unique(b2, return_index=True, return_counts=True); out[u] = r2[first + np.floor(q * (cnt - 1) + 1e-9).astype(int)]
    if thin_near > 0:   # AMCL (max_beams 180 of 720) looks at every 4th beam: blank 2 of every 3 of THOSE that are closer than thin_near m, so the far ones weigh relatively more
        idx = np.arange(NB); out[(idx % 4 == 0) & ((idx // 4) % 3 != 0) & (out < thin_near)] = np.inf
    return out, np.bincount(b, minlength=NB)
