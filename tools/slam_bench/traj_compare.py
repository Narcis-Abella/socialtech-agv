"""Compares two Voxel-SLAM trajectories (alidarState.txt: t x y z qx qy qz qw) of the same bag, e.g. a run at 2x against the run at 1x.

  traj_compare.py <reference alidarState.txt> <test alidarState.txt>

coverage: share of the reference poses that have a test pose within 20 ms (scans the test run lost lower it; the node does not warn about them).
ATE: position error after the best rigid alignment (each run starts in its own arbitrary frame), over the matched poses: rmse, median and max, in metres."""
import sys

import numpy as np

MAX_DT = 0.02   # s: both runs timestamp the same scans, so a match is within a few ms (a scan period is 0.1 s)


def umeyama(src, dst):
    """Rigid (rotation + translation) transform R, t that best maps src onto dst, least squares."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((dst - mu_d).T @ (src - mu_s))
    S = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ S @ Vt
    return R, mu_d - R @ mu_s


def compare(ref, test):
    """dict(coverage, matched, ate_rmse, ate_median, ate_max) between two (n, 8) pose arrays sorted by time."""
    i = np.clip(np.searchsorted(test[:, 0], ref[:, 0]), 1, len(test) - 1)
    j = np.where(np.abs(test[i - 1, 0] - ref[:, 0]) <= np.abs(test[i, 0] - ref[:, 0]), i - 1, i)   # nearest test pose in time
    ok = np.abs(test[j, 0] - ref[:, 0]) <= MAX_DT
    n = int(ok.sum())
    r = dict(coverage=n / len(ref), matched=n, ate_rmse=float("nan"), ate_median=float("nan"), ate_max=float("nan"))
    if n >= 3:
        a, b = test[j[ok], 1:4], ref[ok, 1:4]
        R, t = umeyama(a, b)
        e = np.linalg.norm(a @ R.T + t - b, axis=1)
        r.update(ate_rmse=float(np.sqrt((e ** 2).mean())), ate_median=float(np.median(e)), ate_max=float(e.max()))
    return r


def main(argv):
    ref, test = (np.loadtxt(p, usecols=range(8)) for p in argv[:2])
    ref, test = ref[np.argsort(ref[:, 0])], test[np.argsort(test[:, 0])]
    r = compare(ref, test)
    print(f"coverage {r['coverage']:.1%} ({r['matched']} of {len(ref)} reference poses) | ATE rmse {r['ate_rmse']:.3f} m, median {r['ate_median']:.3f} m, max {r['ate_max']:.3f} m")


if __name__ == "__main__":
    main(sys.argv[1:])
