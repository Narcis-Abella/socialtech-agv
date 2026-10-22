"""Navigation metrics from recorded series (offline: numpy and scipy). Poses are (x, y) in the map frame, times in seconds.
Definitions: SPL = S * l / max(p, l) (Anderson et al., 2018); jerk and acceleration come from the commanded velocity resampled to a fixed step, so
two runs with different controller rates are compared on the same grid; clearance is the distance from the robot footprint (not its centre) to the nearest occupied cell."""
import math

import numpy as np
from scipy import ndimage


def path_length(xy):
    xy = np.asarray(xy, float)
    return float(np.sum(np.hypot(*np.diff(xy, axis=0).T))) if len(xy) > 1 else 0.0


def spl(success, shortest, actual):
    if not success or shortest <= 0:
        return 0.0
    return shortest / max(actual, shortest)


def resample(t, v, dt):
    """Linear interpolation of v(t) on a uniform grid from t[0] to t[-1] (inclusive)."""
    t, v = np.asarray(t, float), np.asarray(v, float)
    grid = np.arange(t[0], t[-1] + 1e-9, dt)
    return grid, np.interp(grid, t, v)


def acc_jerk(t, v):
    """Acceleration and jerk of a uniformly sampled velocity."""
    acc = np.gradient(v, t)
    return acc, np.gradient(acc, t)


def sign_reversals(v, deadband):
    """Number of sign changes of v, ignoring samples with |v| <= deadband (oscillation count)."""
    s = np.sign(np.asarray(v, float)[np.abs(v) > deadband])
    return int(np.count_nonzero(s[1:] != s[:-1]))


def cross_track(path, traj):
    """Distance from each trajectory point to the nearest point of the path polyline."""
    path, traj = np.asarray(path, float), np.asarray(traj, float)
    a, b = path[:-1], path[1:]
    ab = b - a
    denom = np.maximum(np.sum(ab * ab, axis=1), 1e-12)
    out = np.empty(len(traj))
    for i, p in enumerate(traj):
        u = np.clip(np.sum((p - a) * ab, axis=1) / denom, 0.0, 1.0)
        out[i] = np.min(np.hypot(*(a + u[:, None] * ab - p).T))
    return out


def distance_to_occupied(occ, res):
    """Distance (m) from each cell centre to the nearest occupied cell centre; 0 on occupied cells. occ[row, col], row 0 = top of the image."""
    return ndimage.distance_transform_edt(~np.asarray(occ, bool)) * res


def footprint_clearance(dist, origin, res, x, y, yaw, length, width):
    """Minimum distance from the footprint rectangle (length along heading, width across) to occupied cells; <= 0 when it covers one; inf off the map."""
    h, w = dist.shape
    n_l, n_w = int(math.ceil(length / (res / 2))) + 1, int(math.ceil(width / (res / 2))) + 1
    u, v = np.meshgrid(np.linspace(-length / 2, length / 2, n_l), np.linspace(-width / 2, width / 2, n_w))
    c, s = math.cos(yaw), math.sin(yaw)
    px, py = x + c * u - s * v, y + s * u + c * v
    col = np.floor((px - origin[0]) / res).astype(int)
    row = h - 1 - np.floor((py - origin[1]) / res).astype(int)
    inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
    return float(dist[row[inside], col[inside]].min()) if inside.any() else float("inf")


def collides(dist, origin, res, x, y, yaw, length, width):
    return footprint_clearance(dist, origin, res, x, y, yaw, length, width) <= 0.0


def rms(x):
    x = np.asarray(x, float)
    return float(np.sqrt(np.mean(x * x))) if x.size else float("nan")


def p95(x):
    x = np.asarray(x, float)
    return float(np.percentile(x, 95)) if x.size else float("nan")
