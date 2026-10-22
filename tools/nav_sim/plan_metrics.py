"""Metrics of a planned path (list of [x, y]) against a map (offline: numpy, scipy via metrics.py). Clearance uses the robot footprint along the path heading and is capped at 5 m;
turning = total absolute heading change; max_curvature = largest heading change over 0.25 m of path divided by 0.25 (1/m); sharp_turns = separate stretches where the heading changes more
than 0.7 rad within 0.5 m; cusps = vertices where the path reverses on itself (turn > 150 deg)."""
import math

import numpy as np

import metrics as m

STEP = 0.05
EMPTY = {"length": 0.0, "turning": 0.0, "max_curvature": 0.0, "sharp_turns": 0, "cusps": 0, "clearance_min": 5.0, "clearance_p5": 5.0}


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def path_metrics(path, ctx):
    path = np.asarray(path, float)
    if len(path) < 2:
        return dict(EMPTY)
    seg = np.diff(path, axis=0)
    seg = seg[np.hypot(*seg.T) > 1e-9]
    if not len(seg):
        return dict(EMPTY)
    length = float(np.sum(np.hypot(*seg.T)))
    head = np.arctan2(seg[:, 1], seg[:, 0])
    cusps = int(np.sum(np.abs(wrap(np.diff(head))) > math.radians(150)))
    pts = np.vstack([path[:1], path[1:][np.hypot(*np.diff(path, axis=0).T) > 1e-9]])
    arc = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
    grid = np.append(np.arange(0.0, length, STEP), length)
    xy = np.column_stack([np.interp(grid, arc, pts[:, 0]), np.interp(grid, arc, pts[:, 1])])
    d = np.gradient(xy, axis=0)
    h = np.arctan2(d[:, 1], d[:, 0])
    turning = float(np.sum(np.abs(wrap(np.diff(h)))))
    k, w = int(round(0.25 / STEP)), int(round(0.5 / STEP))
    curv = np.abs(wrap(h[k:] - h[:-k])) / (k * STEP) if len(h) > k else np.zeros(1)
    sharp = np.abs(wrap(h[w:] - h[:-w])) > 0.7 if len(h) > w else np.zeros(1, bool)
    runs = int(np.sum(sharp[1:] & ~sharp[:-1]) + (1 if sharp[0] else 0))
    cl = np.array([m.footprint_clearance(ctx["dist"], ctx["origin"], ctx["res"], x, y, yaw, *ctx["footprint"]) for (x, y), yaw in zip(xy[::2], h[::2])])
    cl = np.minimum(cl, 5.0)
    return {"length": length, "turning": turning, "max_curvature": float(curv.max()), "sharp_turns": runs, "cusps": cusps,
            "clearance_min": float(cl.min()), "clearance_p5": float(np.percentile(cl, 5))}
