"""Test-only synthetic corridor whose beams are cast analytically (a Mid-360-like pattern, 1 degree, -7..51 degrees), shared by the grid2d, ghosts and make_map tests."""
import numpy as np
from scipy.spatial.transform import Rotation

import grid2d

RES = 0.05
SENSOR_H = 0.5
ORIGIN = (-1.0, -2.0)                 # map frame, lower-left corner of the grid
NX, NY = 240, 80                      # 12 x 4 m
LEN, HALF, CEIL = 10.0, 1.0, 2.4      # corridor along x from 0; walls at y = +-HALF
T = np.eye(4)                         # p_map = T p_world: a SLAM world that is neither aligned with the walls nor at floor height
T[:3, :3] = Rotation.from_euler("z", 90, degrees=True).as_matrix()
T[:3, 3] = [3.0, 1.0, 0.5]
AZ, EL = np.meshgrid(np.radians(np.arange(0, 360, 1.0)), np.radians(np.arange(-7, 52, 1.0)))
DIRS = np.c_[(np.cos(EL) * np.cos(AZ)).ravel(), (np.cos(EL) * np.sin(AZ)).ravel(), np.sin(EL).ravel()]


def planes(pillar=False):
    """(axis, value, {other axis: (lo, hi)}): walls, floor, ceiling, end caps and, optionally, a thin pillar at x = 5 (y 0.4..0.6)."""
    along, up = {0: (-1, LEN + 1), 2: (0, CEIL)}, {1: (-HALF, HALF), 2: (0, CEIL)}
    p = [(1, HALF, along), (1, -HALF, along), (2, 0.0, {0: (-1, LEN + 1), 1: (-HALF, HALF)}), (2, CEIL, {0: (-1, LEN + 1), 1: (-HALF, HALF)}), (0, -1.0, up), (0, LEN + 1, up)]
    return p + [(0, 5.0, {1: (0.4, 0.6), 2: (0, CEIL)})] if pillar else p


def cast(origin, planes):
    """Points (sensor axes) where a Mid-360-like pattern of beams (1 deg, -7..51 deg) first meets a plane."""
    best = np.full(len(DIRS), np.inf)
    for axis, value, bounds in planes:
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (value - origin[axis]) / DIRS[:, axis]
            p = origin + t[:, None] * DIRS
        ok = t > 0
        for ax, (lo, hi) in bounds.items():
            ok &= (p[:, ax] >= lo) & (p[:, ax] <= hi)
        best = np.where(ok & (t < best), t, best)
    fin = np.isfinite(best)
    return DIRS[fin] * best[fin, None]


def drive(planes):
    """Scans (points, R, t) as the tool takes them, and the path, both in the SLAM world: the sensor drives down the corridor axis; the lidar axes are the map axes."""
    Rm, tm = T[:3, :3], T[:3, 3]
    scans = []
    for x in np.arange(0, LEN + 0.1, 0.5):
        o = np.array([x, 0.0, SENSOR_H])
        scans.append((cast(o, planes), Rm.T, (o - tm) @ Rm))
    return scans, np.array([s[2] for s in scans])


def corridor_grid():
    """Map frame grid: both walls occupied, a strip of floor along the path already free, everything else unknown. Also returns the cell centres."""
    g = np.full((NY, NX), grid2d.UNK, np.uint8)
    X, Y = np.meshgrid(ORIGIN[0] + (np.arange(NX) + .5) * RES, ORIGIN[1] + (NY - 1 - np.arange(NY) + .5) * RES)   # row 0 = max y
    inside = (X > 0) & (X < LEN)
    g[inside & (np.abs(np.abs(Y) - HALF) < RES)] = grid2d.OCC
    g[inside & (np.abs(Y) < 0.2)] = grid2d.FREE
    return g, X, Y


def cell_at(x, y):
    return NY - 1 - int((y - ORIGIN[1]) / RES), int((x - ORIGIN[0]) / RES)


PHANTOM = np.random.default_rng(3).uniform([4.85, 0.35, 0.8], [5.15, 0.65, 1.1], (300, 3))   # map frame: 30 x 30 cm in the middle of the corridor where nothing is


def world_cloud(scans, extra=None):
    """The SLAM cloud in the SLAM world: every scan point, plus points given in the map frame that no beam ever hit. Also returns the count of real points and Q (the map-frame cloud)."""
    Rm, tm = T[:3, :3], T[:3, 3]
    P = np.vstack([pts @ Rm + t for pts, _, t in scans])
    n = len(P)
    if extra is not None:
        P = np.vstack([P, (extra - tm) @ Rm])
    return P, n, P @ Rm.T + tm
