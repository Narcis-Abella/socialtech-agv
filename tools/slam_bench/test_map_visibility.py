"""Checks map_visibility.py on a synthetic corridor whose beams are cast analytically. Run: python3 test_map_visibility.py"""
import numpy as np
from scipy.spatial.transform import Rotation

import map_visibility as mv
import ply_to_map as pm

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
    g = np.full((NY, NX), pm.UNK, np.uint8)
    X, Y = np.meshgrid(ORIGIN[0] + (np.arange(NX) + .5) * RES, ORIGIN[1] + (NY - 1 - np.arange(NY) + .5) * RES)   # row 0 = max y
    inside = (X > 0) & (X < LEN)
    g[inside & (np.abs(np.abs(Y) - HALF) < RES)] = pm.OCC
    g[inside & (np.abs(Y) < 0.2)] = pm.FREE
    return g, X, Y


def cell_at(x, y):
    return NY - 1 - int((y - ORIGIN[1]) / RES), int((x - ORIGIN[0]) / RES)


def test_unknown_floor_the_beams_crossed_is_freed_and_the_outside_stays_unknown():
    g0, X, Y = corridor_grid()
    scans, traj = drive(planes())
    g = mv.fill_unknown(g0, RES, ORIGIN, T, scans, traj, extr=(0, 0, 0))
    side = (X > 1) & (X < LEN - 1) & (np.abs(Y) > 0.3) & (np.abs(Y) < 0.9)
    assert (g[side] == pm.FREE).mean() > 0.95, (g[side] == pm.FREE).mean()   # between the path and the walls
    assert (g[np.abs(Y) > 1.2] == pm.UNK).all()                              # behind the walls no beam ever went
    known = g0 != pm.UNK
    assert (g[known] == g0[known]).all()                                     # occupied and free cells are never touched


def test_a_cell_the_beams_keep_hitting_is_not_freed():
    g0, _, _ = corridor_grid()
    scans, traj = drive(planes(pillar=True))
    g = mv.fill_unknown(g0, RES, ORIGIN, T, scans, traj, extr=(0, 0, 0))
    assert g[cell_at(5.02, 0.5)] == pm.UNK       # on the pillar: hit from both sides
    assert g[cell_at(3.0, 0.5)] == pm.FREE       # the same distance from the path, nothing there


def test_a_few_stray_hits_are_tolerated_but_a_real_share_is_not():
    passed = np.array([[100, 100, 100], [100, 100, 100]])
    hit = np.array([[1, 0, 0], [4, 4, 0]])       # 1 of 301 observations vs 8 of 308
    assert mv.free_cells(hit, passed).tolist() == [True, False]


def test_pass_evidence_is_only_required_at_the_heights_that_were_observed():
    hit = np.zeros((3, 3), int)
    passed = np.array([[5, 5, 0],                # two heights seen, the third never: free
                       [5, 0, 0],                # only one height seen: not enough
                       [5, 5, 2]])               # all seen but one with 2 passes: not free
    assert mv.free_cells(hit, passed).tolist() == [True, False, False]


def test_only_cells_connected_to_known_free_space_are_kept():
    g = np.full((10, 10), pm.UNK, np.uint8)
    g[0, 0] = pm.FREE
    m = np.zeros((10, 10), bool)
    m[0, 1:4] = True                             # touches the free cell
    m[7:9, 7:9] = True                           # an isolated pocket
    keep = mv.connected_to_free(m, g)
    assert keep[0, 1:4].all() and not keep[7:9, 7:9].any()


def test_pose_at_interpolates_between_poses_and_refuses_gaps():
    q90 = Rotation.from_euler("z", 90, degrees=True).as_quat()
    poses = np.array([[0, 0, 0, 0, 0, 0, 0, 1], [0.4, 2, 0, 0, *q90], [5, 2, 0, 0, *q90]], float)   # t x y z qx qy qz qw
    R, p = mv.pose_at(poses, 0.2)
    assert np.allclose(p, [1, 0, 0]) and np.allclose(R, Rotation.from_euler("z", 45, degrees=True).as_matrix())
    assert mv.pose_at(poses, 3.0) is None        # 4 s between two poses
    assert mv.pose_at(poses, -1.0) is None and mv.pose_at(poses, 9.0) is None


PHANTOM = np.random.default_rng(3).uniform([4.85, 0.35, 0.8], [5.15, 0.65, 1.1], (300, 3))   # map frame: 30 x 30 cm in the middle of the corridor where nothing is


def world_cloud(scans, extra=None):
    """The SLAM cloud in the SLAM world: every scan point, plus points given in the map frame that no beam ever hit. Also returns the count of real points and Q (the map-frame cloud)."""
    Rm, tm = T[:3, :3], T[:3, 3]
    P = np.vstack([pts @ Rm + t for pts, _, t in scans])
    n = len(P)
    if extra is not None:
        P = np.vstack([P, (extra - tm) @ Rm])
    return P, n, P @ Rm.T + tm


def test_the_ghost_rule_needs_enough_observations_and_a_hit_ratio_under_the_threshold():
    hit, passed = np.array([0, 1, 5, 0]), np.array([20, 9, 5, 5])      # observations 20, 10, 10, 5; ratios 0, 0.1, 0.5, 0
    assert mv.ghost_voxels(hit, passed, 0.10).tolist() == [True, False, False, False]


def test_points_the_beams_pass_through_are_flagged_and_the_walls_are_not():
    scans, _ = drive(planes())
    P, n, Q = world_cloud(scans, PHANTOM)
    ghost = mv.ghost_points(P, Q, scans, extr=(0, 0, 0))
    assert ghost[n:].all()                              # the phantom: every beam went through it
    assert ghost[:n].mean() < 0.02, ghost[:n].mean()    # the real surfaces: the beams end there


def test_voxels_seen_in_few_scans_are_not_flagged():
    scans, _ = drive(planes())
    P, n, Q = world_cloud(scans, PHANTOM)
    assert not mv.ghost_points(P, Q, scans[:3], extr=(0, 0, 0))[n:].any()   # 3 scans = at most 3 observations of a voxel


def test_build_map_drops_the_phantom_and_frees_the_unknown_without_touching_the_rest():
    scans, traj = drive(planes())
    P, n, Q = world_cloud(scans, PHANTOM)
    args = (P, Q, T, lambda: scans, traj)
    raw, _ = mv.build_map(*args, min_ratio=0, fill=False, extr=(0, 0, 0))
    clean, _ = mv.build_map(*args, fill=False, extr=(0, 0, 0))
    filled, _ = mv.build_map(*args, extr=(0, 0, 0))
    assert (raw == pm.OCC).sum() - (clean == pm.OCC).sum() >= 20       # the 30 x 30 cm phantom is about 36 cells
    assert (filled == pm.UNK).sum() < (clean == pm.UNK).sum()
    assert ((filled == pm.OCC) == (clean == pm.OCC)).all()


if __name__ == "__main__":
    test_unknown_floor_the_beams_crossed_is_freed_and_the_outside_stays_unknown()
    test_a_cell_the_beams_keep_hitting_is_not_freed()
    test_a_few_stray_hits_are_tolerated_but_a_real_share_is_not()
    test_pass_evidence_is_only_required_at_the_heights_that_were_observed()
    test_only_cells_connected_to_known_free_space_are_kept()
    test_pose_at_interpolates_between_poses_and_refuses_gaps()
    test_the_ghost_rule_needs_enough_observations_and_a_hit_ratio_under_the_threshold()
    test_points_the_beams_pass_through_are_flagged_and_the_walls_are_not()
    test_voxels_seen_in_few_scans_are_not_flagged()
    test_build_map_drops_the_phantom_and_frees_the_unknown_without_touching_the_rest()
    print("ok")
