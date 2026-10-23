"""Run: python3 -B test_synth_scan.py"""
import math

import numpy as np

import synth_scan as ss


def room(n=200, res=0.05):
    """10 m x 10 m room (walls one cell thick) with a 1 m square pillar at x 6..7, y 4..5; origin (0, 0)."""
    occ = np.zeros((n, n), bool)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    occ[int(4 / res):int(5 / res), int(6 / res):int(7 / res)] = True       # rows are y (bottom row = y 0, flipped by the caller)
    return occ[::-1], res, (0.0, 0.0)                                       # image rows top to bottom


def test_ranges_to_walls_and_pillar():
    occ, res, org = room()
    r = ss.scan(occ, res, org, (2.0, 4.5, 0.0), n=720, angle_min=-math.pi, inc=2 * math.pi / 720, rmax=20.0)
    ang = lambda a: int(round((a + math.pi) / (2 * math.pi / 720)))
    assert abs(r[ang(0.0)] - 4.0) < 0.1                          # pillar face at x = 6
    assert abs(r[ang(math.pi / 2)] - 5.5) < 0.1                  # top wall at y = 10 (inner face 9.95), from y 4.5
    assert abs(r[ang(-math.pi / 2)] - 4.5) < 0.1                 # bottom wall
    assert abs(r[0] - 2.0) < 0.1                                 # angle -pi: left wall at x = 0 (inner face 0.05)


def test_beams_that_leave_the_map_or_exceed_range_are_inf():
    occ, res, org = room()
    r = ss.scan(occ, res, org, (2.0, 4.5, 0.0), n=720, angle_min=-math.pi, inc=2 * math.pi / 720, rmax=3.0)
    assert np.isinf(r[int(round((0.0 + math.pi) / (2 * math.pi / 720)))])     # pillar at 4 m > rmax 3
    assert np.isfinite(r).any()


def test_yaw_rotates_the_scan():
    occ, res, org = room()
    a = ss.scan(occ, res, org, (2.0, 4.5, 0.0), 720, -math.pi, 2 * math.pi / 720, 20.0)
    b = ss.scan(occ, res, org, (2.0, 4.5, math.pi / 2), 720, -math.pi, 2 * math.pi / 720, 20.0)
    k = 720 // 4
    assert np.allclose(np.roll(a, -k), b, atol=0.12, equal_nan=True)


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
