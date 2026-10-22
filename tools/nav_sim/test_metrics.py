"""Run: python3 -B test_metrics.py   (offline analysis: needs numpy and scipy)"""
import math

import numpy as np

import metrics as m


def close(a, b, tol=1e-6):
    return abs(a - b) <= tol


def test_path_length():
    assert close(m.path_length(np.array([[0, 0], [3, 4], [3, 10]])), 11.0)
    assert m.path_length(np.array([[1.0, 2.0]])) == 0.0


def test_spl():
    assert close(m.spl(True, 10.0, 10.0), 1.0)
    assert close(m.spl(True, 10.0, 20.0), 0.5)
    assert m.spl(False, 10.0, 10.0) == 0.0
    assert close(m.spl(True, 10.0, 8.0), 1.0)    # a path shorter than the reference does not score above 1


def test_resample_and_derivatives():
    t = np.linspace(0, 4, 41)
    v = 0.5 * t                     # constant acceleration 0.5, zero jerk
    tt, vv = m.resample(t, v, 0.05)
    acc, jerk = m.acc_jerk(tt, vv)
    assert close(float(np.median(acc)), 0.5, 1e-9) and float(np.max(np.abs(jerk))) < 1e-6


def test_jerk_of_a_sine():
    t = np.linspace(0, 10, 2001)
    v = np.sin(t)                   # a = cos t, j = -sin t: RMS of j is 1/sqrt(2) over whole periods
    tt, vv = m.resample(t, v, 0.01)
    _, jerk = m.acc_jerk(tt, vv)
    inner = jerk[5:-5]
    assert abs(float(np.sqrt(np.mean(inner ** 2))) - 1 / math.sqrt(2)) < 0.02


def test_resample_irregular_times():
    t = np.array([0.0, 0.1, 0.35, 1.0])
    v = np.array([0.0, 1.0, 1.0, 0.0])
    tt, vv = m.resample(t, v, 0.05)
    assert close(tt[0], 0.0) and close(tt[-1], 1.0) and close(float(vv[2]), 1.0) and close(float(vv[-1]), 0.0)


def test_sign_reversals_ignore_the_deadband():
    v = np.array([0.2, 0.2, 0.005, -0.005, 0.2, -0.2, -0.2, 0.2])
    assert m.sign_reversals(v, deadband=0.01) == 2     # + to - and - to +; the dither inside the deadband is not a reversal
    assert m.sign_reversals(np.zeros(5), deadband=0.01) == 0


def test_cross_track():
    path = np.array([[0.0, 0.0], [10.0, 0.0]])
    traj = np.array([[1.0, 0.3], [5.0, -0.1], [9.0, 0.0], [12.0, 0.4]])   # last one is beyond the end: distance to the end point
    d = m.cross_track(path, traj)
    assert np.allclose(d, [0.3, 0.1, 0.0, math.hypot(2.0, 0.4)])


def test_cross_track_handles_a_corner():
    path = np.array([[0.0, 0.0], [5.0, 0.0], [5.0, 5.0]])
    d = m.cross_track(path, np.array([[4.0, 1.0], [6.0, 2.0]]))
    assert np.allclose(d, [1.0, 1.0])


def grid_with_wall():
    g = np.zeros((40, 60), bool)    # 0.05 m cells, row 0 = top; wall = column 40
    g[:, 40] = True
    return g


def test_clearance_map_and_footprint_clearance():
    occ = grid_with_wall()
    dist = m.distance_to_occupied(occ, 0.05)
    # robot 0.70 x 0.58 pointing along x, front edge 0.5 m before the wall centre line
    x = 40 * 0.05 + 0.025 - 0.5 - 0.35        # wall cell centre minus 0.5 minus half length
    y = 20 * 0.05
    c = m.footprint_clearance(dist, (0.0, 0.0), 0.05, x, y, 0.0, 0.70, 0.58)
    assert abs(c - 0.5) < 0.06                  # one cell of discretisation
    # turning 90 deg: the long side is now across x, the wide side along y: front edge is half the width from the centre
    c90 = m.footprint_clearance(dist, (0.0, 0.0), 0.05, x + 0.35 - 0.29, y, math.pi / 2, 0.70, 0.58)
    assert abs(c90 - 0.5) < 0.06


def test_footprint_clearance_is_nonpositive_inside_a_wall():
    occ = grid_with_wall()
    dist = m.distance_to_occupied(occ, 0.05)
    c = m.footprint_clearance(dist, (0.0, 0.0), 0.05, 40 * 0.05, 1.0, 0.0, 0.70, 0.58)
    assert c <= 0.0
    assert m.collides(dist, (0.0, 0.0), 0.05, 40 * 0.05, 1.0, 0.0, 0.70, 0.58)
    assert not m.collides(dist, (0.0, 0.0), 0.05, 1.0, 1.0, 0.0, 0.70, 0.58)


def test_footprint_clearance_uses_the_right_image_row():
    occ = np.zeros((40, 60), bool)
    occ[0:5, 20:25] = True          # block at the TOP of the image: y between 1.75 and 2.0 m
    dist = m.distance_to_occupied(occ, 0.05)
    assert m.collides(dist, (0.0, 0.0), 0.05, 1.12, 1.9, 0.0, 0.70, 0.58)
    assert not m.collides(dist, (0.0, 0.0), 0.05, 1.12, 0.3, 0.0, 0.70, 0.58)   # same x, far from the block


def test_footprint_clearance_outside_the_map_is_infinite_not_a_crash():
    dist = m.distance_to_occupied(grid_with_wall(), 0.05)
    assert m.footprint_clearance(dist, (0.0, 0.0), 0.05, -5.0, -5.0, 0.0, 0.70, 0.58) == float("inf")


def test_rms_p95():
    x = np.array([3.0, -4.0])
    assert close(m.rms(x), math.sqrt(12.5))
    assert close(m.p95(np.arange(101.0)), 95.0)
    assert math.isnan(m.rms(np.array([])))


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
