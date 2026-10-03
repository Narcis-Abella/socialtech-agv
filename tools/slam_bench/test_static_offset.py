"""Checks static_offset.py on synthetic data with a known IMU-vs-floor offset. Run: python3 test_static_offset.py"""
import numpy as np

import static_offset

RNG = np.random.default_rng(3)
OFFSET = np.array([-0.0075, 0.0049, 0.0])  # true up minus IMU up, in the sensor frame


def unit(v):
    return np.asarray(v) / np.linalg.norm(v)


def scene(u, floor_h=0.6, ceil_h=1.9, floor=True):
    """Points in the sensor frame: a floor below and a ceiling above, both perpendicular to the TRUE up (u + OFFSET), plus walls and clutter."""
    up = unit(u + OFFSET)
    a = unit(np.cross(up, [1, 0, 0]))
    b = np.cross(up, a)
    pts = []
    for h, n in ((-floor_h, 6000 if floor else 0), (ceil_h, 6000)):
        c = RNG.uniform(-8, 8, (n, 2))
        pts.append(np.outer(c[:, 0], a) + np.outer(c[:, 1], b) + h * up + RNG.normal(0, 0.01, (n, 1)) * up)
    wall = RNG.uniform(-8, 8, (2500, 3))
    wall[:, 0] = 8.0
    pts.append(wall)
    pts.append(RNG.uniform(-8, 8, (600, 3)))
    return np.vstack(pts), up


def test_recovers_floor_and_ceiling_normals():
    u = unit([0.0124, -0.0018, 0.9999])
    P, up = scene(u)
    res = static_offset.level_planes(P, u, np.random.default_rng(0))
    for kind in ("floor", "ceiling"):
        assert res[kind] is not None, kind
        assert np.linalg.norm(res[kind]["n"] - up) < 0.0015, (kind, res[kind]["n"], up)  # < 0.09 deg
    assert abs(res["floor"]["h"] - 0.6) < 0.03 and abs(res["ceiling"]["h"] - 1.9) < 0.03


def test_no_floor_gives_none_not_a_wrong_plane():
    u = unit([0.0, 0.0, 1.0])
    P, _ = scene(u, floor=False)
    res = static_offset.level_planes(P, u, np.random.default_rng(0))
    assert res["floor"] is None and res["ceiling"] is not None


def test_static_windows_skip_motion():
    rate, n = 200, 200 * 30
    t = np.arange(n) / rate
    acc = np.tile([0.0, 0.0, 1.0], (n, 1)) + RNG.normal(0, 0.001, (n, 3))
    gyr = RNG.normal(0, 0.001, (n, 3))
    acc[1000:3000] += RNG.normal(0, 0.05, (2000, 3))  # moving from 5 s to 15 s
    wins = static_offset.static_windows(t, acc, gyr, win=2.0, min_gap=4.0, max_n=50)
    starts = [t[i] for i, _ in wins]
    assert wins and all(not (3.0 < s < 15.0) for s in starts), starts  # a 2 s window must not overlap the moving segment
    assert all(b - a > 0 for a, b in wins)


def test_summary_and_leave_one_out():
    bags = {name: [OFFSET[:2] + RNG.normal(0, 0.001, 2) for _ in range(8)] for name in ("a", "b", "c")}
    bags["d"] = [np.array([0.004, -0.006]) + RNG.normal(0, 0.001, 2) for _ in range(8)]  # an odd bag
    s = static_offset.summary(bags)
    for name in "abc":
        assert s[name]["loo_residual_deg"] < 0.15, s[name]  # predicted from the others, which include one odd bag
    assert s["d"]["loo_residual_deg"] > 0.5, s["d"]


def test_correction_quaternion_maps_imu_up_onto_true_up():
    from scipy.spatial.transform import Rotation as R
    off = np.array([-0.0080, 0.0053])
    q = static_offset.correction_quaternion(off)
    u = unit([0.0124, -0.0018, 0.9999])
    out = R.from_quat(q).apply(u)
    assert np.allclose(out, u + np.array([off[0], off[1], 0.0]), atol=2e-4), (out, u + off)  # the rotation moves IMU-up to floor-up


if __name__ == "__main__":
    test_recovers_floor_and_ceiling_normals()
    test_no_floor_gives_none_not_a_wrong_plane()
    test_static_windows_skip_motion()
    test_summary_and_leave_one_out()
    test_correction_quaternion_maps_imu_up_onto_true_up()
    print("ok")
