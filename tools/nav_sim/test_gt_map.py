"""Run: python3 -B test_gt_map.py   (offline: numpy, scipy, PIL, yaml)"""
import math
import os
import tempfile

import numpy as np

import gt_map as g
import scenarios as sc

RES = 0.05


def room():
    occ = np.zeros((120, 160), bool)                     # 6 m x 8 m
    occ[:2, :] = occ[-2:, :] = occ[:, :2] = occ[:, -2:] = True
    occ[40:60, 70:74] = True                             # a pillar
    return occ


def fake_scan(occ, x, y, n=720, rmax=12.0, noise=0.0, rng=None, yaw=0.0):
    """Ranges of a 360 deg lidar at (x, y) against `occ` (origin 0,0), by marching each ray."""
    h = occ.shape[0]
    ang = np.linspace(-math.pi, math.pi, n, endpoint=False)
    ranges = np.full(n, np.inf)
    for i, a in enumerate(ang):
        for r in np.arange(0.1, rmax, RES / 3):
            c, rr = int((x + r * math.cos(a + yaw)) / RES), h - 1 - int((y + r * math.sin(a + yaw)) / RES)
            if not (0 <= rr < h and 0 <= c < occ.shape[1]) or occ[rr, c]:
                ranges[i] = r
                break
    if noise:
        ranges = ranges + rng.normal(0, noise, n)
    return {"pose": [x, y, yaw], "angle_min": -math.pi, "angle_increment": 2 * math.pi / n, "range_min": 0.45, "range_max": rmax, "ranges": ranges.tolist()}


def test_integration_reproduces_a_known_room():
    occ = room()
    scans = [fake_scan(occ, x, y) for x, y in ((1.0, 1.0), (3.0, 1.0), (5.0, 1.0), (1.0, 3.0), (5.0, 3.0), (1.0, 5.0), (3.0, 5.0), (5.0, 5.0), (7.0, 5.0), (7.0, 1.0), (7.0, 3.0))]
    est = g.integrate(scans, shape=occ.shape, res=RES, origin=(0.0, 0.0))
    surface = occ & ndilate(~occ)                         # the lidar can only see the face of a wall that borders free space
    agree = (est["occupied"] & surface).sum() / surface.sum()
    assert agree > 0.95
    assert (est["occupied"] & ~ndilate(occ)).sum() == 0   # nothing is marked occupied away from a true obstacle
    walls = occ
    inner = np.zeros_like(occ)
    inner[10:-10, 10:-10] = True
    inner &= ~ndilate(walls, 4)
    assert (est["free"] & inner).sum() / inner.sum() > 0.9
    assert not (est["free"] & est["occupied"]).any()


def ndilate(a, n=1):
    from scipy import ndimage
    return ndimage.binary_dilation(a, iterations=n)


def test_a_cell_never_hit_is_not_occupied_and_unseen_is_unknown():
    occ = np.zeros((40, 160), bool)                      # 2 m x 8 m
    occ[:, 0] = True
    s = fake_scan(occ, 1.0, 1.0, rmax=3.0)
    est = g.integrate([s], shape=occ.shape, res=RES, origin=(0.0, 0.0), clear_unreturned=True)
    assert not est["free"][5, 150] and not est["occupied"][5, 150]          # x = 7.5 m, beyond the 3 m range of the only pose: unknown
    assert est["free"][38, 40]                                              # x = 2.0 m, y = 0.1 m: inside the range, free


def test_noise_does_not_create_obstacles_in_free_space():
    occ = room()
    rng = np.random.default_rng(0)
    scans = [fake_scan(occ, x, y, n=360, noise=0.01, rng=rng) for x, y in ((1.0, 1.0), (3.0, 1.0), (5.0, 1.0), (1.0, 3.0), (5.0, 3.0), (1.0, 5.0), (3.0, 5.0), (5.0, 5.0), (7.0, 3.0))]
    est = g.integrate(scans, shape=occ.shape, res=RES, origin=(0.0, 0.0))
    stray = est["occupied"] & ~ndilate(occ, 3)
    assert stray.sum() <= 0.03 * occ.sum()


def test_write_map_roundtrip_with_load_map():
    occ = room()
    scans = [fake_scan(occ, 3.0, 3.0)]
    est = g.integrate(scans, shape=occ.shape, res=RES, origin=(0.0, 0.0))
    with tempfile.TemporaryDirectory() as d:
        g.write_map(est, os.path.join(d, "gt"), RES, (1.5, -2.5))
        mp = sc.load_map(os.path.join(d, "gt.yaml"))
    assert mp["origin"] == (1.5, -2.5) and mp["res"] == RES
    assert (mp["occ"] == est["occupied"]).all()
    assert (~mp["blocked"] == est["free"]).all()


def test_lattice_poses_are_spaced_free_and_cover_the_region():
    blocked = np.zeros((100, 200), bool)
    blocked[:4, :] = blocked[-4:, :] = blocked[:, :4] = blocked[:, -4:] = True
    pts = sc.lattice_poses(blocked, clearance=0.5, spacing=1.0, res=RES, origin=(0.0, 0.0))
    assert len(pts) > 20
    xs = sorted({round(x, 3) for x, _ in pts})
    assert all(abs((b - a) - 1.0) < 1e-6 for a, b in zip(xs, xs[1:]))
    ys = sorted({round(y, 3) for _, y in pts})
    assert all(abs((b - a) - 1.0) < 1e-6 for a, b in zip(ys, ys[1:]))
    from scipy import ndimage
    d = ndimage.distance_transform_edt(~blocked) * RES
    for x, y in pts:
        r, c = sc.to_cell(x, y, (0.0, 0.0), RES, 100)
        assert d[r, c] >= 0.5 - 1e-9
    assert sc.lattice_poses(np.ones((10, 10), bool), 0.1, 1.0, RES, (0.0, 0.0)) == []


def test_the_scanner_yaw_is_applied():
    occ = room()
    surface = occ & ndilate(~occ)
    poses = ((1.0, 1.0), (3.0, 1.0), (5.0, 1.0), (1.0, 3.0), (5.0, 3.0), (1.0, 5.0), (3.0, 5.0), (5.0, 5.0), (7.0, 5.0), (7.0, 1.0), (7.0, 3.0))
    est = g.integrate([fake_scan(occ, x, y, yaw=1.0 + 0.3 * i) for i, (x, y) in enumerate(poses)], shape=occ.shape, res=RES, origin=(0.0, 0.0))
    assert (est["occupied"] & surface).sum() / surface.sum() > 0.95 and (est["occupied"] & ~ndilate(occ)).sum() == 0


def beams(x, y, angle, rng_m, n=3, rmax=12.0):
    return {"pose": [x, y, 0.0], "angle_min": angle, "angle_increment": 0.0, "range_min": 0.45, "range_max": rmax, "ranges": [rng_m] * n}


def test_a_single_spurious_return_is_not_an_obstacle():
    shape = (60, 160)
    scans = [beams(1.0, 1.0, 0.0, 3.0, n=1), beams(1.0, 1.0, 0.0, float("inf"), n=1)]      # one phantom hit at 3 m, one ray that sees nothing
    est = g.integrate(scans, shape=shape, res=RES, origin=(0.0, 0.0))
    assert not est["occupied"].any()


def test_a_cell_is_never_both_free_and_occupied():
    shape = (60, 160)
    scans = [beams(1.0, 1.0, 0.0, 3.0, n=3), beams(5.0, 1.0, math.pi, float("inf"), n=3)]  # three hits on (4, 1) from the west, three rays through it from the east
    est = g.integrate(scans, shape=shape, res=RES, origin=(0.0, 0.0), clear_unreturned=True)
    r, c = sc.to_cell(4.0, 1.0, (0.0, 0.0), RES, 60)
    assert est["occupied"][r, c] and not est["free"][r, c]
    assert not (est["free"] & est["occupied"]).any()


def test_the_ray_does_not_clear_the_cell_it_hits():
    est = g.integrate([beams(1.0, 1.0, 0.0, 3.04, n=2)], shape=(60, 160), res=RES, origin=(0.0, 0.0))   # the hit lands inside cell 4.00-4.05, not on its edge
    r, c = sc.to_cell(4.04, 1.0, (0.0, 0.0), RES, 60)
    assert est["hits"][r, c] == 2 and est["passes"][r, c] == 0


def test_rays_without_a_return_say_nothing_by_default():
    shape = (60, 160)
    scans = [beams(1.0, 1.0, 0.0, float("inf"), n=5), beams(1.0, 1.0, 0.3, 50.0, n=5)]            # nothing seen within range, and a "return" beyond range_max
    est = g.integrate(scans, shape=shape, res=RES, origin=(0.0, 0.0))
    assert not est["free"].any() and not est["occupied"].any()                                    # open space seen through a gap is not floor we know
    on = g.integrate(scans, shape=shape, res=RES, origin=(0.0, 0.0), clear_unreturned=True)
    assert on["free"].sum() > 100 and not on["occupied"].any()


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
