"""Run: python3 -B test_scenarios.py   (offline: numpy and scipy)"""
import math

import numpy as np

import initpose
import scenarios as sc

RES = 0.05


def room(h=100, w=200, wall=3):
    """Free room with a border wall; returns the occupancy grid (row 0 = top)."""
    occ = np.zeros((h, w), bool)
    occ[:wall, :] = occ[-wall:, :] = occ[:, :wall] = occ[:, -wall:] = True
    return occ


def test_load_map_classifies_like_map_server(tmp_path_factory=None):
    import os
    import tempfile
    from PIL import Image
    with tempfile.TemporaryDirectory() as d:
        img = np.array([[254, 205, 0], [254, 254, 205]], np.uint8)
        Image.fromarray(img).save(os.path.join(d, "m.pgm"))
        open(os.path.join(d, "m.yaml"), "w").write("image: m.pgm\nresolution: 0.05\norigin: [1.0, 2.0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
        mp = sc.load_map(os.path.join(d, "m.yaml"))
    assert mp["res"] == 0.05 and mp["origin"] == (1.0, 2.0) and mp["occ"].shape == (2, 3)
    assert mp["occ"].tolist() == [[False, False, True], [False, False, False]]
    assert mp["blocked"].tolist() == [[False, True, True], [False, False, True]]    # unknown counts as blocked for planning


def test_init_offset_is_deterministic_rotates_direction_and_alternates_yaw():
    offs = [initpose.init_offset(k, 0.5, 15.0) for k in range(6)]
    assert offs == [initpose.init_offset(k, 0.5, 15.0) for k in range(6)]
    for dx, dy, dyaw in offs:
        assert abs(math.hypot(dx, dy) - 0.5) < 1e-9 and abs(abs(dyaw) - math.radians(15.0)) < 1e-9
    assert offs[0][2] > 0 > offs[1][2]                                              # yaw alternates in sign
    angles = [math.degrees(math.atan2(dy, dx)) % 360 for dx, dy, _ in offs]
    assert [round(x) for x in angles] == [0, 60, 120, 180, 240, 300]               # the direction turns 60 deg per trial
    assert initpose.init_offset(3, 0.0, 0.0) == (0.0, 0.0, 0.0) or all(abs(v) < 1e-12 for v in initpose.init_offset(3, 0.0, 0.0))


def test_to_cell_and_back():
    r, c = sc.to_cell(1.0, 2.0, (0.0, 0.0), RES, 100)
    assert (r, c) == (100 - 1 - 40, 20)
    x, y = sc.to_xy(r, c, (0.0, 0.0), RES, 100)
    assert abs(x - 1.025) < 1e-9 and abs(y - 2.025) < 1e-9


def test_eroded_free_keeps_the_clearance():
    occ = room()
    free = sc.eroded_free(occ, 0.45, RES)
    assert not free[10, 10]                      # next to the wall
    assert free[50, 100]
    d = sc.distance_to_occupied_cells(occ, RES)
    assert np.all(d[free] >= 0.45 - 1e-9)


def test_geodesic_length_in_an_empty_room_is_the_straight_line():
    occ = room()
    free = sc.eroded_free(occ, 0.3, RES)
    a, b = (50, 40), (50, 160)
    length = sc.shortest_length(free, a, b, RES)
    assert abs(length - 120 * RES) < 0.02


def test_geodesic_length_with_a_diagonal_is_not_octile_inflated():
    occ = room()
    free = sc.eroded_free(occ, 0.3, RES)
    a, b = (30, 30), (70, 150)
    straight = math.hypot(40, 120) * RES
    assert abs(sc.shortest_length(free, a, b, RES) - straight) < 0.03 * straight   # grid paths alone would be ~6 % longer


def test_geodesic_length_goes_around_a_wall():
    occ = room()
    occ[10:80, 100] = True                        # wall with a gap at the bottom rows 80..96
    free = sc.eroded_free(occ, 0.2, RES)
    a, b = (50, 60), (50, 140)
    length = sc.shortest_length(free, a, b, RES)
    assert length > 80 * RES * 1.2                # much longer than the straight 4 m
    assert math.isfinite(length)


def test_unreachable_is_infinite():
    occ = room()
    occ[:, 100] = True
    free = sc.eroded_free(occ, 0.2, RES)
    assert sc.shortest_length(free, (50, 40), (50, 160), RES) == float("inf")


def test_sample_pairs_are_reproducible_reachable_and_in_range():
    occ = room()
    kw = dict(n=12, seed=5, clearance=0.45, dmin=2.0, dmax=7.0, res=RES, origin=(0.0, 0.0))
    p1, p2 = sc.sample_pairs(occ, **kw), sc.sample_pairs(occ, **kw)
    assert p1 == p2 and len(p1) == 12
    free = sc.eroded_free(occ, 0.45, RES)
    for t in p1:
        sx, sy, _ = t["start"]
        gx, gy, _ = t["goal"]
        d = math.hypot(gx - sx, gy - sy)
        assert 2.0 <= d <= 7.0
        assert t["shortest"] >= d - 1e-3 and math.isfinite(t["shortest"])   # both are rounded to 1 mm
        for x, y in ((sx, sy), (gx, gy)):
            r, c = sc.to_cell(x, y, (0.0, 0.0), RES, 100)
            assert free[r, c]
    assert sc.sample_pairs(occ, **{**kw, "seed": 6}) != p1


def test_goal_band_keeps_goals_close_to_the_obstacles():
    occ = room()
    kw = dict(n=12, seed=5, clearance=0.45, dmin=2.0, dmax=7.0, res=RES, origin=(0.0, 0.0))
    tight = sc.sample_pairs(occ, goal_band=0.65, **kw)
    d = sc.distance_to_occupied_cells(occ, RES)
    for t in tight:
        r, c = sc.to_cell(t["goal"][0], t["goal"][1], (0.0, 0.0), RES, 100)
        assert 0.45 <= d[r, c] < 0.65                       # inside the band
    loose = sc.sample_pairs(occ, **kw)
    assert max(d[sc.to_cell(t["goal"][0], t["goal"][1], (0.0, 0.0), RES, 100)] for t in loose) >= 0.65   # without the band, goals reach the room centre


def test_sample_pairs_stay_in_the_main_component():
    occ = room()
    occ[:, 100] = True                            # splits the room in two halves
    pairs = sc.sample_pairs(occ, n=10, seed=1, clearance=0.4, dmin=1.0, dmax=6.0, res=RES, origin=(0.0, 0.0))
    xs = [t["start"][0] for t in pairs] + [t["goal"][0] for t in pairs]
    assert all(x < 100 * RES for x in xs) or all(x > 100 * RES for x in xs)   # both ends in the same half


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
