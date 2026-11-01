"""Run: python3 -B test_plan_metrics.py   (offline: numpy and scipy)"""
import math

import numpy as np

import metrics as m
import plan_metrics as p

RES = 0.05


def ctx(occ=None):
    occ = np.zeros((300, 400), bool) if occ is None else occ          # 15 m x 20 m
    return {"dist": m.distance_to_occupied(occ, RES), "origin": (0.0, 0.0), "res": RES, "footprint": (0.70, 0.58)}


def line(a, b, step=0.1):
    n = max(2, int(math.dist(a, b) / step) + 1)
    return [list(q) for q in zip(np.linspace(a[0], b[0], n), np.linspace(a[1], b[1], n))]


def test_straight_path():
    r = p.path_metrics(line((1, 2), (11, 2)), ctx())
    assert abs(r["length"] - 10.0) < 1e-9 and r["turning"] < 1e-6 and r["max_curvature"] < 1e-6 and r["sharp_turns"] == 0 and r["cusps"] == 0
    assert r["clearance_min"] == 5.0                                               # open map: capped at 5 m like analyze.py


def test_straight_path_beside_a_wall_has_the_footprint_clearance():
    occ = np.zeros((300, 400), bool)
    occ[int((15 - 3.5) / RES), :] = True                                         # wall along y = 3.5
    r = p.path_metrics(line((1, 2), (11, 2)), ctx(occ))
    assert abs(r["clearance_min"] - (3.5 - 2.0 - 0.29)) < 0.06                    # half the footprint width is 0.29 m


def test_an_L_has_one_sharp_turn_of_ninety_degrees():
    path = line((1, 2), (6, 2)) + line((6, 2), (6, 7))[1:]
    r = p.path_metrics(path, ctx())
    assert abs(r["length"] - 10.0) < 1e-9
    assert abs(r["turning"] - math.pi / 2) < 0.05 and r["sharp_turns"] == 1 and r["max_curvature"] > 3.0


def test_turns_in_opposite_directions_do_not_cancel():
    path = line((1, 2), (4, 2)) + line((4, 2), (4, 5))[1:] + line((4, 5), (7, 5))[1:]        # left turn then right turn
    r = p.path_metrics(path, ctx())
    assert abs(r["turning"] - math.pi) < 0.1 and r["sharp_turns"] == 2


def test_the_footprint_is_oriented_along_the_path_heading():
    occ = np.zeros((300, 400), bool)
    occ[int((15 - 8.0) / RES), :] = True                                              # wall across y = 8.0
    c = ctx(occ)
    c["footprint"] = (1.2, 0.2)                                                       # long along the heading, narrow across
    r = p.path_metrics(line((5, 2), (5, 6.5)), c)                                     # driving north towards the wall, ends 1.5 m before it
    assert abs(r["clearance_min"] - (8.0 - 6.5 - 0.6)) < 0.06                         # the long half (0.6 m) points at the wall; a footprint fixed along x would give 1.4


def test_a_semicircle_is_smooth():
    th = np.linspace(-math.pi / 2, math.pi / 2, 200)
    path = [[6 + 2 * math.cos(t), 6 + 2 * math.sin(t)] for t in th]
    r = p.path_metrics(path, ctx())
    assert abs(r["turning"] - math.pi) < 0.05 and r["sharp_turns"] == 0
    assert abs(r["max_curvature"] - 0.5) < 0.1                                    # 1 / radius


def test_a_path_through_an_obstacle_has_nonpositive_clearance():
    occ = np.zeros((300, 400), bool)
    occ[int((15 - 2.0) / RES):int((15 - 1.9) / RES), 120:140] = True              # a block at x 6..7, y 1.9..2.0 (cell rows are flipped)
    r = p.path_metrics(line((1, 2), (11, 2)), ctx(occ))
    assert r["clearance_min"] <= 0.0


def test_a_cusp_is_a_reversal_of_direction():
    path = line((1, 2), (5, 2)) + line((5, 2), (3, 2))[1:]                        # goes forward then back along itself
    r = p.path_metrics(path, ctx())
    assert r["cusps"] == 1


def test_degenerate_paths():
    assert p.path_metrics([], ctx())["length"] == 0.0
    assert p.path_metrics([[1.0, 1.0]], ctx())["length"] == 0.0


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
