"""Run: python3 -B test_replay_rviz.py"""
import math

import numpy as np

import replay_rviz as rr


def grid_with(cells, shape=(40, 40)):
    occ = np.zeros(shape, bool)
    for r, c in cells:
        occ[r, c] = True
    return occ


def test_face_clearance_is_the_gap_to_the_wall_face_not_to_its_cell_centre():
    # res 0.05, origin (0, 0), 40x40 cells: cell (row 20, col 30) spans x 1.50-1.55, y 0.95-1.00 -> row 20 = h-1-19 -> y cell index 19
    occ = grid_with([(20, 30)])
    # footprint 0.70 x 0.58 centred at (1.0, 0.975), yaw 0: its +x face is at x = 1.35; wall face at x = 1.50 -> gap 0.15
    gap, cell, _ = rr.face_clearance(occ, (0.0, 0.0), 0.05, 1.0, 0.975, 0.0, 0.70, 0.58)
    assert abs(gap - 0.15) < 0.011, gap          # footprint sampled every 1 cm
    assert cell == (20, 30)


def test_face_clearance_with_rotation_and_corner_distance():
    occ = grid_with([(20, 30)])
    # yaw 90 deg: footprint spans x +-0.29, y +-0.35 around (1.0, 0.975): +x face at 1.29 -> gap 0.21
    gap, _, _ = rr.face_clearance(occ, (0.0, 0.0), 0.05, 1.0, 0.975, math.pi / 2, 0.70, 0.58)
    assert abs(gap - 0.21) < 0.011, gap


def test_face_clearance_is_zero_when_the_footprint_covers_an_occupied_cell():
    occ = grid_with([(20, 20)])
    gap, _, _ = rr.face_clearance(occ, (0.0, 0.0), 0.05, 1.025, 0.975, 0.0, 0.70, 0.58)
    assert gap == 0.0


def test_face_clearance_without_obstacles_is_inf():
    gap, cell, _ = rr.face_clearance(np.zeros((10, 10), bool), (0.0, 0.0), 0.05, 0.25, 0.25, 0.0, 0.7, 0.58)
    assert math.isinf(gap) and cell is None


def test_colours_give_one_colour_per_label_even_with_many_cases():
    c = rr.colours(["a", "b", "a", "b", "a", "b", "a", "b", "a", "b"])
    assert c[0] == c[2] == c[8] and c[1] == c[3] and c[0] != c[1]


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
