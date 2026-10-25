"""Run: python3 -B test_speed_guard.py"""
import math

import numpy as np

import speed_guard as sg


def pts(*p):
    return np.array(p, float).reshape(-1, 2)


def test_free_distance_is_measured_from_the_robot_front_to_the_nearest_lethal_cell_in_the_lane():
    p = pts((1.0, 0.0), (0.8, 0.2), (2.0, 0.0))                 # robot frame: x forward, y left
    assert abs(sg.free_distance(p, half_width=0.4, front=0.35, horizon=1.5) - (0.8 - 0.35)) < 1e-9


def test_cells_outside_the_lane_behind_the_robot_or_beyond_the_horizon_are_ignored():
    # horizon is measured from the robot front: 0.35 + 1.5 = 1.85 m from the robot centre
    assert math.isinf(sg.free_distance(pts((1.0, 0.6), (-0.5, 0.0), (2.0, 0.0)), 0.4, 0.35, 1.5))
    assert abs(sg.free_distance(pts((1.8, 0.0)), 0.4, 0.35, 1.5) - 1.45) < 1e-9             # just inside the horizon


def test_a_cell_closer_than_the_front_counts_as_zero_free_distance():
    assert sg.free_distance(pts((0.2, 0.0)), 0.4, 0.35, 1.5) == 0.0


def test_no_cells_means_infinite_free_distance():
    assert math.isinf(sg.free_distance(pts(), 0.4, 0.35, 1.5))


def test_speed_limit_lets_the_robot_stop_inside_the_free_distance():
    a, margin = 1.0, 0.15
    v = sg.speed_limit(d_free=0.65, decel=a, margin=margin, v_min=0.05)
    assert abs(v - math.sqrt(2 * a * 0.5)) < 1e-9
    assert v * v / (2 * a) <= 0.65 - margin + 1e-9               # stopping distance fits before the margin


def test_speed_limit_is_floored_and_absent_without_an_obstacle():
    assert sg.speed_limit(0.10, 1.0, 0.15, 0.05) == 0.05          # inside the margin: not zero, because 0.0 means "no limit" to Nav2
    assert sg.speed_limit(math.inf, 1.0, 0.15, 0.05) is None


def test_nearest_in_lane_returns_the_cell_that_sets_the_free_distance():
    assert sg.nearest_in_lane(pts((1.0, 0.0), (0.8, 0.2), (0.8, 0.9)), 0.4, 0.35, 1.5) == (0.8, 0.2)
    assert sg.nearest_in_lane(pts((1.0, 0.9)), 0.4, 0.35, 1.5) is None


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
