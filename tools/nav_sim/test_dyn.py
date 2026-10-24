"""Run: python3 -B test_dyn.py"""
import math

import dyn


def close(a, b, tol=1e-6):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_path_pose_moves_at_constant_speed_along_the_polyline_and_faces_its_heading():
    path = [[0.0, 0.0], [4.0, 0.0], [4.0, 3.0]]
    assert close(dyn.path_pose(path, 2.0, 1.0), (2.0, 0.0, 0.0))
    assert close(dyn.path_pose(path, 2.0, 2.5), (4.0, 1.0, math.pi / 2))     # 5 m travelled: 1 m up the second leg


def test_path_pose_waits_before_the_start_and_stops_at_the_end():
    path = [[1.0, 1.0], [1.0, -1.0]]
    assert close(dyn.path_pose(path, 1.0, -3.0), (1.0, 1.0, -math.pi / 2))
    assert close(dyn.path_pose(path, 1.0, 99.0), (1.0, -1.0, -math.pi / 2))


def test_path_pose_ignores_repeated_points():
    assert close(dyn.path_pose([[0.0, 0.0], [0.0, 0.0], [2.0, 0.0]], 1.0, 1.0), (1.0, 0.0, 0.0))


def test_rect_distance_is_zero_when_overlapping_or_touching_and_exact_when_apart():
    a = (0.0, 0.0, 0.0, 1.0, 1.0)                        # x, y, yaw, size_x, size_y
    assert dyn.rect_distance(a, (0.5, 0.0, 0.0, 1.0, 1.0)) == 0.0          # overlap
    assert dyn.rect_distance(a, (1.0, 0.0, 0.0, 1.0, 1.0)) < 1e-9          # sharing a face
    assert abs(dyn.rect_distance(a, (2.0, 0.0, 0.0, 1.0, 1.0)) - 1.0) < 1e-9
    assert abs(dyn.rect_distance(a, (2.0, 2.0, 0.0, 1.0, 1.0)) - math.sqrt(2)) < 1e-9   # corner to corner


def test_rect_distance_with_rotation_and_one_inside_the_other():
    big = (0.0, 0.0, 0.0, 4.0, 4.0)
    assert dyn.rect_distance(big, (0.0, 0.0, 0.7, 0.5, 0.5)) == 0.0        # fully inside: no edge crosses
    diamond = (3.0, 0.0, math.pi / 4, 1.0, 1.0)                              # half diagonal 0.7071: its left corner is at x = 2.2929
    assert abs(dyn.rect_distance((0.0, 0.0, 0.0, 2.0, 2.0), diamond) - (3.0 - math.sqrt(0.5) - 1.0)) < 1e-9


def test_rect_distance_of_crossed_rectangles_without_vertex_inside_is_zero():
    assert dyn.rect_distance((0.0, 0.0, 0.0, 6.0, 0.2), (0.0, 0.0, 0.0, 0.2, 6.0)) == 0.0     # a plus sign: no vertex of either lies inside the other


def test_triggered_is_a_circle_around_a_point():
    assert dyn.triggered((5.0, 0.0), [5.5, 0.0, 1.0])
    assert not dyn.triggered((5.0, 0.0), [7.5, 0.0, 1.0])
    assert not dyn.triggered(None, [0.0, 0.0, 1.0])


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
