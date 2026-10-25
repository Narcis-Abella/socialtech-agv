"""Run: python3 -B test_ghost.py"""
import numpy as np

import ghost


def grid(w=20, h=10):
    return np.zeros(w * h, np.int16), np.zeros(w * h, np.int16)


def test_lethal_cells_off_the_static_map_are_counted():
    cm, st = grid()
    cm[5], cm[27] = 100, 100                       # two lethal cells on free map cells
    st[40] = 100; cm[40] = 100                     # a wall: lethal in both, not a ghost
    assert ghost.wake_cells(cm, st, 20, 10, (0.0, 0.0), 0.05, []) == 2


def test_cells_near_a_real_obstacle_are_not_counted_as_wake():
    cm, st = grid()
    cm[5] = 100                                    # cell (row 0, col 5): centre x = 0.275, y = 0.025
    cm[27] = 100                                   # cell (row 1, col 7): centre x = 0.375, y = 0.075
    assert ghost.wake_cells(cm, st, 20, 10, (0.0, 0.0), 0.05, [(0.3, 0.05)], radius=0.1) == 0
    assert ghost.wake_cells(cm, st, 20, 10, (0.0, 0.0), 0.05, [(5.0, 5.0)], radius=0.1) == 2


def test_inflation_values_below_lethal_are_not_counted():
    cm, st = grid()
    cm[3], cm[4] = 99, 50
    assert ghost.wake_cells(cm, st, 20, 10, (0.0, 0.0), 0.05, []) == 0


def test_wake_xy_gives_the_cell_centres():
    cm, st = grid()
    cm[27] = 100
    x, y = ghost.wake_xy(cm, st, 20, 10, (1.0, 2.0), 0.05, [])
    assert list(np.round(x, 3)) == [1.375] and list(np.round(y, 3)) == [2.075]


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
