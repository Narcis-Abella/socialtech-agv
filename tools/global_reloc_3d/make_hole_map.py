"""Evaluator-side negative-control map: remove all prior points within `radius` m (XY)
of the window's reference position. The method only ever sees the resulting map."""
import argparse

import numpy as np


def hole(points, xy, radius):
    points = np.asarray(points)
    return points[np.hypot(points[:, 0] - xy[0], points[:, 1] - xy[1]) > radius]


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--map', required=True)
    p.add_argument('--x', type=float, required=True)
    p.add_argument('--y', type=float, required=True)
    p.add_argument('--radius', type=float, default=8.)
    p.add_argument('--out', required=True)
    a = p.parse_args()
    m = np.loadtxt(a.map)
    h = hole(m, (a.x, a.y), a.radius)
    np.savetxt(a.out, h, fmt='%.6f')
    print(f'kept {len(h)} of {len(m)} points')
