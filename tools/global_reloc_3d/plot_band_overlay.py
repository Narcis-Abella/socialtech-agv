"""Plot frozen hypotheses over the map, both cut to the 0.10-1.80 m map-frame band."""
import argparse
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from offline_s2 import transform


def band(points, lo=.10, hi=1.80):
    return points[(points[:, 2] >= lo) & (points[:, 2] <= hi)]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--case', nargs=3, action='append', metavar=('DIR', 'TITLE', 'HALF_M'), required=True,
                   help='directory with map.xyz, check.xyz, result.json')
    p.add_argument('--out', required=True)
    a = p.parse_args()
    fig, axes = plt.subplots(1, len(a.case), figsize=(4.2 * len(a.case), 4), layout='constrained')
    for ax, (d, title, half) in zip(np.atleast_1d(axes), a.case):
        half = float(half)
        world = band(np.loadtxt(f'{d}/map.xyz'))
        T = np.array(json.load(open(f'{d}/result.json'))['T_map_lidar_refined'])
        q = band(transform(np.loadtxt(f'{d}/check.xyz'), T))
        x, y = T[0, 3], T[1, 3]
        near = world[(abs(world[:, 0] - x) < half) & (abs(world[:, 1] - y) < half)]
        ax.scatter(near[:, 0], near[:, 1], s=.3, c='0.6', rasterized=True, label='map band')
        ax.scatter(q[:, 0], q[:, 1], s=.6, c='#d94841', alpha=.6, rasterized=True, label='query band')
        ax.plot([x], [y], 'k^', ms=5)
        ax.set_xlim(x - half, x + half); ax.set_ylim(y - half, y + half)
        ax.set_aspect('equal'); ax.set_title(title, fontsize=9)
        ax.set_xlabel('map x (m)'); ax.set_ylabel('map y (m)')
    np.atleast_1d(axes)[0].legend(loc='lower left', fontsize=7, markerscale=8)
    fig.savefig(a.out, dpi=110)


if __name__ == '__main__':
    main()
