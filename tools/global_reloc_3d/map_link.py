"""Evaluator-only frame link between two sessions' maps (map-to-map, whole maps).

T_A_B maps session-B map coordinates into session-A map coordinates. It is built
only from the two whole maps: KISS-Matcher global registration (descriptor-based,
not the localizer's FFT generator) then GICP, both via register_v2. Uncertainty:
(1) GICP restarts from perturbed starts, (2) the two halves of map B (split at its
median x) registered separately: their disagreement bounds map deformation.
No localizer output and no per-window query enters this link.
"""
import argparse
import json
import math
import subprocess
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from correlative_search import pose
from offline_s2 import transform
from search_tiles import pose_difference


def register(binary, target, source, out, voxel, init=None):
    args = [str(binary), str(target), str(source), str(source), str(voxel), str(out)]
    if init is not None:
        np.savetxt(Path(out).with_suffix('.init'), init)
        args += ['--init', str(Path(out).with_suffix('.init'))]
    subprocess.run(args, capture_output=True, text=True, timeout=600)
    return json.loads(Path(out).read_text())


def residual(tree, points, T, band=(.10, 1.80)):
    q = transform(points, T)
    q = q[(q[:, 2] >= band[0]) & (q[:, 2] <= band[1])]
    d = tree.query(q, workers=1)[0]
    near = d <= .5  # overlap proxy: band points that have map within 0.5 m
    return {'band_points': int(len(q)), 'overlap_le_0.5m': float(near.mean()),
            'median_m_in_overlap': float(np.median(d[near])), 'p95_m_in_overlap': float(np.percentile(d[near], 95))}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--map-a', required=True, help='known map (target)')
    p.add_argument('--map-b', required=True, help='other session map (source)')
    p.add_argument('--binary', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--voxel', type=float, default=.5)
    a = p.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=False)
    A, B = np.loadtxt(a.map_a), np.loadtxt(a.map_b)
    kiss = register(a.binary, a.map_a, a.map_b, out / 'kiss.json', a.voxel)
    T = np.asarray(kiss['T_map_lidar_refined'])
    rng = np.random.default_rng(7)
    restarts = []
    for i in range(10):
        d = pose(math.radians(rng.uniform(-3, 3)), [*rng.uniform(-.5, .5, 2), 0.]) @ T
        r = register(a.binary, a.map_a, a.map_b, out / f'restart_{i}.json', .3, d)
        restarts.append(pose_difference(T, np.asarray(r['T_map_lidar_refined'])))
    halves = []
    x_mid = np.median(B[:, 0])
    for name, part in [('west', B[B[:, 0] < x_mid]), ('east', B[B[:, 0] >= x_mid])]:
        path = out / f'half_{name}.xyz'
        np.savetxt(path, part, fmt='%.6f')
        r = register(a.binary, a.map_a, path, out / f'half_{name}.json', .3, T)
        halves.append({'half': name, 'points': int(len(part)),
                       'diff_to_link': pose_difference(T, np.asarray(r['T_map_lidar_refined'])),
                       'T': r['T_map_lidar_refined']})
    report = {'T_A_B': T.tolist(), 'kiss': {k: kiss[k] for k in ('kiss_valid', 'cross_checked', 'initial_matches',
                                                                'translation_inliers', 'gicp_converged')},
              'kiss_initial_vs_refined': pose_difference(np.asarray(kiss['T_map_lidar_kiss']), T),
              'restart_spread_xy_yaw': {'max_xy_m': max(r[0] for r in restarts), 'max_yaw_deg': max(r[1] for r in restarts)},
              'halves': halves,
              'halves_disagreement_xy_yaw': pose_difference(np.asarray(halves[0]['T']), np.asarray(halves[1]['T'])),
              'residual_band': residual(cKDTree(A), B, T),
              'tilt_deg': math.degrees(math.acos(min(1., T[2, 2]))),
              'role': 'evaluator_only_frame_link', 'uses_localizer_output': False}
    (out / 'link.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'halves'}, indent=2))


if __name__ == '__main__':
    main()
