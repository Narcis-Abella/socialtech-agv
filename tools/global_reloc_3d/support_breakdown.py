"""Truth-free support breakdown of frozen hypotheses: where does check support come from?

For each distinct top hypothesis of a search, the check batch is placed in the map
and split by (a) map-frame height band (0.10-1.80 m: the PGM obstacle band; map
z = 0 is the floor by construction of make_map) and (b) local surface normal
(vertical surface = wall-like, horizontal = floor/ceiling-like). Diagnostic only:
no threshold here is an acceptance rule.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from offline_s2 import transform


def normals_z(points, k=12):
    """|n_z| of the PCA normal from k neighbours (1 = horizontal surface)."""
    _, idx = cKDTree(points).query(points, k=k)
    nb = points[idx] - points[idx].mean(axis=1, keepdims=True)
    cov = np.einsum('nki,nkj->nij', nb, nb)
    _, vec = np.linalg.eigh(cov)
    return np.abs(vec[:, 2, 0])


def breakdown(map_tree, check_local, nz, T, band=(.10, 1.80), radius=.15):
    q = transform(check_local, T)
    ok = map_tree.query(q, workers=1)[0] <= radius
    in_band = (q[:, 2] >= band[0]) & (q[:, 2] <= band[1])
    vertical = nz < .3
    horizontal = nz > .9
    out = {}
    for name, m in [('all', np.ones(len(q), bool)), ('band', in_band), ('vertical', vertical),
                    ('band_vertical', in_band & vertical), ('horizontal', horizontal)]:
        out[name] = {'n': int(m.sum()), 'support': float(ok[m].mean()) if m.any() else None}
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run', required=True)
    p.add_argument('--search', required=True)
    p.add_argument('--out', required=True)
    a = p.parse_args()
    run, search = Path(a.run), Path(a.search)
    report = json.loads((search / 'search.json').read_text())
    check = np.loadtxt(run / 'check.xyz')
    tree = cKDTree(np.loadtxt(run / 'map.xyz'))
    nz = normals_z(check)  # levelled local frame: z is up for every yaw hypothesis
    rows = {}
    for hid in report['distinct_top_ids']:
        T = np.asarray(json.loads((search / hid / 'result.json').read_text())['T_map_lidar_refined'])
        rows[hid] = breakdown(tree, check, nz, T)
    out = {'primary_id': report['primary_id'], 'decision': report['decision'], 'hypotheses': rows,
           'truth_used': False}
    Path(a.out).write_text(json.dumps(out, indent=2) + '\n')


if __name__ == '__main__':
    main()
