"""Evaluator-only: rank of the first candidate within 0.30 m / 8 deg of the reference.

Retrieval recall is not correct_accept (protocol: a top-K set containing the true
pose without acceptance is retrieval recall). Run only after search.json is frozen.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from evaluate import error, reference_at
from offline_s2 import map_transform


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run', required=True)
    p.add_argument('--search', required=True)
    p.add_argument('--reference', required=True)
    p.add_argument('--map-yaml', required=True)
    a = p.parse_args()
    m = json.loads((Path(a.run) / 'manifest.json').read_text())
    s = json.loads((Path(a.search) / 'search.json').read_text())
    T_ref = map_transform(a.map_yaml) @ reference_at(np.loadtxt(a.reference, usecols=range(8)), m['last_scan_end_s'])
    tail = np.asarray(m.get('T_level_local', np.eye(4))) @ np.asarray(m['T_local_imu_at_last'])
    rows = []
    for r in s['all_trials']:
        if 'result' not in r:
            continue
        for k in ('kiss', 'refined'):
            T = np.asarray(r['result']['T_map_lidar_' + k]) @ tail
            r[k] = error(T, T_ref)
        rows.append((r['id'], r['correlative']['score'], r['kiss'], r['refined'], r.get('geometry', {}).get('support_015')))
    print(json.dumps({'search': a.search, 'decision': s['decision'], 'candidates': rows}))


if __name__ == '__main__':
    main()
