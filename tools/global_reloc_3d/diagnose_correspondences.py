"""Evaluator-only: score dumped KISS correspondences against the reference pose.

Uses the reference. Run it only after the method output is frozen, never inside
the method container. It answers: did FPFH produce correct pairs, and did ROBIN
keep them?
"""
import argparse
import json
from pathlib import Path

import numpy as np

from evaluate import reference_at
from offline_s2 import map_transform, transform


def oracle_map_local(manifest, reference, map_yaml):
    T_ref = map_transform(map_yaml) @ reference_at(
        np.loadtxt(reference, usecols=range(8)), manifest['last_scan_end_s'])
    T_level_local = np.asarray(manifest.get('T_level_local', np.eye(4)))
    # Registration source frame is the levelled local frame when present.
    return T_ref @ np.linalg.inv(np.asarray(manifest['T_local_imu_at_last'])) @ np.linalg.inv(T_level_local)


def pair_errors(source, target, T):
    source = np.asarray(source, float).reshape(-1, 3)
    target = np.asarray(target, float).reshape(-1, 3)
    if not len(source):
        return np.zeros(0)
    return np.linalg.norm(transform(source, T) - target, axis=1)


def summarize(errors):
    n = len(errors)
    out = {'n': n}
    for r in (.5, 1.):
        out[f'correct_le_{r:.1f}m'] = int(np.sum(errors <= r))
    if n:
        out['inlier_ratio_le_1.0m'] = out['correct_le_1.0m'] / n
    return out


def analyze(manifest, result, T_oracle):
    return {'cross_checked': summarize(pair_errors(result.get('cross_source', []), result.get('cross_target', []), T_oracle)),
            'pruned': summarize(pair_errors(result.get('pruned_source', []), result.get('pruned_target', []), T_oracle)),
            'kiss_valid': result['kiss_valid'], 'translation_inliers': result['translation_inliers'],
            'truth_used': True, 'role': 'post_method_diagnostic_only'}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run', required=True)
    p.add_argument('--result', required=True, nargs='+')
    p.add_argument('--reference', required=True)
    p.add_argument('--map-yaml', required=True)
    p.add_argument('--out', required=True)
    a = p.parse_args()
    manifest = json.loads((Path(a.run) / 'manifest.json').read_text())
    T = oracle_map_local(manifest, a.reference, a.map_yaml)
    rows = {}
    for path in a.result:
        rows[path] = analyze(manifest, json.loads(Path(path).read_text()), T)
        print(path, json.dumps({k: rows[path][k] for k in ('cross_checked', 'pruned', 'translation_inliers')}))
    Path(a.out).write_text(json.dumps({'T_map_source_oracle': T.tolist(), 'results': rows}, indent=2) + '\n')


if __name__ == '__main__':
    main()
