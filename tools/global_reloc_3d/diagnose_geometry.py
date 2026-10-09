"""Evaluator-only geometry diagnosis. Never mount this with the matcher."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from evaluate import reference_at
from offline_s2 import map_transform, transform


def profile(points):
    points = np.asarray(points)
    span = np.ptp(points, axis=0)
    # Eigenvalues describe shape; they do not identify semantic surfaces.
    eigenvalues = np.linalg.eigvalsh(np.cov(points.T))[::-1]
    return {'points': int(len(points)), 'span_xyz_m': span.tolist(),
            'eigenvalues_m2': eigenvalues.tolist(),
            'z_quantiles_m': np.percentile(points[:, 2], [0, 25, 50, 75, 100]).tolist()}


def nearest_profile(map_points, query_points):
    d = cKDTree(map_points).query(query_points, workers=1)[0]
    supported = d <= .15
    return {'support_015': float(np.mean(supported)),
            'rmse_supported_m': float(np.sqrt(np.mean(d[supported] ** 2))) if np.any(supported) else None,
            'nearest_quantiles_m': np.percentile(d, [50, 90, 95, 99]).tolist()}


def analyze(run, reference, map_yaml):
    run = Path(run)
    manifest = json.loads((run / 'manifest.json').read_text())
    if manifest['frame_fit'] != 'local_first_imu':
        raise ValueError('requires FAST-LIO local_first_imu input')
    prior = np.loadtxt(run / 'map.xyz')
    fit = np.loadtxt(run / 'fit.xyz')
    check = np.loadtxt(run / 'check.xyz')
    T_ref = map_transform(map_yaml) @ reference_at(
        np.loadtxt(reference, usecols=range(8)), manifest['last_scan_end_s'])
    T_map_local = T_ref @ np.linalg.inv(np.asarray(manifest['T_local_imu_at_last']))
    out = {'frame': 'map', 'truth_used': True, 'role': 'post_method_diagnostic_only',
           'represented_time_s': manifest['last_scan_end_s'],
           'prior': profile(prior), 'fit_local': profile(fit), 'check_local': profile(check)}
    for name, cloud in [('fit', fit), ('check', check)]:
        query_map = transform(cloud, T_map_local)
        low = query_map.min(axis=0) - 1.
        high = query_map.max(axis=0) + 1.
        vicinity = prior[np.all((prior >= low) & (prior <= high), axis=1)]
        out[name + '_at_reference'] = {
            'profile_map_frame': profile(query_map),
            'prior_points_in_query_box_plus_1m': int(len(vicinity)),
            'nearest': nearest_profile(prior, query_map)}
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run', required=True)
    p.add_argument('--reference', required=True)
    p.add_argument('--map-yaml', required=True)
    p.add_argument('--out', required=True)
    a = p.parse_args()
    result = analyze(a.run, a.reference, a.map_yaml)
    Path(a.out).write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: {'points': v.get('points'), 'support_015': v.get('nearest', {}).get('support_015'),
                          'prior_points_in_query_box_plus_1m': v.get('prior_points_in_query_box_plus_1m')}
                      for k, v in result.items() if isinstance(v, dict)}, indent=2))


if __name__ == '__main__':
    main()
