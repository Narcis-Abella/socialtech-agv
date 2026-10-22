"""Truth-free geometric diagnostic; absent alternative/temporal evidence abstains."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from offline_s2 import transform


def score(map_points, check_points, T):
    T = np.asarray(T, float)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        return {'valid': False, 'reason': 'nonfinite_transform'}
    d = cKDTree(map_points).query(transform(check_points, T), workers=1)[0]
    supported = d <= .15
    return {'valid': True, 'support_015': float(np.mean(supported)),
            'matched_rmse_m': float(np.sqrt(np.mean(d[supported]**2))) if np.any(supported) else None,
            'median_nearest_m': float(np.median(d)), 'p95_nearest_m': float(np.percentile(d, 95)),
            'n_check': len(check_points), 'n_supported': int(np.sum(supported))}


def decide(result, geom, alternative=None, temporal=None):
    if not result['kiss_valid'] or not result['gicp_converged']:
        return 'reject_solver'
    if not geom['valid'] or geom['support_015'] < .70 or geom['matched_rmse_m'] is None or geom['matched_rmse_m'] > .10:
        return 'reject_geometry'
    if alternative is None or alternative < .10:
        return 'ambiguous_no_alternative_margin'
    if temporal is None or not temporal:
        return 'ambiguous_no_temporal_confirmation'
    return 'accepted'


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run', required=True)
    p.add_argument('--out', required=True)
    a = p.parse_args()
    run = Path(a.run)
    result = json.loads((run/'result.json').read_text())
    map_points = np.loadtxt(run/'map.xyz')
    check_points = np.loadtxt(run/'check.xyz')
    g = score(map_points, check_points, result['T_map_lidar_refined'])
    status = decide(result, g)
    output = {'geometry': g, 'decision': status, 'accepted': status == 'accepted',
              'alternative_margin': None, 'temporal_confirmation': None,
              'policy': 'point4_initial_3d_gate'}
    Path(a.out).write_text(json.dumps(output, indent=2)+'\n')
    print(json.dumps(output, indent=2))


if __name__ == '__main__':
    main()
