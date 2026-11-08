"""Truth-free geometric diagnostic; absent alternative/temporal evidence abstains."""

import numpy as np
from scipy.spatial import cKDTree

from pointcloud import transform


def score(map_points, check_points, T, tree=None):
    """tree: a cKDTree of map_points when the caller scores many poses on the same map (it is the cost of the call)."""
    T = np.asarray(T, float)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        return {'valid': False, 'reason': 'nonfinite_transform'}
    d = (tree if tree is not None else cKDTree(map_points)).query(transform(check_points, T), workers=1)[0]
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
