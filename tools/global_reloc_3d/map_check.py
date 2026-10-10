"""Truth-free check of a PROPAGATED pose against the map, from the newest scans only (killable worker, like search_v2.py).

Why: an accepted registration is some seconds old when its pose is handed over; the node carries it to "now" through FAST-LIO2
odometry. If the odometry diverges in between, the pose that goes out is wrong although the registration was right (case h27,
note 21). Here the newest deskewed body clouds are placed in the map WITH that propagated pose and must fit the map by
themselves: GICP refinement started from the propagated pose, then the unchanged geometric rule of verify.decide() on a later
batch the refinement never saw, and the refined pose must not be a different pose (search_tiles.distinct: 0.30 m / 8 deg).
A diverged odometry puts the scans where the map is not, so nothing here trusts the odometry it is testing; within the window
only the relative odometry between its scans is used.

  map_check.py --input snapshot.npz --map map.xyz --binary register_v2 --out NEW_DIR [--timeout-s 8]
snapshot.npz: T_map_ci (4x4, map <- odometry frame), odom_t (n), odom_T (n,4,4), cloud_t (m), cloud_n (m), cloud_xyz (sum n, 3)."""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from lio_submap import interpolate, rebase
from offline_s2 import voxel, write_xyz
from search_tiles import distinct, pose_difference
from verify import decide, score

VOXEL_M = .20        # as lio_submap.assemble
GICP_VOXEL_M = .3    # as search_v2 --gicp-voxel
MIN_CLOUDS = 6


def batches(odom, clouds):
    """(fit, check, T_ci_first): the clouds in the body frame of the first one, split 2/3 - 1/3 in time as lio_submap.assemble."""
    clouds = sorted(clouds, key=lambda c: c[0])
    if len(clouds) < MIN_CLOUDS:
        raise ValueError('too_few_clouds')
    first = interpolate(odom, clouds[0][0])
    t0, span = clouds[0][0], clouds[-1][0] - clouds[0][0]
    fit = [rebase(p, first, interpolate(odom, t)) for t, p in clouds if t < t0 + span * 2 / 3]
    check = [rebase(p, first, interpolate(odom, t)) for t, p in clouds if t >= t0 + span * 2 / 3]
    if not fit or not check:
        raise ValueError('batch_missing')
    return voxel(np.concatenate(fit), VOXEL_M), voxel(np.concatenate(check), VOXEL_M), first


def judge(map_points, check, T_init, result):
    """result = register_v2 output (or None). Returns the verdict dict; ok only if every rule holds."""
    out = {'ok': False, 'reason': None, 'at_propagated': score(map_points, check, T_init), 'truth_used': False}
    if result is None or not result.get('gicp_converged'):
        out['reason'] = 'solver'
        return out
    T = np.asarray(result['T_map_lidar_refined'])
    out['geometry'] = geom = score(map_points, check, T)
    xy, deg = pose_difference(np.asarray(T_init), T)
    out['shift'] = {'xy_m': xy, 'yaw_deg': deg}
    # verify.decide() without alternative/temporal evidence answers 'ambiguous_no_alternative_margin' exactly when the solver and
    # geometry rules pass. Uniqueness was established by the global search; this check only asks "still there?".
    verdict = decide({**result, 'kiss_valid': True}, geom)
    if verdict in ('reject_solver', 'reject_geometry'):
        out['reason'] = verdict[len('reject_'):]
    elif distinct(np.asarray(T_init), T):
        out['reason'] = 'shift'
    else:
        out['ok'] = True
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--input', required=True)
    p.add_argument('--map', required=True)
    p.add_argument('--binary', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--timeout-s', type=float, default=8.)
    a = p.parse_args()
    from search_v2 import refine
    t0 = time.monotonic()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=False)
    z = np.load(a.input)
    odom = list(zip(z['odom_t'].tolist(), z['odom_T']))
    clouds = list(zip(z['cloud_t'].tolist(), np.split(z['cloud_xyz'], np.cumsum(z['cloud_n'])[:-1])))
    try:
        fit, check, first = batches(odom, clouds)
    except ValueError as e:
        report = {'ok': False, 'reason': str(e), 'truth_used': False}
    else:
        write_xyz(out / 'fit.xyz', fit); write_xyz(out / 'check.xyz', check)
        T_init = z['T_map_ci'] @ first
        t_g = time.monotonic()
        log, result = refine(a.binary, a.map, out / 'fit.xyz', out / 'check.xyz', T_init, out / 'gicp', GICP_VOXEL_M,
                             a.timeout_s - (time.monotonic() - t0))
        t_s = time.monotonic()
        report = judge(np.loadtxt(a.map), check, T_init, result)
        if log['exit_code'] == 124:
            report['reason'] = 'timeout'
        report.update(run=log, fit_points=len(fit), check_points=len(check), gicp_s=t_s - t_g, score_s=time.monotonic() - t_s)
    report.update(clouds=len(clouds), window_s=float(z['cloud_t'][-1] - z['cloud_t'][0]) if len(clouds) else 0.,
                  newest_cloud_stamp=float(z['cloud_t'][-1]) if len(clouds) else None, elapsed_s=time.monotonic() - t0)
    (out / 'check.json').write_text(json.dumps(report, indent=2, default=float) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'run'}, default=float))


if __name__ == '__main__':
    main()
