"""Truth-free check of a PROPAGATED pose against the map, from the newest scans only (killable worker, like search_v2.py).

Why: an accepted registration is some seconds old when its pose is handed over; the node carries it to "now" through FAST-LIO2
odometry. If the odometry diverges in between, the pose that goes out is wrong although the registration was right (case h27,
note 21). Here the newest deskewed body clouds are placed in the map WITH that propagated pose and must fit the map by
themselves: GICP refinement started from the propagated pose, then the unchanged geometric rule of verify.decide() on a later
batch the refinement never saw, and the refined pose must not be a different pose (search_tiles.distinct: 0.30 m / 8 deg).
A diverged odometry puts the scans where the map is not, so nothing here trusts the odometry it is testing; within the window
only the relative odometry between its scans is used.

  map_check.py --input snapshot.npz --map map.xyz --binary register_v2 --out NEW_DIR [--timeout-s 8]
  map_check.py --serve --map map.xyz --binary register_v2      (requests on stdin, see serve())
snapshot.npz: T_map_ci (4x4, map <- odometry frame), odom_t (n), odom_T (n,4,4), cloud_t (m), cloud_n (m), cloud_xyz (sum n, 3)."""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from lio_submap import interpolate, rebase
from offline_s2 import write_xyz
from search_tiles import distinct, pose_difference
from verify import decide, score

VOXEL_M = .20        # as lio_submap.assemble
GICP_VOXEL_M = .3    # as search_v2 --gicp-voxel
MIN_CLOUDS = 6
TAIL_S = .5          # the newest scans, judged on their own after LOCALIZED (note 22)
TAIL_MIN_CLOUDS = 3


def voxel(points, size):
    """offline_s2.voxel() with the same output, bit for bit (test_map_check), several times faster: the three cell indices are
    packed into one integer in the same lexicographic order, so np.unique sorts numbers instead of rows. A check after LOCALIZED
    spent most of its time in that sort (note 22)."""
    points = np.asarray(points, dtype=np.float64)
    if not len(points):
        return points.reshape(0, 3)
    cell = np.floor(points / size).astype(np.int64)
    cell -= cell.min(axis=0)
    ny, nz = int(cell[:, 1].max()) + 1, int(cell[:, 2].max()) + 1
    _, inverse, counts = np.unique((cell[:, 0] * ny + cell[:, 1]) * nz + cell[:, 2], return_inverse=True, return_counts=True)
    return np.column_stack([np.bincount(inverse, weights=points[:, j]) for j in range(3)]) / counts[:, None]


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


def tail_batch(odom, clouds, first, tail_s=TAIL_S):
    """The scans of the last tail_s, and never fewer than the TAIL_MIN_CLOUDS newest ones, in the body frame of the first scan of
    the window. (A short gap in the cloud stream left 2 scans in 0.5 s in a no-fault control, k40-x1, note 22: the rule then
    failed for lack of data and the gate closed for 0.7 s. Missing data is the business of the freshness rule, not of this one.)"""
    clouds = sorted(clouds, key=lambda c: c[0])
    new = [(t, p) for t, p in clouds if t >= clouds[-1][0] - tail_s]
    new = new if len(new) >= TAIL_MIN_CLOUDS else clouds[-TAIL_MIN_CLOUDS:]
    return voxel(np.concatenate([rebase(p, first, interpolate(odom, t)) for t, p in new]), VOXEL_M)


def judge(map_points, check, T_init, result, tail=..., tail_rule=False):
    """result = register_v2 output (or None). Returns the verdict dict; ok only if every rule holds.

    tail (tail_batch(), optional): the newest scans placed with the same refined pose. The window is 3 s and the rule above looks
    at its last second as a whole, so an odometry that started to drift a moment ago still passes (run k11_r3-w1, note 22: support
    0.700 with AMCL 0.46 m off). With tail_rule the unchanged geometric rule must also hold on the newest TAIL_S alone."""
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
    if tail is not ...:
        out['tail'] = tg = score(map_points, tail, T) if tail is not None else None
        if out['ok'] and tail_rule and (tg is None or decide({**result, 'kiss_valid': True}, tg) == 'reject_geometry'):
            out['ok'], out['reason'] = False, 'tail_geometry' if tg is not None else 'tail_missing'
    return out


def run_check(input_path, map_path, binary, out, timeout_s, map_points=None, tail_rule=False):
    """One check. map_points: the map already in memory (serve mode); the rule and the data path are the same either way."""
    from search_v2 import refine
    t0 = time.monotonic()
    out = Path(out); out.mkdir(parents=True, exist_ok=False)
    z = np.load(input_path)
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
        log, result = refine(binary, map_path, out / 'fit.xyz', out / 'check.xyz', T_init, out / 'gicp', GICP_VOXEL_M,
                             timeout_s - (time.monotonic() - t0))
        t_s = time.monotonic()
        report = judge(np.loadtxt(map_path) if map_points is None else map_points, check, T_init, result,
                       tail=tail_batch(odom, clouds, first), tail_rule=tail_rule)
        if log['exit_code'] == 124:
            report['reason'] = 'timeout'
        report.update(run=log, fit_points=len(fit), check_points=len(check), gicp_s=t_s - t_g, score_s=time.monotonic() - t_s)
    report.update(clouds=len(clouds), window_s=float(z['cloud_t'][-1] - z['cloud_t'][0]) if len(clouds) else 0.,
                  newest_cloud_stamp=float(z['cloud_t'][-1]) if len(clouds) else None, elapsed_s=time.monotonic() - t0)
    (out / 'check.json').write_text(json.dumps(report, indent=2, default=float) + '\n')
    return report


def serve(a):
    """Stay alive between checks (after LOCALIZED the interpreter start and the imports were most of a check's wall time).
    One JSON request per stdin line ({"input": snapshot.npz, "out": NEW_DIR, "tail_rule": bool}), one JSON verdict per stdout line. EOF ends it.
    The parent kills this process when a verdict is late: a hung check is a failed check there."""
    import sys
    map_points = np.loadtxt(a.map)
    for line in sys.stdin:
        req = json.loads(line)
        try:
            r = run_check(req['input'], a.map, a.binary, req['out'], a.timeout_s, map_points, bool(req.get('tail_rule')))
        except Exception as e:                      # unreadable snapshot, full scratch...: a failed check, the server lives on
            r = {'ok': False, 'reason': f'worker_{type(e).__name__}', 'truth_used': False}
        print(json.dumps({k: v for k, v in r.items() if k != 'run'}, default=float), flush=True)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--input')
    p.add_argument('--map', required=True)
    p.add_argument('--binary', required=True)
    p.add_argument('--out')
    p.add_argument('--timeout-s', type=float, default=8.)
    p.add_argument('--serve', action='store_true')
    a = p.parse_args()
    if a.serve:
        return serve(a)
    if not a.input or not a.out:
        p.error('--input and --out are required without --serve')
    report = run_check(a.input, a.map, a.binary, a.out, a.timeout_s)
    print(json.dumps({k: v for k, v in report.items() if k != 'run'}, default=float))


if __name__ == '__main__':
    main()
