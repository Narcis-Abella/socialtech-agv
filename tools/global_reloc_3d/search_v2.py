"""Truth-free global search v2: correlative (x, y, z, yaw) candidates, GICP, same gate.

Requires a gravity-levelled source (lio_submap.py --imu-bag). The acceptance rule
is verify.decide() unchanged: converged refinement, check support >= 0.70 with
matched RMSE <= 0.10 m, support lead >= 0.10 over a distinct runner-up, and an
independent global search of the later batch agreeing within 0.20 m / 5 deg,
inside the 60 s deadline. Candidates come only from the method's own search.
"""
import argparse
import json
import math
import os
from pathlib import Path
import resource
import subprocess
import time

import numpy as np

from correlative_search import MapField, crop_source, search
from search_tiles import agreement, distinct, pose_difference, temporal_evidence
from verify import decide, score

DEADLINE_S = 60.
STAGE_S = {'gicp': 0., 'score': 0., 'confirm_total': 0.}  # additive instrumentation, no effect on decisions


def tilt_deg(T):
    """Angle between the pose's z axis and map +z (both frames gravity-levelled)."""
    return float(math.degrees(math.acos(np.clip(np.asarray(T)[2, 2], -1., 1.))))


def refine(binary, map_path, source_path, check_path, T_init, out_dir, voxel, timeout_s):
    out_dir.mkdir()
    init = out_dir / 'init.txt'
    np.savetxt(init, np.asarray(T_init))
    try:
        proc = subprocess.run([str(binary), str(map_path), str(source_path), str(check_path), str(voxel),
                               str(out_dir / 'result.json'), '--init', str(init)],
                              capture_output=True, text=True, timeout=max(.001, timeout_s),
                              env={**os.environ, 'OMP_NUM_THREADS': '2'})
        log = {'exit_code': proc.returncode, 'stdout': proc.stdout[-500:], 'stderr': proc.stderr[-500:]}
    except subprocess.TimeoutExpired:
        return {'exit_code': 124, 'stderr': 'matcher_deadline_exceeded'}, None
    path = out_dir / 'result.json'
    return log, (json.loads(path.read_text()) if path.exists() else None)


def global_hypotheses(field, map_points, batch, batch_path, other, other_path, map_path, args, out, prefix, t0):
    """Correlative search on one batch, GICP refine, score on the other batch."""
    found = search(map_points, batch, args.res, yaw_step_deg=args.yaw_step, radius=args.radius,
                   top=args.top, field=field)
    rows = []
    for i, c in enumerate(found['candidates']):
        remaining = DEADLINE_S - (time.monotonic() - t0)
        if remaining <= 0:
            return found, rows, True
        t_g = time.monotonic()
        log, result = refine(args.binary, map_path, batch_path, other_path, c['T_map_source'],
                             out / f'{prefix}_{i:02d}', args.gicp_voxel, remaining)
        STAGE_S['gicp'] += time.monotonic() - t_g
        if log['exit_code'] == 124:
            return found, rows, True
        row = {'id': f'{prefix}_{i:02d}', 'correlative': {k: c[k] for k in ('score', 'yaw_rad', 'xyz')}, 'run': log}
        if result is not None:
            # The generator replaces KISS; keep verify.decide()'s interface.
            result['kiss_valid'] = True
            row['result'] = result
            if result['gicp_converged']:
                t_s = time.monotonic()
                row['geometry'] = score(map_points, other, result['T_map_lidar_refined'])
                STAGE_S['score'] += time.monotonic() - t_s
                row['tilt_deg'] = tilt_deg(result['T_map_lidar_refined'])
                xy, deg = pose_difference(np.asarray(c['T_map_source']), np.asarray(result['T_map_lidar_refined']))
                row['gicp_shift'] = {'xy_m': xy, 'yaw_deg': deg}
        rows.append(row)
    return found, rows, False


def rank(rows):
    valid = [r for r in rows if 'geometry' in r and r['geometry']['valid']]
    valid.sort(key=lambda r: (-r['geometry']['support_015'], r['geometry']['matched_rmse_m'] or 9.))
    top = []
    for r in valid:
        T = np.asarray(r['result']['T_map_lidar_refined'])
        if all(distinct(T, np.asarray(o['result']['T_map_lidar_refined'])) for o in top):
            top.append(r)
        if len(top) == 5:
            break
    return top


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run', required=True, help='prepared run with map/fit/check.xyz, levelled source')
    p.add_argument('--out', required=True, help='new versioned output directory')
    p.add_argument('--binary', required=True, help='register_v2 (GICP-only --init mode)')
    p.add_argument('--res', type=float, default=.5)
    p.add_argument('--yaw-step', type=float, default=2.5)
    p.add_argument('--radius', type=float, default=15.)
    p.add_argument('--top', type=int, default=10)
    p.add_argument('--gicp-voxel', type=float, default=.3)
    p.add_argument('--deadline-s', type=float, default=60., help='remaining decision budget from this process start')
    a = p.parse_args()
    global DEADLINE_S
    DEADLINE_S = a.deadline_s
    run, out = Path(a.run), Path(a.out)
    out.mkdir(parents=True, exist_ok=False)
    t0 = time.monotonic()
    manifest = json.loads((run / 'manifest.json').read_text())
    if manifest['frame_fit'] != 'local_first_imu_levelled':
        raise ValueError('search_v2 needs a gravity-levelled source (lio_submap.py --imu-bag)')
    full = np.loadtxt(run / 'map.xyz')
    fit = np.loadtxt(run / 'fit.xyz')
    check = np.loadtxt(run / 'check.xyz')
    reach_z = max(np.abs(crop_source(fit, a.radius)[:, 2]).max(), np.abs(crop_source(check, a.radius)[:, 2]).max())
    field = MapField(full, a.res, .5, a.radius + a.res, reach_z + a.res)
    t_field = time.monotonic() - t0
    temporal_ok, separation = temporal_evidence(manifest['stamps_s'], manifest['start_header_s'], manifest['duration_s'])
    found, rows, timed_out = global_hypotheses(field, full, fit, run / 'fit.xyz', check, run / 'check.xyz',
                                               run / 'map.xyz', a, out, 'cand', t0)
    top = rank(rows)
    primary = top[0] if top else None
    runner = top[1] if len(top) > 1 else None
    margin = primary['geometry']['support_015'] - runner['geometry']['support_015'] if runner else None
    confirmation = None
    if primary and not timed_out and time.monotonic() - t0 < DEADLINE_S:
        # Independent global search of the later batch; its best is chosen on the fit batch.
        t_c = time.monotonic()
        c_found, c_rows, timed_out = global_hypotheses(field, full, check, run / 'check.xyz', fit, run / 'fit.xyz',
                                                       run / 'map.xyz', a, out, 'confirm', t0)
        STAGE_S['confirm_total'] = time.monotonic() - t_c
        c_top = rank(c_rows)
        confirmation = {'search_seconds': c_found['seconds'], 'top_ids': [r['id'] for r in c_top], 'agreement': False}
        if c_top:
            Tp = np.asarray(primary['result']['T_map_lidar_refined'])
            Tc = np.asarray(c_top[0]['result']['T_map_lidar_refined'])
            confirmation['agreement'] = agreement(Tp, Tc)
            confirmation['xy_yaw_difference'] = pose_difference(Tp, Tc)
            confirmation['best_id'] = c_top[0]['id']
    timed_out = timed_out or time.monotonic() - t0 >= DEADLINE_S
    if timed_out:
        status = 'timeout'
    elif primary is None:
        status = 'reject_no_hypothesis'
    else:
        status = decide(primary['result'], primary['geometry'], margin,
                        (confirmation['agreement'] and temporal_ok) if confirmation else None)
    children = resource.getrusage(resource.RUSAGE_CHILDREN)
    own = resource.getrusage(resource.RUSAGE_SELF)
    report = {'dataset_run': str(run), 'generator': 'correlative_fft_levelled', 'res_m': a.res,
              'yaw_step_deg': a.yaw_step, 'radius_m': a.radius, 'top': a.top,
              'field_seconds': t_field, 'correlative_seconds': found['seconds'],
              'correlative_score_quantiles': found['score_quantiles_all_peaks'],
              'grid_shape': found['grid_shape'], 'valid_candidates': sum('geometry' in r for r in rows),
              'distinct_top_ids': [r['id'] for r in top],
              'distinct_top': [{'id': r['id'], 'support': r['geometry']['support_015'],
                                'rmse': r['geometry']['matched_rmse_m'], 'tilt_deg': r['tilt_deg'],
                                'correlative_score': r['correlative']['score']} for r in top],
              'primary_id': primary['id'] if primary else None, 'runner_id': runner['id'] if runner else None,
              'support_margin': margin, 'confirmation': confirmation, 'decision': status,
              'accepted': status == 'accepted', 'elapsed_search_s': time.monotonic() - t0,
              'child_cpu_seconds': children.ru_utime + children.ru_stime,
              'self_cpu_seconds': own.ru_utime + own.ru_stime,
              'child_maxrss_kib': children.ru_maxrss, 'orchestrator_maxrss_kib': own.ru_maxrss,
              'fit_check_median_separation_s': separation, 'temporal_evidence_ok': temporal_ok,
              'stage_seconds': dict(STAGE_S), 'deadline_s': DEADLINE_S,
              'truth_used': False, 'all_trials': rows}
    (out / 'search.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'all_trials'}, indent=2))


if __name__ == '__main__':
    main()
