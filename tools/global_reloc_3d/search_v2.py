"""Truth-free global search v2: correlative (x, y, z, yaw) candidates, GICP, same gate.

Requires a gravity-levelled source (lio_submap.assemble with an IMU). The acceptance rule
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
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np
from scipy.spatial import cKDTree

from correlative_search import MIN_SOURCE_POINTS, MapField, SparseSource, crop_source, search, z_cell_range
from lio_submap import FIT_FRACTION
from verify import decide, score

DEADLINE_S = 60.
STAGE_S = {'gicp': 0., 'score': 0., 'confirm_total': 0.}  # additive instrumentation, no effect on decisions
STAGE_LOCK = threading.Lock()                               # the primary and the confirmation searches add to it at the same time


# Policy over several hypotheses: when two poses are one place, and whether the exposure spans enough time to confirm.
def yaw(T):
    return math.atan2(T[1, 0], T[0, 0])


def pose_difference(a, b):
    xy = np.linalg.norm(a[:2, 3]-b[:2, 3])
    d = yaw(a)-yaw(b)
    return float(xy), abs(math.degrees(math.atan2(math.sin(d), math.cos(d))))


def distinct(a, b):
    xy, deg = pose_difference(a, b)
    return xy > .30 or deg > 8.


def agreement(a, b):
    xy, deg = pose_difference(a, b)
    return xy <= .20 and deg <= 5.


def temporal_evidence(stamps, start, duration):
    fit=[t for t in stamps if t < start+duration*FIT_FRACTION]
    check=[t for t in stamps if t >= start+duration*FIT_FRACTION]
    if not fit or not check:
        return False, None
    separation=float(np.median(check)-np.median(fit))
    return separation >= .5 and check[-1]-fit[0] >= 1., separation


def run_concurrently(f, g):
    """(f(), g()) with both running at the same time; the first error is raised (after both ended)."""
    with ThreadPoolExecutor(2) as ex:
        a, b = ex.submit(f), ex.submit(g)
        return a.result(), b.result()


def tilt_deg(T):
    """Angle between the pose's z axis and map +z (both frames gravity-levelled)."""
    return float(math.degrees(math.acos(np.clip(np.asarray(T)[2, 2], -1., 1.))))


def run_register(binary, args, timeout_s, threads):
    """register_v2 with `args`; (log, timed_out). On a timeout the result files already written stay."""
    try:
        proc = subprocess.run([str(binary), *map(str, args)], capture_output=True, text=True, timeout=max(.001, timeout_s),
                              env={**os.environ, 'OMP_NUM_THREADS': str(threads)})
        return {'exit_code': proc.returncode, 'stdout': proc.stdout[-500:], 'stderr': proc.stderr[-500:]}, False
    except subprocess.TimeoutExpired:
        return {'exit_code': 124, 'stderr': 'matcher_deadline_exceeded'}, True


def refine_all(binary, map_path, source_path, check_path, T_inits, out, prefix, voxel, timeout_s, threads=2):
    """GICP from every T_init in one register_v2 process (the map and the source are preprocessed once).
    Result i ends in out/<prefix>_NN/result.json, as if each init had its own run. (log, results, timed_out):
    results[i] is None when that init has no result (it failed, or the deadline came first)."""
    work = out / f'{prefix}_batch'
    work.mkdir()
    inits = work / 'inits.txt'
    inits.write_text(''.join(' '.join(f'{v:.18e}' for v in np.asarray(T).ravel()) + '\n' for T in T_inits))
    log, timed_out = run_register(binary, [map_path, source_path, check_path, voxel, work, '--init-list', inits],
                                  timeout_s, threads)
    results = []
    for i in range(len(T_inits)):
        done = work / f'{i:02d}.json'
        if done.exists():
            d = out / f'{prefix}_{i:02d}'
            d.mkdir()
            done.rename(d / 'result.json')
            results.append(json.loads((d / 'result.json').read_text()))
        else:
            results.append(None)
    if all(r is None for r in results) and 'unknown option' in log['stderr']:    # an old binary without --init-list; any other failure with no result is "no hypothesis", as before
        raise RuntimeError(f"register_v2 does not know --init-list (stage the binary with the tools): {log['stderr'].strip()}")
    return log, results, timed_out


MAP_TREE = {}


def map_tree(map_points):
    """One KD-tree per map array for all the candidates of both searches."""
    with STAGE_LOCK:
        if id(map_points) not in MAP_TREE:
            MAP_TREE[id(map_points)] = cKDTree(map_points)
        return MAP_TREE[id(map_points)]


def global_hypotheses(field, map_points, batch, batch_path, other, other_path, map_path, args, out, prefix, t0):
    """Correlative search on one batch, GICP refine, score on the other batch."""
    found = search(map_points, batch, args.res, yaw_step_deg=args.yaw_step, radius=args.radius,
                   top=args.top, field=field)       # the z band is already in the field
    candidates = found['candidates']
    if not candidates:
        return found, [], False
    if DEADLINE_S - (time.monotonic() - t0) <= 0:
        return found, [], True
    t_g = time.monotonic()
    log, results, timed_out = refine_all(args.binary, map_path, batch_path, other_path, [c['T_map_source'] for c in candidates],
                                         out, prefix, args.gicp_voxel, DEADLINE_S - (time.monotonic() - t0), args.threads)
    with STAGE_LOCK:
        STAGE_S['gicp'] += time.monotonic() - t_g
    rows = []
    late = timed_out        # past the deadline the finished candidates are kept, unscored; the search ends as a timeout
    for i, (c, result) in enumerate(zip(candidates, results)):
        late = late or DEADLINE_S - (time.monotonic() - t0) <= 0
        if late and result is None:
            continue
        row = {'id': f'{prefix}_{i:02d}', 'correlative': {k: c[k] for k in ('score', 'yaw_rad', 'xyz')}, 'run': log}
        if result is not None:
            # The generator replaces KISS; keep verify.decide()'s interface.
            result['kiss_valid'] = True
            row['result'] = result
            if result['gicp_converged'] and not late:
                t_s = time.monotonic()
                row['geometry'] = score(map_points, other, result['T_map_lidar_refined'], tree=map_tree(map_points))
                with STAGE_LOCK:
                    STAGE_S['score'] += time.monotonic() - t_s
                row['tilt_deg'] = tilt_deg(result['T_map_lidar_refined'])
                xy, deg = pose_difference(np.asarray(c['T_map_source']), np.asarray(result['T_map_lidar_refined']))
                row['gicp_shift'] = {'xy_m': xy, 'yaw_deg': deg}
        rows.append(row)
    return found, rows, late


def source_reach_z(fit, check, radius):
    """Largest |z| of the two source batches inside the search radius; 0 when a batch has no point there (a near-empty frame), so the search rejects cleanly."""
    zs = [np.abs(crop_source(s, radius)[:, 2]) for s in (fit, check)]
    return max((float(z.max()) for z in zs if len(z)), default=0.)


def in_band(row, z_band, margin=.25):
    """The refined pose keeps the sensor within the band (+- margin): a GICP that drifts out of the heights the robot can have is not a hypothesis."""
    return z_band is None or z_band[0] - margin <= row['result']['T_map_lidar_refined'][2][3] <= z_band[1] + margin


def rank(rows, z_band=None):
    valid = [r for r in rows if 'geometry' in r and r['geometry']['valid'] and in_band(r, z_band)]
    valid.sort(key=lambda r: (-r['geometry']['support_015'], 9. if r['geometry']['matched_rmse_m'] is None else r['geometry']['matched_rmse_m']))
    top = []
    for r in valid:
        T = np.asarray(r['result']['T_map_lidar_refined'])
        if all(distinct(T, np.asarray(o['result']['T_map_lidar_refined'])) for o in top):
            top.append(r)
        if len(top) == 5:
            break
    return top


def positive_int(s):
    n = int(s)
    if n < 1:
        raise argparse.ArgumentTypeError('must be >= 1')
    return n


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run', required=True, help='prepared run with map/fit/check.xyz, levelled source')
    p.add_argument('--out', required=True, help='new versioned output directory')
    p.add_argument('--binary', required=True, help='register_v2 (GICP-only --init-list mode)')
    p.add_argument('--res', type=float, default=.5)
    p.add_argument('--yaw-step', type=float, default=2.5)
    p.add_argument('--radius', type=float, default=15.)
    p.add_argument('--top', type=int, default=10)
    p.add_argument('--fft-workers', type=positive_int, default=1, help='FFT threads of each search (1: the primary and the confirmation search run side by side; 2 costs ~40 %% more CPU on a laptop for no gain in wall time, to be measured on a board)')
    p.add_argument('--gicp-voxel', type=float, default=.3)
    p.add_argument('--deadline-s', type=float, default=60., help='remaining decision budget from this process start')
    p.add_argument('--z-band', type=float, nargs=2, metavar=('LO', 'HI'), help='heights (m, map frame) the sensor can have: the search and the refined poses stay inside it (+- 0.25 m for the latter)')
    p.add_argument('--corr-backend', choices=('python', 'cpp'), default='cpp', help='the correlative search: cpp_search/libcorr_search.so (build.sh; no fallback when it is missing), or numpy/scipy: the same operations, slower')
    p.add_argument('--threads', type=int, default=2, help='OMP_NUM_THREADS of each register_v2 call (small_gicp is built without OpenMP, so GICP runs on one thread whatever this says); the FFT threads are --fft-workers')
    a = p.parse_args()
    global DEADLINE_S
    DEADLINE_S = a.deadline_s
    run, out = Path(a.run), Path(a.out)
    out.mkdir(parents=True, exist_ok=False)
    t0 = time.monotonic()
    manifest = json.loads((run / 'manifest.json').read_text())
    if manifest['frame_fit'] != 'local_first_imu_levelled':
        raise ValueError('search_v2 needs a gravity-levelled source (built by lio_submap.assemble from an IMU)')
    fit = np.loadtxt(run / 'fit.xyz').reshape(-1, 3)      # reshape: one point or none load as 1-D
    check = np.loadtxt(run / 'check.xyz').reshape(-1, 3)
    if min(len(crop_source(s, a.radius)) for s in (fit, check)) < MIN_SOURCE_POINTS:      # before the map and the field: this exit is retried, on a shared board
        raise SparseSource('source has too few points inside the search radius')
    full = np.loadtxt(run / 'map.xyz')
    reach_z = source_reach_z(fit, check, a.radius)
    z_band = tuple(a.z_band) if a.z_band else None
    if a.corr_backend == 'cpp':
        from corr_cpp import CppMapField as Field
    else:
        Field = MapField
    field = Field(full, a.res, .5, a.radius + a.res, reach_z + a.res, workers=a.fft_workers, z_band=z_band,
                     z_cells=z_cell_range([fit, check], a.res, a.radius))
    t_field = time.monotonic() - t0
    temporal_ok, separation = temporal_evidence(manifest['stamps_s'], manifest['start_header_s'], manifest['duration_s'])
    def confirmation_search():
        # independent global search of the later batch; its best is chosen on the fit batch. It does not depend on the primary search, so both run at the same time
        t_c = time.monotonic()
        r = global_hypotheses(field, full, check, run / 'check.xyz', fit, run / 'fit.xyz', run / 'map.xyz', a, out, 'confirm', t0)
        with STAGE_LOCK:
            STAGE_S['confirm_total'] = time.monotonic() - t_c
        return r
    (found, rows, timed_out), (c_found, c_rows, c_timed_out) = run_concurrently(
        lambda: global_hypotheses(field, full, fit, run / 'fit.xyz', check, run / 'check.xyz', run / 'map.xyz', a, out, 'cand', t0), confirmation_search)
    top = rank(rows, z_band)
    primary = top[0] if top else None
    runner = top[1] if len(top) > 1 else None
    margin = primary['geometry']['support_015'] - runner['geometry']['support_015'] if runner else None
    confirmation = None
    if primary and not timed_out and time.monotonic() - t0 < DEADLINE_S:
        timed_out = c_timed_out
        c_top = rank(c_rows, z_band)
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
              'score_quantiles_unskipped_peaks': found['score_quantiles_unskipped_peaks'],
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
    try:
        main()
    except (SparseSource, MemoryError) as e:      # this scene or this moment: the supervisor tries again. Any other exception is a bug or bad input (exit 1): it does not
        print(f'search_v2: {e!r}', file=sys.stderr)
        sys.exit(3)
