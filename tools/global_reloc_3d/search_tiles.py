"""Truth-free exhaustive fixed-grid KISS search, GICP refinement and S2 check.

Each tile is a prior-map-only region. A registration is global within that tile:
KISS receives no initial transform. No query pose is used to choose tiles.
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

from verify import decide, score


def yaw(T):
    return math.atan2(T[1, 0], T[0, 0])


def candidate_key(r):
    """Best first: support, then the rmse of the matched points (None = nothing matched, sorts last as in search_v2.rank), then the inliers."""
    rmse = r['geometry']['matched_rmse_m']
    return (-r['geometry']['support_015'], 9. if rmse is None else rmse, -r['result']['translation_inliers'])


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
    fit=[t for t in stamps if t < start+duration*2/3]
    check=[t for t in stamps if t >= start+duration*2/3]
    if not fit or not check:
        return False, None
    separation=float(np.median(check)-np.median(fit))
    return separation >= .5 and check[-1]-fit[0] >= 1., separation


def run_one(binary, map_path, source_path, check_path, size, out, timeout_s):
    try:
        proc = subprocess.run([str(binary), str(map_path), str(source_path), str(check_path), str(size), str(out)],
                              capture_output=True, text=True, timeout=max(.001,timeout_s),
                              env={**os.environ, 'OMP_NUM_THREADS': '2'})
        return {'exit_code': proc.returncode, 'stdout': proc.stdout[-2000:], 'stderr': proc.stderr[-2000:]}
    except subprocess.TimeoutExpired:
        return {'exit_code': 124, 'stdout': '', 'stderr': 'matcher_deadline_exceeded'}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run', required=True, help='prepared run containing map/fit/check.xyz')
    p.add_argument('--out', required=True, help='new versioned output directory')
    p.add_argument('--binary', required=True)
    p.add_argument('--tile', type=float, default=15.)
    p.add_argument('--stride', type=float, default=10.)
    p.add_argument('--kiss-voxel', type=float, default=.3)
    a = p.parse_args()
    run, out = Path(a.run), Path(a.out)
    out.mkdir(parents=True, exist_ok=False)
    t0 = time.monotonic()
    full = np.loadtxt(run/'map.xyz')
    heldout = np.loadtxt(run/'check.xyz')
    manifest = json.loads((run/'manifest.json').read_text())
    temporal_evidence_ok, batch_separation=temporal_evidence(
        manifest['stamps_s'],manifest['start_header_s'],manifest['duration_s'])
    tiles = [('full', full)]
    xmin, ymin = np.floor(full[:, :2].min(axis=0))
    xmax, ymax = np.ceil(full[:, :2].max(axis=0))
    for xi, x in enumerate(np.arange(xmin, xmax, a.stride)):
        for yi, y in enumerate(np.arange(ymin, ymax, a.stride)):
            subset = full[(full[:,0]>=x)&(full[:,0]<x+a.tile)&(full[:,1]>=y)&(full[:,1]<y+a.tile)]
            if len(subset) >= 500:
                tiles.append((f'tile_{xi}_{yi}', subset))
    trials=[]
    timed_out=False
    for name, points in tiles:
        if time.monotonic()-t0 >= 60.:
            timed_out=True
            break
        d = out/name
        d.mkdir()
        target = d/'target.xyz'
        np.savetxt(target, points, fmt='%.6f')
        runlog = run_one(a.binary, target, run/'fit.xyz', run/'check.xyz', a.kiss_voxel, d/'result.json', 60.-(time.monotonic()-t0))
        if runlog['exit_code']==124:
            timed_out=True
        row={'id': name, 'target_points': len(points), 'run': runlog}
        if runlog['exit_code']!=124 and (d/'result.json').exists():
            result=json.loads((d/'result.json').read_text())
            row['result']=result
            if result['kiss_valid'] and result['gicp_converged']:
                row['geometry']=score(full, heldout, result['T_map_lidar_refined'])
        trials.append(row)
        if timed_out:
            break
    candidates=[r for r in trials if 'geometry' in r]
    candidates.sort(key=candidate_key)
    distinct_top=[]
    for row in candidates:
        if all(distinct(np.array(row['result']['T_map_lidar_refined']), np.array(other['result']['T_map_lidar_refined'])) for other in distinct_top):
            distinct_top.append(row)
        if len(distinct_top)==5:
            break
    primary=distinct_top[0] if distinct_top else None
    runner=distinct_top[1] if len(distinct_top)>1 else None
    margin=(primary['geometry']['support_015']-runner['geometry']['support_015']) if runner else None
    # Independent later-batch global run, separated in the archived sensor stamps.
    confirmation=None
    if primary and not timed_out and time.monotonic()-t0 < 60.:
        confirm_log=run_one(a.binary, run/'map.xyz', run/'check.xyz', run/'fit.xyz', a.kiss_voxel,
                            out/'confirm_result.json',60.-(time.monotonic()-t0))
        if confirm_log['exit_code']==124:
            timed_out=True
        confirmation={'run': confirm_log}
        if confirm_log['exit_code']!=124 and (out/'confirm_result.json').exists():
            c=json.loads((out/'confirm_result.json').read_text())
            confirmation['result']=c
            confirmation['agreement']=c['kiss_valid'] and c['gicp_converged'] and agreement(np.array(primary['result']['T_map_lidar_refined']), np.array(c['T_map_lidar_refined']))
            confirmation['xy_yaw_difference']=pose_difference(np.array(primary['result']['T_map_lidar_refined']), np.array(c['T_map_lidar_refined']))
        else:
            confirmation['agreement']=False
    # Full-map and tiled candidates are all searched independently of truth.
    timed_out = timed_out or time.monotonic()-t0 >= 60.
    status='timeout' if timed_out else (decide(primary['result'], primary['geometry'], margin,
                  confirmation['agreement'] and temporal_evidence_ok if confirmation else None) if primary else 'reject_no_hypothesis')
    children=resource.getrusage(resource.RUSAGE_CHILDREN)
    own=resource.getrusage(resource.RUSAGE_SELF)
    report={'dataset_run': str(run), 'tile_m': a.tile, 'stride_m': a.stride,
            'kiss_voxel_m': a.kiss_voxel, 'planned_targets': len(tiles), 'searched_targets': len(trials),
            'valid_candidates': len(candidates), 'distinct_top_ids': [r['id'] for r in distinct_top],
            'primary_id': primary['id'] if primary else None,
            'runner_id': runner['id'] if runner else None, 'support_margin': margin,
            'confirmation': confirmation, 'decision': status, 'accepted': status=='accepted',
            'elapsed_search_s': time.monotonic()-t0,
            'child_cpu_seconds': children.ru_utime+children.ru_stime,
            'child_maxrss_kib': children.ru_maxrss,
            'orchestrator_maxrss_kib': own.ru_maxrss,
            'first_fit_stamp_s': manifest['stamps_s'][0],
            'last_check_stamp_s': manifest['stamps_s'][-1],
            'fit_check_median_separation_s': batch_separation,
            'temporal_evidence_ok': temporal_evidence_ok,
            'truth_used': False, 'all_trials': trials}
    (out/'search.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('all_trials','confirmation')}, indent=2))
    print(json.dumps({'confirmation': confirmation}, indent=2))


if __name__=='__main__':
    main()
