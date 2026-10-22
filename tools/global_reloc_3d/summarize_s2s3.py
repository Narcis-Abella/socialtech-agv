"""Table + distributions for mode s2s3 runs (metrics.json + evaluation.json + cost_summary.json + witness + tegrastats).
usage: summarize_s2s3.py RUNS_DIR TAG[,TAG...] OUT.csv   (prints the aggregate as markdown)"""
import csv, glob, json, os, re, sys

import numpy as np


def load(path):
    return json.load(open(path)) if os.path.exists(path) and os.path.getsize(path) else None


def row(d, tag):
    mf = sorted(glob.glob(d + '/work/attempt*/metrics.json'))
    e = load(d + '/evaluation.json')
    if not mf:
        return {'run': os.path.basename(d), 'valid': 'no_metrics'}
    m = json.load(open(mf[-1]))
    ds, win, token, variant = os.path.basename(d)[:-len(tag) - 1].split('-')
    rel = m['marks_rel_first_input_s']
    cost, wit = load(d + '/cost_summary.json'), load(d + '/witness_imu.json')
    gpu = [int(x) for x in re.findall(r'GR3D_FREQ (\d+)%', open(d + '/tegrastats.log').read())] if os.path.exists(d + '/tegrastats.log') else []
    temp = [float(x) for x in re.findall(r'cpu@([\d.]+)C', open(d + '/tegrastats.log').read())] if os.path.exists(d + '/tegrastats.log') else []
    s2 = (m.get('stages') or {}).get('s2') or {}
    s2s, st, tot, acq = m.get('s2_stage_s') or {}, m['stage_s'], m['total_s'], m['acquisition']
    imu = m['streams']['imu']
    r3 = lambda x: '' if x is None else round(x, 3)
    per = (cost or {}).get('per_process', {})
    return {
        'run': os.path.basename(d), 'dataset': ds, 'window': win, 'start': 'stationary_episode' if win.startswith('z') else 'motion_follows',
        'config': token, 'variant': variant, 'attempts': len(mf), 'accepted': m['accepted'],
        'class': e['class'] if e else '', 'state': m['state'], 'reason': m['reason'] or '', 'accepted_stage': m.get('accepted_stage') or '',
        's2_decision': s2.get('decision', 'accepted' if m.get('accepted_stage') == 's2' else ''), 's2_clouds': acq.get('s2_clouds'),
        's2_odom_radius_m': r3(acq.get('s2_odom_radius_m')), 'imu_gate_s': r3(st['imu_time_s']), 'lio_ready_s': r3(st['lio_time_s']),
        's2_exposure_s': r3(s2s.get('exposure_s')), 's2_build_s': r3(s2s.get('build_s')), 's2_search_s': r3(s2s.get('search_worker_wall_s')),
        's2_decided_at_s': r3(s2s.get('decided_at_s')), 's3_requested_at_s': r3(m['s3']['requested_at_s']) if m.get('s3') else '',
        's3_authorized_at_s': r3(m['s3']['authorized_at_s']) if m.get('s3') else '', 's3_authorized_by': (m.get('s3') or {}).get('authorized_by') or '',
        's3_acq_end_at_s': r3(rel.get('s3:acq_end')), 's3_build_s': r3(st['submap_build_s']) if m.get('last_stage') == 's3' else '',
        's3_search_s': r3(st['search_worker_wall_s']) if m.get('last_stage') == 's3' else '',
        'fft_s': r3(sum(st[k] or 0 for k in ('fft_primary_s', 'fft_confirm_s', 'map_field_s'))) if st['fft_primary_s'] is not None else '',
        'gicp_s': r3(st['gicp_s']), 'accept_at_s': r3(tot['protocol_accept_from_first_input']), 'handoff_confirm_s': r3(st['handoff_confirm_s']),
        'localized_at_s': r3(tot['localized_from_first_input']), 'attempt_end_at_s': r3(tot['attempt_end_from_first_input']),
        'node_start_to_first_input_s': r3(st['startup_to_first_input_s']), 'pose_published': m['initialpose'] is not None,
        'xy_err_m': r3((e or {}).get('initialpose_error', {}).get('xy_m')), 'yaw_err_deg': r3((e or {}).get('initialpose_error', {}).get('yaw_deg')),
        'amcl_xy_median_m': r3((e or {}).get('amcl', {}).get('xy_median_m')), 'acq_stop': acq['stop_reason'] or '', 'clouds': acq['clouds'],
        'odom_max_radius_m': r3(acq.get('odom_max_radius_m')), 'odom_path_m': r3(acq.get('odom_path_m')),
        'supervisor_stop_signalled': (m.get('s3') or {}).get('supervisor_stop_signalled', ''),
        'cpu_mean_pct': r3(cost and cost['total_excl_bag_play']['cpu_pct']['mean']), 'cpu_p95_pct': r3(cost and cost['total_excl_bag_play']['cpu_pct']['p95']),
        'cpu_max_pct': r3(cost and cost['total_excl_bag_play']['cpu_pct'].get('max')),
        'pss_peak_mb': r3(cost and cost['total_excl_bag_play']['pss_peak_mb']),
        'rss_peak_mb': r3(cost and sum(v['rss_peak_mb'] for k, v in per.items() if k != 'bag_play')),
        'worker_maxrss_mb': r3(((m.get('search') or {}).get('child_maxrss_kib') or 0) / 1024), 'gpu_max_pct': max(gpu) if gpu else '',
        'cpu_temp_max_c': max(temp) if temp else '',
        'imu_rx': imu['received'], 'imu_lost_pct': r3(100 * imu['lost_estimate'] / max(1, imu['received'] + imu['lost_estimate'])),
        'imu_rate_hz': r3(imu['rate_hz_wall']), 'witness_imu_lost_pct': r3(wit and 100 * wit['loss_fraction']),
        'lidar_rx': m['streams']['lidar']['received'], 'lidar_lost': m['streams']['lidar']['lost_estimate'],
        'odom_rx': m['streams']['odom']['received'], 'odom_lost': m['streams']['odom']['lost_estimate'],
        'cloud_rx': m['streams']['cloud']['received'], 'cloud_lost': m['streams']['cloud']['lost_estimate'], 'valid': 'yes'}


def dist(v):
    v = np.array([float(x) for x in v if x not in ('', None)])
    return 'NA (N=0)' if not len(v) else f'{np.median(v):.1f} / {np.percentile(v, 95):.1f} / {v.max():.1f} (N={len(v)})'


def main():
    runs, tags, out = sys.argv[1], sys.argv[2].split(','), sys.argv[3]
    rows = [row(d, t) for t in tags for d in sorted(glob.glob(f'{runs}/*-{t}'))]
    cols = max(rows, key=len).keys()
    with open(out, 'w', newline='') as f:
        w = csv.DictWriter(f, cols); w.writeheader(); w.writerows(rows)
    ok = [r for r in rows if r.get('valid') == 'yes']
    groups = {}
    for r in ok:
        groups.setdefault((r['config'], r['start'], r['variant']), []).append(r)
    print('| config | start | variant | N | classes | accepted in S2 / S3 | accept at s (med/p95/max) | LOCALIZED at s | attempt end at s | xy err m (max) | yaw err deg (max) |')
    print('|---|---|---|---|---|---|---|---|---|---|---|')
    for (cfg, start, var), g in sorted(groups.items()):
        cl = {}
        for r in g:
            cl[r['class'] or r['state']] = cl.get(r['class'] or r['state'], 0) + 1
        xy = [float(r['xy_err_m']) for r in g if r['xy_err_m'] != '']; yw = [float(r['yaw_err_deg']) for r in g if r['yaw_err_deg'] != '']
        print(f"| {cfg} | {start} | {var} | {len(g)} | {', '.join(f'{k} {v}' for k, v in sorted(cl.items()))} | {sum(r['accepted_stage'] == 's2' for r in g)} / {sum(r['accepted_stage'] == 's3' for r in g)} "
              f"| {dist(r['accept_at_s'] for r in g)} | {dist(r['localized_at_s'] for r in g)} | {dist(r['attempt_end_at_s'] for r in g)} "
              f"| {max(xy) if xy else 'NA'} | {max(yw) if yw else 'NA'} |")
    print()
    for k in ('imu_gate_s', 'lio_ready_s', 's2_exposure_s', 's2_build_s', 's2_search_s', 's2_decided_at_s', 's3_acq_end_at_s', 's3_build_s', 's3_search_s', 'handoff_confirm_s',
              'cpu_mean_pct', 'cpu_p95_pct', 'cpu_max_pct', 'pss_peak_mb', 'rss_peak_mb', 'worker_maxrss_mb', 'gpu_max_pct', 'cpu_temp_max_c', 'imu_lost_pct', 'witness_imu_lost_pct', 'imu_rate_hz'):
        print(f'{k}: {dist(r.get(k) for r in ok)}')
    print('pose published without acceptance:', sum(1 for r in ok if r['pose_published'] and not r['accepted']))


if __name__ == '__main__':
    main()
