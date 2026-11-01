"""Rows for the end-to-end table from run directories (metrics.json + evaluation.json + cost_summary.json + witness + tegrastats).
usage: summarize_online.py RUNS_DIR TAG > table.csv"""
import csv, glob, json, os, re, sys

COLS = ['run', 'dataset', 'window', 'scenario', 'variant', 'startup_time', 'imu_time', 'lio_time', 'submap_time', 'submap_build', 'fft_time', 'gicp_time',
        'verification_time', 'search_wall', 'handoff_time', 'total_time_protocol', 'total_from_node_start', 'localized_total', 'cpu_mean_pct', 'cpu_p95_pct', 'rss_peak_mb', 'pss_peak_mb',
        'search_child_maxrss_mb', 'gpu_max_pct', 'drops_imu_pct', 'drops_witness_imu_pct', 'drops_lidar', 'drops_odom', 'state', 'reason', 'accepted', 'classification',
        'xy_err_m', 'yaw_err_deg', 'amcl_xy_median_m', 'amcl_frac_within', 'acq_stop', 'valid']


def f(x, nd=3):
    return '' if x is None else round(x, nd)


def main():
    runs, tag = sys.argv[1], sys.argv[2]
    w = csv.DictWriter(sys.stdout, COLS); w.writeheader()
    for d in sorted(glob.glob(f'{runs}/*-{tag}')):
        mf = sorted(glob.glob(d + '/work/attempt*/metrics.json'))
        if not mf and glob.glob(d + '/work/attempt*/metrics_interrupted.json'):
            w.writerow({'run': os.path.basename(d), 'valid': 'interrupted_bag_ended_or_runner_stop'}); continue
        if not mf or not os.path.exists(d + '/evaluation.json') or os.path.getsize(d + '/evaluation.json') == 0:
            w.writerow({'run': os.path.basename(d), 'valid': 'no_metrics'}); continue
        m, e = json.load(open(mf[-1])), json.load(open(d + '/evaluation.json'))
        name = os.path.basename(d)[:-len(tag) - 1]
        ds, win, mode, variant = name.split('-')
        s, t = m['stage_s'], m['total_s']
        cost = json.load(open(d + '/cost_summary.json')) if os.path.exists(d + '/cost_summary.json') else None
        wit = json.load(open(d + '/witness_imu.json')) if os.path.exists(d + '/witness_imu.json') else None
        gpu = [int(x) for x in re.findall(r'GR3D_FREQ (\d+)%', open(d + '/tegrastats.log').read())] if os.path.exists(d + '/tegrastats.log') else []
        fft = (s['fft_primary_s'] or 0) + (s['fft_confirm_s'] or 0) + (s['map_field_s'] or 0) if s['fft_primary_s'] is not None else None
        search = m.get('search') or {}
        # verification = everything the worker does that is neither the FFT search nor GICP (scoring, ranking, decision, process start/IO)
        ver = (s['search_worker_wall_s'] - fft - (s['gicp_s'] or 0)) if fft is not None and s['search_worker_wall_s'] is not None else None
        imu = m['streams']['imu']
        mem = (search.get('child_maxrss_kib') or 0) / 1024
        w.writerow({'run': os.path.basename(d), 'dataset': ds, 'window': win, 'scenario': mode, 'variant': variant,
                    'startup_time': f(s['startup_to_first_input_s']), 'imu_time': f(s['imu_time_s']), 'lio_time': f(s['lio_time_s']), 'submap_time': f(s['submap_exposure_s']),
                    'submap_build': f(s['submap_build_s']), 'fft_time': f(fft), 'gicp_time': f(s['gicp_s']), 'verification_time': f(ver), 'search_wall': f(s['search_worker_wall_s']),
                    'handoff_time': f(s['handoff_confirm_s']), 'total_time_protocol': f(t['protocol_accept_from_first_input']), 'total_from_node_start': f(t['accept_from_node_start']),
                    'localized_total': f(t['localized_from_first_input']),
                    'cpu_mean_pct': f(cost and cost['total_excl_bag_play']['cpu_pct']['mean'], 1), 'cpu_p95_pct': f(cost and cost['total_excl_bag_play']['cpu_pct']['p95'], 1),
                    'rss_peak_mb': f(cost and sum(v['rss_peak_mb'] for k, v in cost['per_process'].items() if k != 'bag_play'), 1),
                    'pss_peak_mb': f(cost and cost['total_excl_bag_play']['pss_peak_mb'], 1), 'search_child_maxrss_mb': f(mem, 1),
                    'gpu_max_pct': max(gpu) if gpu else '', 'drops_imu_pct': f(100 * imu['lost_estimate'] / max(1, imu['received'] + imu['lost_estimate']), 2),
                    'drops_witness_imu_pct': f(wit and 100 * wit['loss_fraction'], 2), 'drops_lidar': m['streams']['lidar']['lost_estimate'], 'drops_odom': m['streams']['odom']['lost_estimate'],
                    'state': m['state'], 'reason': m['reason'], 'accepted': m['accepted'], 'classification': e['class'],
                    'xy_err_m': f(e.get('initialpose_error', {}).get('xy_m'), 4), 'yaw_err_deg': f(e.get('initialpose_error', {}).get('yaw_deg'), 3),
                    'amcl_xy_median_m': f(e.get('amcl', {}).get('xy_median_m'), 3), 'amcl_frac_within': f(e.get('amcl', {}).get('frac_within_limits'), 2),
                    'acq_stop': m['acquisition']['stop_reason'], 'valid': 'yes'})


if __name__ == '__main__':
    main()
