"""Distributions from the end-to-end CSV (summarize_online.py): outcome counts with Wilson 95 % bounds, latency and resource percentiles.
usage: analyze_online.py table.csv [table2.csv ...]  -> markdown on stdout. N is printed with every figure."""
import csv
import math
import sys
from collections import defaultdict


def wilson(k, n, z=1.96):
    if n == 0:
        return (float('nan'),) * 2
    p = k / n
    d = 1 + z * z / n
    c = p + z * z / (2 * n)
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return (c - h) / d, (c + h) / d


def pct(v, q):
    v = sorted(v)
    if not v:
        return float('nan')
    i = (len(v) - 1) * q / 100
    lo, hi = int(math.floor(i)), int(math.ceil(i))
    return v[lo] + (v[hi] - v[lo]) * (i - lo)


def stats(v):
    return f'{pct(v, 50):.2f} / {pct(v, 95):.2f} / {max(v):.2f} (N={len(v)})' if v else 'n/a'


def main():
    rows = [r for p in sys.argv[1:] for r in csv.DictReader(open(p)) if r.get('valid') == 'yes']
    groups = defaultdict(list)
    for r in rows:
        groups[(r['scenario'], r['variant'])].append(r)
    print('| scenario | variant | N | correct_accept | wrong_accept | false_reject | correct_reject | error_no_pose | unscorable | wrong_accept 95 % upper (Wilson) |\n|---|---|---|---|---|---|---|---|---|---|')
    for (sc, va), g in sorted(groups.items()):
        c = lambda k: sum(r['classification'] == k for r in g)
        fr = sum(r['classification'].startswith('false_reject') for r in g)
        n_scored = len(g) - c('unscorable_reference_gap')
        w = wilson(c('wrong_accept'), n_scored)[1]
        print(f"| {sc} | {va} | {len(g)} | {c('correct_accept')} | {c('wrong_accept')} | {fr} | {c('correct_reject')} | {c('error_no_pose')} | {c('unscorable_reference_gap')} | {w:.2f} |")
    pos = [r for r in rows if r['variant'] == 'own' and r['classification'] == 'correct_accept' and r['scenario'] == 's3']
    f = lambda k: [float(r[k]) for r in pos if r[k] != '']
    print(f'\nLatency of correct accepts, S3 positives (median / p95 / max seconds), N={len(pos)}:\n')
    print('| quantity | median / p95 / max |\n|---|---|')
    for k in ('imu_time', 'lio_time', 'submap_time', 'submap_build', 'fft_time', 'gicp_time', 'verification_time', 'search_wall', 'handoff_time', 'total_time_protocol', 'localized_total',
              'cpu_mean_pct', 'cpu_p95_pct', 'rss_peak_mb', 'pss_peak_mb', 'search_child_maxrss_mb', 'drops_imu_pct', 'drops_witness_imu_pct', 'xy_err_m', 'yaw_err_deg', 'amcl_xy_median_m'):
        print(f'| {k} | {stats(f(k))} |')


if __name__ == '__main__':
    main()
