"""Summary of a software-restart campaign table (batch_restarts.sh + f2_check.py): counts with Wilson 95 % bounds, times, memory.
  restart_summary.py table-<tag>.csv RUNS_DIR > summary.json"""
import collections
import csv
import json
import math
import os
import re
import sys

import numpy as np


def wilson(k, n, z=1.96):
    if n == 0:
        return None
    p = k / n; d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d; h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return [round(max(0., c - h), 4), round(min(1., c + h), 4)]


def dist(v):
    v = np.array([float(x) for x in v if x not in ('', None)])
    return {'n': len(v), 'median': round(float(np.median(v)), 2), 'p95': round(float(np.percentile(v, 95)), 2), 'max': round(float(v.max()), 2),
            'min': round(float(v.min()), 2)} if len(v) else {'n': 0}


def main():
    rows = list(csv.DictReader(open(sys.argv[1])))
    runs = sys.argv[2].rstrip('/')
    n = len(rows)
    valid = [r for r in rows if r['verdict'] != 'INVALID']
    mem, lio_fail = [], []
    for r in rows:
        h = f"{runs}/{r['run']}/host.txt"
        m = re.search(r'mem_available_kb_before: (\d+) after: (\d+)', open(h).read()) if os.path.exists(h) else None
        if m:
            mem.append((int(m.group(1)), int(m.group(2))))
        if 'lio_not_ready' in r['observed'] or 'fastlio_exited' in r['observed']:
            lio_fail.append(r['run'])
    by_ds = collections.defaultdict(collections.Counter)
    for r in valid:
        by_ds[r['run'].split('-')[1]][r['observed'].split('/')[0]] += 1
    k = lambda f, rs=valid: sum(1 for r in rs if f(r))
    nv = len(valid)
    out = {
        'kind': 'software_restart (fresh container, node and FAST-LIO2 on a warm board), not a cold boot',
        'runs': n, 'valid': nv, 'invalid': [(r['run'], r['after_teardown'], r['observed'], r['fails']) for r in rows if r['verdict'] == 'INVALID'],
        'verdict': dict(collections.Counter(r['verdict'] for r in rows)),
        'invariant_failures_all_rows': [(r['run'], r['fails']) for r in rows if r['fails']],
        'observed': dict(collections.Counter(r['observed'] for r in valid)),
        'class': dict(collections.Counter(r['class'] for r in valid)),
        'by_dataset': {d: dict(c) for d, c in by_ds.items()},
        'localized': {'k': k(lambda r: r['observed'].startswith('LOCALIZED')), 'n': nv, 'wilson95': wilson(k(lambda r: r['observed'].startswith('LOCALIZED')), nv)},
        'wrong_accept': {'k': k(lambda r: r['class'] == 'wrong_accept'), 'n': nv, 'wilson95': wilson(k(lambda r: r['class'] == 'wrong_accept'), nv)},
        'pose_without_acceptance': k(lambda r: r['class'] == 'pose_without_acceptance'),
        'fastlio_start_failures': {'k': len(lio_fail), 'n': n, 'wilson95': wilson(len(lio_fail), n), 'runs': lio_fail},
        'orphans_nonzero': [r['run'] for r in rows if re.search(r'=[1-9]', r['orphans'])],
        'localized_s': dist(r['localized_s'] for r in valid), 'accept_s': dist(r['accept_s'] for r in valid),
        'attempt_end_s_not_localized': dist(r['attempt_end_s'] for r in valid if not r['observed'].startswith('LOCALIZED')),
        'xy_err_m_accepted': dist(r['xy_err_m'] for r in valid), 'pss_peak_mb': dist(r['pss_peak_mb'] for r in valid),
        'mem_available_mb': {'first_before': mem[0][0] // 1024, 'last_after': mem[-1][1] // 1024, 'min_after': min(b for a, b in mem) // 1024,
                             'slope_mb_per_run': round(float(np.polyfit(range(len(mem)), [b / 1024 for a, b in mem], 1)[0]), 3)} if mem else None,
    }
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
