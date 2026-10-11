"""Evaluator side: one row per run of a latency batch (permit_exposure.py lines joined with the f2_check.py table and the node's
supervision.json), then the distribution of latency_s over the runs whose permit was ever withdrawn or revoked.
  latency_summary.py exposure-<tag>.jsonl table-<tag>.csv RUNS_DIR > latency-<tag>.csv      (distribution on stderr)"""
import csv
import glob
import json
import sys

import numpy as np

COLS = ['run', 'observed', 'verdict', 'gate_open_s', 'out_at_s', 'close_at_s', 'latency_s', 'exposure_max_xy_m', 'exposure_max_yaw_deg',
        'first_failed_check_s', 'checks', 'checks_failed', 'check_wall_s_median', 'max_pass_age_s', 'withheld_in_doubt', 'withheld_stale',
        'worker_cpu_pct_of_core', 'min_support_passed', 'min_tail_support_passed']


def main():
    expo = {json.loads(l)['run']: json.loads(l) for l in open(sys.argv[1]) if l.strip()}
    table = {r['run']: r for r in csv.DictReader(open(sys.argv[2]))}
    w = csv.DictWriter(sys.stdout, COLS)
    w.writeheader()
    lat = []
    for run, e in expo.items():
        t = table.get(run, {})
        sup = sorted(glob.glob(f'{sys.argv[3].rstrip("/")}/{run}/work/attempt*/supervision.json'))
        s = json.load(open(sup[-1])) if sup else {}
        ms, permit = s.get('map_supervision') or {}, s.get('permit') or {}
        cpu = 100 * ms['worker_cpu_s'] / s['held_s'] if ms.get('worker_cpu_s') and s.get('held_s') else None
        w.writerow({'run': run, 'observed': t.get('observed'), 'verdict': t.get('verdict'), 'gate_open_s': e.get('open_s'), 'out_at_s': e.get('out_at_s'),
                    'close_at_s': e.get('close_at_s'), 'latency_s': e.get('latency_s'), 'exposure_max_xy_m': e.get('exposure_max_xy_m'),
                    'exposure_max_yaw_deg': e.get('exposure_max_yaw_deg'), 'first_failed_check_s': e.get('first_failed_check_s'), 'checks': e.get('checks'),
                    'checks_failed': e.get('checks_failed'), 'check_wall_s_median': e.get('check_wall_s_median'),
                    'max_pass_age_s': ms.get('max_pass_age_s') and round(ms['max_pass_age_s'], 2), 'withheld_in_doubt': permit.get('withheld_in_doubt'),
                    'withheld_stale': permit.get('withheld_stale'), 'worker_cpu_pct_of_core': cpu and round(cpu, 1),
                    'min_support_passed': ms.get('min_support') and round(ms['min_support'], 3),
                    'min_tail_support_passed': ms.get('min_tail_support') and round(ms['min_tail_support'], 3)})
        if 'LOCALIZATION_LOST' in (t.get('observed') or '') and e.get('open_s'):
            lat.append(e.get('latency_s') or 0.)
    if lat:
        v = np.array(lat)
        print(json.dumps({'revoked_runs_with_open_gate': len(v), 'latency_s': {'median': round(float(np.median(v)), 2), 'p95': round(float(np.percentile(v, 95)), 2),
                          'max': round(float(v.max()), 2), 'zero': int((v == 0).sum())}, 'sorted': sorted(round(float(x), 2) for x in v)}), file=sys.stderr)


if __name__ == '__main__':
    main()
