"""Summarize a batch: one row per run, plus Wilson 95 % bounds. Evaluator-side."""
import argparse
import csv
import glob
import json
import math
from pathlib import Path


def wilson(k, n, z=1.96):
    if n == 0:
        return (0., 1.)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0., c - h), min(1., c + h))


def classify(variant, search, evaluation):
    accepted = search['accepted']
    if search['decision'] == 'timeout':
        return 'timeout'
    if variant == 'cross':
        return 'wrong_accept' if accepted else 'correct_reject'
    if variant == 'hole' and not accepted:
        return 'reject'
    if evaluation is None:
        return 'invalid_unscorable'
    ok = evaluation['refined']['within_limits']
    if accepted:
        return 'correct_accept' if ok else 'wrong_accept'
    # Protocol: any non-acceptance of a positive is a false reject; whether the
    # rejected primary was right is reported separately (primary_within_limits).
    return 'false_reject'


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--root', required=True)
    p.add_argument('--version', required=True)
    p.add_argument('--out', required=True)
    a = p.parse_args()
    rows = []
    for s in sorted(glob.glob(f'{a.root}/search/*-{a.version}/search.json')):
        d = Path(s).parent
        name = d.name[:-len(a.version) - 1]
        if name.rsplit('-', 1)[-1] not in ('own', 'x01', 'cross', 'hole'):
            continue
        dataset, window, variant = name.rsplit('-', 2)
        search = json.loads(Path(s).read_text())
        ev = d / 'evaluation.json'
        evaluation = json.loads(ev.read_text()) if ev.exists() else None
        manifest = json.loads((Path(a.root) / 'inputs' / d.name / 'manifest.json').read_text())
        top = search['distinct_top'][0] if search['distinct_top'] else {}
        conf = search.get('confirmation') or {}
        rows.append({'dataset': dataset, 'window': window, 'variant': variant, 'decision': search['decision'],
                     'class': classify(variant, search, evaluation),
                     'primary_within_limits': evaluation['refined']['within_limits'] if evaluation else None,
                     'xy_m': evaluation['refined']['xy_error_m'] if evaluation else None,
                     'yaw_deg': evaluation['refined']['yaw_error_deg'] if evaluation else None,
                     'support': top.get('support'), 'rmse': top.get('rmse'), 'margin': search['support_margin'],
                     'confirm_xy_yaw': conf.get('xy_yaw_difference'), 'tilt_deg': top.get('tilt_deg'),
                     'search_s': search['elapsed_search_s'],
                     'cpu_s': search['child_cpu_seconds'] + search['self_cpu_seconds'],
                     'rss_kib': search['orchestrator_maxrss_kib'],
                     'span_s': manifest['stamps_s'][-1] - manifest['stamps_s'][0],
                     's3_stop': (manifest.get('s3_stop') or {}).get('reason'),
                     'odom_path_m': (manifest.get('s3_stop') or {}).get('odom_path_m')})
    with open(a.out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    for variant in ('own', 'x01', 'cross', 'hole'):
        sub = [r for r in rows if r['variant'] == variant]
        if not sub:
            continue
        counts = {}
        for r in sub:
            counts[r['class']] = counts.get(r['class'], 0) + 1
        n = len(sub)
        print(variant, n, counts, 'accept', [round(x, 3) for x in wilson(sum(r['decision'] == 'accepted' for r in sub), n)],
              'wrong_accept', [round(x, 3) for x in wilson(counts.get('wrong_accept', 0), n)])


if __name__ == '__main__':
    main()
