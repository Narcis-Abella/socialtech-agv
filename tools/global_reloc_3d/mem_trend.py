"""Memory trend per process from proc_cost.py samples (cost.csv): PSS at the start and end of a time span and the least-squares slope.
  mem_trend.py cost.csv [t_from_s [t_to_s]]      times are seconds since the first sample; default span = from 60 s to the end"""
import csv
import json
import sys

import numpy as np


def main():
    rows = list(csv.DictReader(open(sys.argv[1])))
    t0 = float(rows[0]['t'])
    a = float(sys.argv[2]) if len(sys.argv) > 2 else 60.
    b = float(sys.argv[3]) if len(sys.argv) > 3 else float('inf')
    out = {}
    for name in sorted({r['name'] for r in rows}):
        v = np.array([(float(r['t']) - t0, float(r['pss_mb']), float(r['cpu_pct'])) for r in rows if r['name'] == name and a <= float(r['t']) - t0 <= b])
        if len(v) < 20:
            continue
        k = max(1, len(v) // 20)
        out[name] = {'samples': len(v), 'span_s': round(float(v[-1, 0] - v[0, 0]), 1), 'pss_first_mb': round(float(np.median(v[:k, 1])), 1),
                     'pss_last_mb': round(float(np.median(v[-k:, 1])), 1), 'pss_max_mb': round(float(v[:, 1].max()), 1),
                     'slope_mb_per_min': round(float(np.polyfit(v[:, 0], v[:, 1], 1)[0] * 60), 3), 'cpu_mean_pct': round(float(v[:, 2].mean()), 1)}
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
