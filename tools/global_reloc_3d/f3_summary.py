"""NX host (python3 + numpy-free): F3 resources and timing, aggregated from finished runs. Reads only; writes JSON/CSV on stdout.
  f3_summary.py stages RUNS_DIR [RUN_NAME_REGEX]     per-run stage times of the attempts that were accepted (metrics.json)
  f3_summary.py teg RUNS_DIR [RUN_NAME_REGEX]        per-run tegrastats: RAM, swap, GR3D, temperatures, power, CPU
  f3_summary.py cost RUNS_DIR [RUN_NAME_REGEX]       per-run per-process cost_summary.json (CPU %, PSS peak)
Distributions (median / p95 / max, N) are in `dist`; the per-run rows go to stderr as CSV."""
import csv
import glob
import json
import os
import re
import sys

STAGES = ['startup_to_first_input_s', 'imu_time_s', 'lio_time_s', 'submap_exposure_s', 'submap_build_s', 'search_worker_wall_s', 'map_field_s',
          'fft_primary_s', 'fft_confirm_s', 'gicp_s', 'score_s', 'confirm_total_s', 'handoff_publish_s', 'handoff_confirm_s', 'map_check_s']
TOTALS = ['protocol_accept_from_first_input', 'accept_from_lio_ready', 'accept_from_acq_end', 'localized_from_first_input']


def pct(v, q):
    v = sorted(v)
    k = (len(v) - 1) * q / 100.
    lo, hi = int(k), min(int(k) + 1, len(v) - 1)
    return v[lo] + (v[hi] - v[lo]) * (k - lo)


def dist(v):
    v = [x for x in v if x is not None]
    if not v:
        return None
    return {'n': len(v), 'median': round(pct(v, 50), 3), 'p95': round(pct(v, 95), 3), 'max': round(max(v), 3), 'min': round(min(v), 3)}


def runs(root, rx):
    r = re.compile(rx or '.')
    return [d for d in sorted(glob.glob(f'{root.rstrip("/")}/*/')) if r.search(os.path.basename(d.rstrip('/')))]


def stages(root, rx):
    rows, out = [], {}
    for d in runs(root, rx):
        for f in sorted(glob.glob(d + 'work/attempt*/metrics.json')):
            m = json.load(open(f))
            if not m.get('accepted'):
                continue
            s, t = m.get('stage_s', {}), m.get('total_s', {})
            row = {'run': os.path.basename(d.rstrip('/')), 'attempt': m.get('attempt'), 'mode': m.get('mode'),
                   **{k: s.get(k) for k in STAGES}, **{k: t.get(k) for k in TOTALS}}
            rows.append(row)
    for k in STAGES + TOTALS:
        out[k] = dist([r[k] for r in rows])
    return rows, {'accepted_attempts': len(rows), 'dist': out}


TEG = re.compile(r'RAM (\d+)/(\d+)MB.*?SWAP (\d+)/\d+MB.*?CPU \[([^\]]*)\] GR3D_FREQ (\d+)%')
TJ, CT, GT, VIN = (re.compile(p) for p in (r'tj@([\d.]+)C', r' cpu@([\d.]+)C', r' gpu@([\d.]+)C', r'VDD_IN (\d+)mW'))


def teg(root, rx):
    rows = []
    for d in runs(root, rx):
        f = d + 'tegrastats.log'
        if not os.path.exists(f):
            continue
        ram, swap, gr3d, tj, cpu_t, gpu_t, vin, cpu = [], [], [], [], [], [], [], []
        for line in open(f, errors='replace'):
            m = TEG.search(line)
            if not m or not (TJ.search(line) and CT.search(line) and GT.search(line) and VIN.search(line)):
                continue
            ram.append(int(m.group(1))); swap.append(int(m.group(3))); gr3d.append(int(m.group(5)))
            tj.append(float(TJ.search(line).group(1))); cpu_t.append(float(CT.search(line).group(1))); gpu_t.append(float(GT.search(line).group(1)))
            vin.append(int(VIN.search(line).group(1)))
            c = [int(x.split('%')[0]) for x in m.group(4).split(',') if x.split('%')[0].isdigit()]
            cpu.append(sum(c) / 100.)
        if ram:
            rows.append({'run': os.path.basename(d.rstrip('/')), 'n': len(ram), 'ram_max_mb': max(ram), 'swap_max_mb': max(swap), 'swap_rise_mb': max(swap) - swap[0],
                         'gr3d_max_pct': max(gr3d), 'gr3d_mean_pct': round(sum(gr3d) / len(gr3d), 2), 'tj_max_c': max(tj), 'cpu_t_max_c': max(cpu_t),
                         'gpu_t_max_c': max(gpu_t), 'vdd_in_mean_w': round(sum(vin) / len(vin) / 1000., 2), 'vdd_in_max_w': round(max(vin) / 1000., 2),
                         'cores_busy_mean': round(sum(cpu) / len(cpu), 2), 'cores_busy_p95': round(pct(cpu, 95), 2)})
    keys = ['ram_max_mb', 'swap_max_mb', 'swap_rise_mb', 'gr3d_max_pct', 'tj_max_c', 'cpu_t_max_c', 'gpu_t_max_c', 'vdd_in_mean_w', 'vdd_in_max_w', 'cores_busy_mean', 'cores_busy_p95']
    return rows, {'runs': len(rows), 'dist': {k: dist([r[k] for r in rows]) for k in keys}}


def cost(root, rx):
    per, rows = {}, []
    for d in runs(root, rx):
        f = d + 'cost_summary.json'
        if not os.path.exists(f):
            continue
        s = json.load(open(f)).get('per_process', {})
        for p, v in s.items():
            row = {'run': os.path.basename(d.rstrip('/')), 'process': p, 'cpu_mean': v['cpu_pct']['mean'], 'cpu_p95': v['cpu_pct']['p95'], 'cpu_max': v['cpu_pct']['max'],
                   'pss_peak_mb': v.get('pss_peak_mb'), 'rss_peak_mb': v.get('rss_peak_mb')}
            rows.append(row)
            per.setdefault(p, []).append(row)
    return rows, {'processes': {p: {'runs': len(v), 'cpu_mean': dist([r['cpu_mean'] for r in v]), 'pss_peak_mb': dist([r['pss_peak_mb'] for r in v])} for p, v in per.items()}}


def main():
    what, root = sys.argv[1], sys.argv[2]
    rx = sys.argv[3] if len(sys.argv) > 3 else None
    rows, summ = {'stages': stages, 'teg': teg, 'cost': cost}[what](root, rx)
    if rows:
        w = csv.DictWriter(sys.stderr, list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(json.dumps(summ, indent=1))


if __name__ == '__main__':
    main()
