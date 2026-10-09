"""CPU time, RSS and PSS per relevant process from /proc, 2 Hz. CPU is process CPU time over wall time (100 % = one core).
  proc_cost.py sample out.csv      until killed      proc_cost.py summarize out.csv > summary.json
The 2 Hz sampling misses short-lived peaks: search_v2's report also carries RUSAGE_CHILDREN maxrss."""
import json
import os
import sys
import time

import numpy as np

WATCH = {'fastlio_mapping': 'fastlio_mapping', 'amcl': 'nav2_amcl/amcl', 'scan_conv': 'pointcloud_to_laserscan_node', 'map_server': 'nav2_map_server/map_server',
         'reloc_node': 'reloc_node.py', 'search_v2': 'search_v2.py', 'register_v2': 'register_v2', 'bag_play': 'bag play', 'lifecycle': 'lifecycle_manager'}
TICK = os.sysconf('SC_CLK_TCK')


def ticks(stat):
    f = stat[stat.rindex(')') + 2:].split()
    return int(f[11]) + int(f[12])


def mem(pid):
    rss = pss = 0.
    try:
        for line in open(f'/proc/{pid}/smaps_rollup'):
            if line.startswith('Rss:'): rss = int(line.split()[1]) / 1024
            elif line.startswith('Pss:'): pss = int(line.split()[1]) / 1024
    except OSError:
        pass
    return rss, pss


def scan():
    out = {}
    for pid in filter(str.isdigit, os.listdir('/proc')):
        if int(pid) == os.getpid():
            continue
        try:
            cmd = open(f'/proc/{pid}/cmdline').read().replace('\0', ' ')
            for name, key in WATCH.items():
                if key in cmd and 'proc_cost' not in cmd:
                    out[int(pid)] = (name, ticks(open(f'/proc/{pid}/stat').read()))
                    break
        except OSError:
            continue
    return out


def sample(path, every=.5):
    prev, t_prev = {}, time.monotonic()
    with open(path, 'w') as f:
        f.write('t,name,pid,cpu_pct,rss_mb,pss_mb\n')
        while True:
            time.sleep(every)
            now = time.monotonic(); cur = scan()
            for pid, (name, tk) in cur.items():
                if pid in prev:
                    rss, pss = mem(pid)
                    f.write(f'{now:.3f},{name},{pid},{(tk - prev[pid]) / TICK / (now - t_prev) * 100:.1f},{rss:.1f},{pss:.1f}\n')
            f.flush(); prev, t_prev = {p: v[1] for p, v in cur.items()}, now


def summarize(path):
    import csv
    rows = list(csv.DictReader(open(path)))
    by, tot = {}, {}
    for r in rows:
        by.setdefault(r['name'], []).append(r)
        a = tot.setdefault(r['t'], {'cpu': 0., 'rss': 0., 'pss': 0., 'cpu_nobag': 0., 'pss_nobag': 0.})
        a['cpu'] += float(r['cpu_pct']); a['rss'] += float(r['rss_mb']); a['pss'] += float(r['pss_mb'])
        if r['name'] != 'bag_play':
            a['cpu_nobag'] += float(r['cpu_pct']); a['pss_nobag'] += float(r['pss_mb'])
    st = lambda v: {'mean': float(np.mean(v)), 'p95': float(np.percentile(v, 95)), 'max': float(np.max(v))}
    out = {'per_process': {n: {'cpu_pct': st([float(r['cpu_pct']) for r in v]), 'rss_peak_mb': max(float(r['rss_mb']) for r in v),
                              'pss_peak_mb': max(float(r['pss_mb']) for r in v), 'samples': len(v)} for n, v in by.items()},
           'total_excl_bag_play': {'cpu_pct': st([a['cpu_nobag'] for a in tot.values()]), 'pss_peak_mb': max(a['pss_nobag'] for a in tot.values())},
           'total_incl_bag_play': {'cpu_pct': st([a['cpu'] for a in tot.values()]), 'rss_peak_mb_sum': max(a['rss'] for a in tot.values())},
           'note': 'rss sum overcounts shared libraries; prefer PSS. CPU 100 % = one core of 8.'}
    print(json.dumps(out, indent=2))


if __name__ == '__main__':
    sample(sys.argv[2]) if sys.argv[1] == 'sample' else summarize(sys.argv[2])
