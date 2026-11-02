"""Per-process CPU and RAM of the localization stack, stdlib only (runs in the container of a sim run and on the robot).
  procmon.py sample OUT.csv [period_s=1]     one row per process and sample until SIGTERM: wall, pid, ppid, cpu ticks (own, reaped children), rss_kb, hwm_kb, command
  procmon.py report OUT.csv [more.csv ...] [-s scenario.json]  avg / p95 / max per stack process and for the whole stack (several runs pool their samples); per phase with the sup_cover.py scenario of a single run
CPU is in % of one core. The stack is everything but the simulator (Gazebo, sim_*, socialtech_livox_sim), the scenario and this tool; processes that match
neither list are printed apart as `other`. register_v2 lives under a second (invisible at 1 Hz): its CPU is the search worker's reaped-children ticks."""
import csv
import os
import re
import signal
import sys
import time

TICK = os.sysconf('SC_CLK_TCK')
SIM = re.compile(r'gz|ruby|livox_mid360_sim|sim_base_model|sim_wheel_odom|sim_imu_model|sim_scan_model|sup_cover|nav_diag|trials\.py|procmon|ros2 (param|topic)|hospital-agv-sim|ros_gz|parameter_bridge|image_bridge|noisy_livox|robot_state_publisher|ros2cli')
STACK = [('register_v2', r'^\S*/register_v2 '), ('search_worker', r'^\S*python\S* (-\S+ )*\S*search_v2'),          # first: their paths contain /runs/gn_supervisor/
         ('fastlio', r'fastlio_mapping'), ('planar_odom', r'planar_odom'), ('scan_median', r'scan_median'), ('gn_loc', r'gn_loc_node'),
         ('supervisor', r'gn_supervisor_node'), ('live_paths', r'live_paths'),
         ('nav2', r'controller_server|planner_server|bt_navigator|behavior_server|smoother_server|velocity_smoother|collision_monitor|lifecycle_manager|map_server|waypoint_follower|route_server|nav2_'),
         ('launch', r'ros2 launch|launch_ros|/ops/')]


def group(cmd):
    if SIM.search(cmd):
        return 'sim'
    return next((g for g, rx in STACK if re.search(rx, cmd)), 'other')


def read(pid):
    with open(f'/proc/{pid}/stat') as f:
        s = f.read()
    r = s.rindex(')')
    f = s[r + 2:].split()                                   # f[0] is the state (field 3)
    with open(f'/proc/{pid}/cmdline', 'rb') as c:
        cmd = c.read().replace(b'\0', b' ').decode(errors='replace').strip() or s[s.index('(') + 1:r]
    rss = hwm = 0
    with open(f'/proc/{pid}/status') as st:
        for line in st:
            if line.startswith('VmRSS:'):
                rss = int(line.split()[1])
            elif line.startswith('VmHWM:'):
                hwm = int(line.split()[1])
    return int(f[1]), int(f[11]) + int(f[12]), int(f[13]) + int(f[14]), rss, hwm, cmd[:160]


def sample(out, period):
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    w = csv.writer(open(out, 'w', newline=''))
    w.writerow(['wall', 'pid', 'ppid', 'ticks', 'child_ticks', 'rss_kb', 'hwm_kb', 'cmd'])
    me = os.getpid()
    while not stop:
        t = time.time()
        for p in os.listdir('/proc'):
            if p.isdigit() and int(p) != me:
                try:
                    ppid, tk, ctk, rss, hwm, cmd = read(p)
                except (OSError, ValueError):               # gone between listdir and read
                    continue
                w.writerow([f'{t:.3f}', p, ppid, tk, ctk, rss, hwm, cmd])
        time.sleep(max(period - (time.time() - t), 0))


def pct(xs, q):
    xs = sorted(xs)
    return xs[min(int(q * len(xs)), len(xs) - 1)] if xs else 0.


def series(*paths):
    """{(pid, cmd, run): [(wall, cpu_pct, child_pct, rss_mb, hwm_mb)]} from the deltas of consecutive samples."""
    rows, out = {}, {}
    for i, path in enumerate(paths):
        with open(path) as f:
            for r in csv.DictReader(f):
                rows.setdefault((r['pid'], r['cmd'], i), []).append(r)
    for k, rs in rows.items():
        for a, b in zip(rs, rs[1:]):
            dt = float(b['wall']) - float(a['wall'])
            if dt > 0:
                out.setdefault(k, []).append((float(b['wall']), 100 * (int(b['ticks']) - int(a['ticks'])) / TICK / dt,
                                              100 * (int(b['child_ticks']) - int(a['child_ticks'])) / TICK / dt, int(b['rss_kb']) / 1024, int(b['hwm_kb']) / 1024))
    return out


def stack_total(ser):
    """Per sample time: stack CPU (own ticks of the stack processes, register_v2 replaced by the worker's reaped children) and stack RSS."""
    cpu, rss = {}, {}
    for (pid, cmd, _), pts in ser.items():
        g = group(cmd)
        if g in ('sim', 'other', 'register_v2'):
            continue
        for t, c, ch, r, _ in pts:
            k = round(t)
            cpu[k] = cpu.get(k, 0.) + c + (ch if g == 'search_worker' else 0.)
            rss[k] = rss.get(k, 0.) + r
    return cpu, rss


def phases(scn):
    import json
    with open(scn) as f:
        ev = json.load(f)['events']
    w = lambda what, **kw: next((e['wall'] for e in ev if e['what'] == what and all(e.get(k) == v for k, v in kw.items())), None)
    mon, cov, unc = w('state', state='MONITOR'), w('cover'), w('uncover')
    back = next((e['wall'] for e in ev if e['what'] == 'state' and e['state'] == 'MONITOR' and unc and e['wall'] > unc), None)
    names = ['startup', 'monitor', 'covered', 'reloc_after_uncover', 'after']
    edges = [0., mon, cov, unc, back, 1e12]
    return [(n, a, b) for n, a, b in zip(names, edges, edges[1:]) if a is not None and b is not None]


def stats(xs):
    return f'avg {sum(xs) / len(xs):6.1f}  p95 {pct(xs, .95):6.1f}  max {max(xs):6.1f}' if xs else 'no samples'


def report(paths, scn=None):
    ser = series(*paths)
    byg = {}
    for (pid, cmd, run), pts in ser.items():
        argv = [a for a in cmd.split() if not a.startswith('-')]
        name = argv[1 if argv[0].split('/')[-1].startswith('python') and len(argv) > 1 else 0].split('/')[-1] if argv else cmd
        byg.setdefault(group(cmd), {}).setdefault(name, []).extend(pts)
    cpu, rss = stack_total(ser)
    print(f'samples {len(cpu)} s of stack; CPU in % of one core, RSS in MB (hwm = VmHWM); a short-lived process (search worker) averages over its own lifetime, the STACK TOTAL is per second')
    print(f'{"process":34s} {"cpu avg":>8s} {"p95":>7s} {"max":>7s} {"rss avg":>8s} {"rss max":>8s} {"hwm":>7s}')
    for g in [g for g, _ in STACK] + ['other']:
        for name, pts in sorted(byg.get(g, {}).items(), key=lambda kv: -sum(p[1] for p in kv[1])):
            c = [p[1] + (p[2] if g == 'search_worker' else 0) for p in pts]
            r = [p[3] for p in pts]
            if g == 'other' or g == 'register_v2' or sum(c) / len(c) > .05 or max(r) > 20:
                print(f'{g[:10] + ":" + name:34.34s} {sum(c) / len(c):8.1f} {pct(c, .95):7.1f} {max(c):7.1f} {sum(r) / len(r):8.1f} {max(r):8.1f} {max(p[4] for p in pts):7.1f}')
    print(f'\nSTACK TOTAL cpu  {stats(list(cpu.values()))}   (% of one core)')
    print(f'STACK TOTAL rss  {stats(list(rss.values()))}   MB')
    if scn:
        for n, a, b in phases(scn):
            ks = [k for k in cpu if a <= k < b]
            if ks:
                print(f'  phase {n:20s} {len(ks):4d} s  cpu {stats([cpu[k] for k in ks])}  rss max {max(rss[k] for k in ks):6.0f} MB')


if __name__ == '__main__':
    if sys.argv[1] == 'sample':
        sample(sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 1.)
    else:
        a = sys.argv[2:]
        report([x for x in a if x != '-s' and not (x.endswith('.json'))], a[a.index('-s') + 1] if '-s' in a else None)
