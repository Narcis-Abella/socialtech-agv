"""CPU and RAM of the processes of a bench run, read from /proc (the image has no psutil).
  cost_sampler.py sample <out.csv> [--every 1.0]   sample until killed; CSV columns t, name, cpu_pct (of ONE core), rss_mb
  cost_sampler.py summarize <out.csv>              mean / p95 CPU and peak RSS per process, and the total of all of them"""
import argparse
import os
import time

import numpy as np

# name -> a substring of the process command line
WATCH = {"fastlio_mapping": "fastlio_mapping", "amcl": "nav2_amcl/amcl", "pointcloud_to_laserscan": "pointcloud_to_laserscan_node",
         "planar_odom": "planar_odom.py", "map_server": "nav2_map_server/map_server"}
TICK = os.sysconf("SC_CLK_TCK")


def cpu_ticks(stat):
    f = stat[stat.rindex(")") + 2:].split()  # the command name may hold spaces and parentheses: cut after its closing one; state is then f[0] (field 3)
    return int(f[11]) + int(f[12])  # utime + stime (fields 14 and 15)


def rss_mb(status):
    for line in status.splitlines():
        if line.startswith("VmRSS:"):
            return int(line.split()[1]) / 1024
    return 0.0


def summarize(rows):
    """rows: (t, name, cpu_pct, rss_mb). Per process, and TOTAL = the sum of all processes at each sample time."""
    by, tot = {}, {}
    for t, name, cpu, rss in rows:
        by.setdefault(name, []).append((cpu, rss))
        a = tot.setdefault(t, [0.0, 0.0])
        a[0] += cpu
        a[1] += rss

    def stats(pairs):
        c = [p[0] for p in pairs]
        return {"cpu_mean": float(np.mean(c)), "cpu_p95": float(np.percentile(c, 95)), "rss_peak_mb": float(max(p[1] for p in pairs))}
    out = {n: stats(v) for n, v in by.items()}
    out["TOTAL"] = stats(list(tot.values()))
    return out


def watched():
    found = {}
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            cmd = open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            continue
        for name, pat in WATCH.items():
            if pat in cmd and "cost_sampler" not in cmd:
                found[int(pid)] = name
    return found


def sample(path, every):
    last = {}
    t0 = time.monotonic()
    with open(path, "w") as f:
        f.write("t,name,cpu_pct,rss_mb\n")
        while True:
            time.sleep(every)
            now = time.monotonic()
            for pid, name in watched().items():
                try:
                    ticks, rss = cpu_ticks(open(f"/proc/{pid}/stat").read()), rss_mb(open(f"/proc/{pid}/status").read())
                except OSError:
                    continue
                if pid in last:
                    f.write(f"{now - t0:.2f},{name},{100 * (ticks - last[pid][1]) / TICK / (now - last[pid][0]):.1f},{rss:.1f}\n")
                last[pid] = (now, ticks)
            f.flush()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("sample")
    s.add_argument("out")
    s.add_argument("--every", type=float, default=1.0)
    m = sub.add_parser("summarize")
    m.add_argument("csv")
    a = ap.parse_args()
    if a.cmd == "sample":
        sample(a.out, a.every)
    else:
        rows = [(float(t), n, float(c), float(r)) for t, n, c, r in (l.strip().split(",") for l in open(a.csv).read().splitlines()[1:])]
        print(f"{'process':<26}{'cpu_mean%':>10}{'cpu_p95%':>10}{'rss_peak_MB':>13}   (cpu in % of ONE core)")
        for n, v in sorted(summarize(rows).items()):
            print(f"{n:<26}{v['cpu_mean']:>10.1f}{v['cpu_p95']:>10.1f}{v['rss_peak_mb']:>13.0f}")
