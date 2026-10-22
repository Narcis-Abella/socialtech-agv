"""Summary of a Jetson cost run (jetson_cost.sh): cores used, RAM, GPU, temperature and power from tegrastats (1 s), and the container's CPU / memory from docker stats (5 s).

  cost_summary.py <mon_<name>.log> [dstats_<name>.log]

Cores used = sum of the per-core utilisations / 100 (the whole board, not only the container). Power is VDD_IN (total input), which the AGX does not report."""
import re
import sys

import numpy as np


def parse_tegrastats(lines):
    """One dict per tegrastats line (others are skipped): ram, ram_total (MB), cores (% each, offline = 0), used (cores), gpu (%), tj (C), vdd_in (W or None)."""
    out = []
    for l in lines:
        m, c = re.search(r"RAM (\d+)/(\d+)MB", l), re.search(r"CPU \[([^\]]*)\]", l)
        if not (m and c):
            continue
        cores = [int(x.split("%")[0]) if "%" in x else 0 for x in c.group(1).split(",")]
        g, t, p = re.search(r"GR3D_FREQ (\d+)%", l), re.search(r"\btj@([\d.]+)C", l), re.search(r"\bVDD_IN (\d+)mW", l)
        out.append(dict(ram=int(m.group(1)), ram_total=int(m.group(2)), cores=cores, used=sum(cores) / 100, gpu=int(g.group(1)) if g else 0,
                        tj=float(t.group(1)) if t else None, vdd_in=int(p.group(1)) / 1000 if p else None))
    return out


def parse_dstats(lines):
    """(container cores, memory MiB) per 'epoch cpu% mem / limit' line; lines without a MiB / GiB figure are skipped."""
    out = []
    for l in lines:
        m = re.match(r"\d+ ([\d.]+)% ([\d.]+)(MiB|GiB)", l)
        if m:
            out.append((float(m.group(1)) / 100, float(m.group(2)) * (1024 if m.group(3) == "GiB" else 1)))
    return out


def summarize(samples):
    u = np.array([s["used"] for s in samples])
    tj = [s["tj"] for s in samples if s["tj"] is not None]
    pw = [s["vdd_in"] for s in samples if s["vdd_in"] is not None]
    return dict(seconds=len(samples), cores_mean=float(u.mean()), cores_p95=float(np.percentile(u, 95)), cores_max=float(u.max()), ram_max=max(s["ram"] for s in samples),
                ram_total=samples[0]["ram_total"], gpu_mean=float(np.mean([s["gpu"] for s in samples])), tj_max=max(tj) if tj else None,
                power_mean=float(np.mean(pw)) if pw else None, power_max=max(pw) if pw else None)


def main(argv):
    r = summarize(parse_tegrastats(open(argv[0]).read().splitlines()))
    print(f"{r['seconds']} s | board cores used mean {r['cores_mean']:.2f}, p95 {r['cores_p95']:.2f}, max {r['cores_max']:.2f} | RAM max {r['ram_max']} of {r['ram_total']} MB | GPU mean {r['gpu_mean']:.0f} % | "
          f"Tj max {r['tj_max']} C | power mean {r['power_mean'] and round(r['power_mean'], 1)} W, max {r['power_max']} W")
    if len(argv) > 1:
        d = parse_dstats(open(argv[1]).read().splitlines())
        if d:
            c, m = np.array([x[0] for x in d]), np.array([x[1] for x in d])
            print(f"container: cores mean {c.mean():.2f}, max {c.max():.2f} | memory max {m.max():.0f} MiB")


if __name__ == "__main__":
    main(sys.argv[1:])
