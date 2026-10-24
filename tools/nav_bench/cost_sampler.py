"""CPU and RAM of the processes of a bench run, read from /proc (the image has no psutil).
  cost_sampler.py sample <out.csv> [--every 1.0]   sample until killed; CSV columns t, name, cpu_pct (of ONE core), rss_mb
  cost_sampler.py summarize <out.csv>              mean / p95 CPU and peak RSS per name (processes sharing a name are added up), and the total of all of them"""
import argparse
import os
import time

import numpy as np

# name -> a substring of the process command line
WATCH = {"fastlio_mapping": "fastlio_mapping", "amcl": "nav2_amcl/amcl", "pointcloud_to_laserscan": "pointcloud_to_laserscan_node",
         "planar_odom": "lib/planar_odom/planar_odom", "map_server": "nav2_map_server/map_server",
         # Nav2 simulation (tools/nav_sim): the Nav2 nodes (count them as nav2_*) and the simulator (sim_*, not part of the robot)
         "nav2_controller": "nav2_controller/controller_server", "nav2_planner": "nav2_planner/planner_server",
         "nav2_bt_navigator": "nav2_bt_navigator/bt_navigator", "nav2_behaviors": "nav2_behaviors/behavior_server",
         "nav2_smoother": "nav2_smoother/smoother_server", "nav2_velocity_smoother": "nav2_velocity_smoother/velocity_smoother",
         "nav2_collision_monitor": "nav2_collision_monitor/collision_monitor", "nav2_waypoint_follower": "nav2_waypoint_follower/waypoint_follower",
         "nav2_route": "nav2_route/route_server", "nav2_docking": "opennav_docking/opennav_docking",
         "nav2_lifecycle_manager": "nav2_lifecycle_manager/lifecycle_manager",
         "sim_gz": "gz sim", "sim_bridge": "ros_gz_bridge/parameter_bridge", "sim_noise": "noisy_"}
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
    """rows: (t, name, cpu_pct, rss_mb). Per name: the processes sharing it are ADDED at each sample time (e.g. the `gz sim` launcher and server), then mean / p95 over time and the peak RSS;
    procs = the most processes seen at one time. TOTAL = the sum of all processes at each sample time."""
    per, tot = {}, {}
    for t, name, cpu, rss in rows:
        a = per.setdefault(name, {}).setdefault(t, [0.0, 0.0, 0])
        a[0] += cpu
        a[1] += rss
        a[2] += 1
        b = tot.setdefault(t, [0.0, 0.0, 0])
        b[0] += cpu
        b[1] += rss
        b[2] += 1

    def stats(series):
        c = [p[0] for p in series]
        return {"cpu_mean": float(np.mean(c)), "cpu_p95": float(np.percentile(c, 95)), "rss_peak_mb": float(max(p[1] for p in series)), "procs": max(p[2] for p in series)}
    out = {n: stats(list(v.values())) for n, v in per.items()}
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
        print(f"{'process':<26}{'cpu_mean%':>10}{'cpu_p95%':>10}{'rss_peak_MB':>13}{'procs':>7}   (cpu in % of ONE core; processes sharing a name are added up)")
        for n, v in sorted(summarize(rows).items()):
            print(f"{n:<26}{v['cpu_mean']:>10.1f}{v['cpu_p95']:>10.1f}{v['rss_peak_mb']:>13.0f}{v['procs']:>7d}")
