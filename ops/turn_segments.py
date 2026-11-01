"""Turning episodes from the angular-speed samples of the command chain (lines `epoch topic w`, topics cmd_vel_nav, cmd_vel_smoothed, cmd_vel, base_odom).
An episode = base_odom |w| above FLOOR for at least MIN_S, merged across gaps shorter than GAP_S. Per episode: duration, peak and mean |w| at each stage, and the angle
the base turned (integral of its own w).  turn_segments.py SAMPLES [FLOOR=0.05] [MIN_S=0.5]"""
import sys
from collections import defaultdict

GAP_S = 0.5
STAGES = ("cmd_vel_nav", "cmd_vel_smoothed", "cmd_vel", "base_odom")


def load(path):
    by = defaultdict(list)
    for line in open(path):
        p = line.split()
        if len(p) == 3 and p[1] in STAGES:
            by[p[1]].append((float(p[0]), float(p[2])))
    return by


def episodes(samples, floor, min_s):
    """-> [(start, end)] where `samples` ((t, w) pairs) has |w| > floor, merged across gaps < GAP_S, at least min_s long."""
    out = []
    for t, w in samples:
        if abs(w) <= floor:
            continue
        if out and t - out[-1][1] < GAP_S:
            out[-1][1] = t
        else:
            out.append([t, t])
    return [(a, b) for a, b in out if b - a >= min_s]


def angle(samples, a, b):
    """Integral of w over [a, b] (rad), rectangle rule on the sample times."""
    pts = [(t, w) for t, w in samples if a - 0.05 <= t <= b + 0.05]
    return sum(w * (t2 - t1) for (t1, w), (t2, _) in zip(pts, pts[1:]))


if __name__ == "__main__":
    by = load(sys.argv[1])
    floor = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05
    min_s = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
    t0 = min(t for s in by.values() for t, _ in s)
    print(f"{'start':>7} {'dur':>6} {'turned':>7} | peak|w| nav smooth cmd base | mean|w| of base")
    for a, b in episodes(by["base_odom"], floor, min_s):
        peak = {n: max((abs(w) for t, w in by[n] if a - 0.3 <= t <= b + 0.3), default=0) for n in STAGES}
        seg = [abs(w) for t, w in by["base_odom"] if a <= t <= b]
        print(f"{a - t0:7.1f} {b - a:5.1f}s {abs(angle(by['base_odom'], a, b)) * 57.2958:6.0f}° | "
              f"{peak['cmd_vel_nav']:.2f} {peak['cmd_vel_smoothed']:.2f} {peak['cmd_vel']:.2f} {peak['base_odom']:.2f} | {sum(seg) / len(seg):.2f}")
