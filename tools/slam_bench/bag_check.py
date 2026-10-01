"""Gaps in the LiDAR and IMU streams of a ROS 2 bag, from the time each message was recorded (nothing is deserialised, so it is fast on large bags).

  bag_check.py <bag> [--lidar /livox/lidar] [--imu /livox/imu]      exit 0 = ok or warnings only, 1 = FAIL

A scan lost while recording (UDP receive buffer full, recorder stalled) is gone for good and neither the driver nor the SLAM warns about it. Run this on every bag right after recording it.
A gap is an interval longer than 2.5x the median interval of the topic (at least 50 ms: scheduler noise on a 200 Hz IMU is not a loss); `lost` counts the messages it swallowed.
Verdict per topic: ok (no gaps) / warn (gaps, but under 1 % of the messages lost and none over 1 s) / FAIL (more, or fewer than 2 messages). Our good bags lose 1-3 scans of ~6 000 (warn, they map fine);
a Mid-360 bag that loses 50 % of its scans (real_01) is FAIL.
Needs numpy + rosbags. Recording time, not the header stamp: a recorder that stalls and then catches up bunches its messages and hides the hole."""
import argparse
import sys
from pathlib import Path

import numpy as np

GAP_FACTOR, GAP_MIN = 2.5, 0.05
FAIL_LOST, FAIL_GAP = 0.01, 1.0         # share of lost messages, longest gap (s)


def check_stream(t):
    """t: message times in seconds. -> n, median period, gaps [(start, length)] and lost (messages the gaps swallowed)."""
    t = np.sort(np.asarray(t, float))
    if len(t) < 2:
        return {"n": len(t), "period": None, "gaps": [], "lost": 0}
    d, period = np.diff(t), np.median(np.diff(t))
    big = np.flatnonzero(d > max(GAP_FACTOR * period, GAP_MIN))
    return {"n": len(t), "period": period, "gaps": [(t[i], d[i]) for i in big], "lost": int(sum(round(d[i] / period) - 1 for i in big))}


def verdict(r):
    if r["n"] < 2:
        return "FAIL"
    if not r["gaps"]:
        return "ok"
    return "FAIL" if r["lost"] / (r["n"] + r["lost"]) > FAIL_LOST or max(g[1] for g in r["gaps"]) > FAIL_GAP else "warn"


def bag_times(bag, topics):
    from rosbags.highlevel import AnyReader
    out = {k: [] for k in topics}
    with AnyReader([Path(bag)]) as r:
        for c, ns, _ in r.messages(connections=[c for c in r.connections if c.topic in topics]):
            out[c.topic].append(ns * 1e-9)
    return out


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--lidar", default="/livox/lidar")
    ap.add_argument("--imu", default="/livox/imu")
    a = ap.parse_args(argv)
    times, bad = bag_times(a.bag, [a.lidar, a.imu]), False
    for topic in (a.lidar, a.imu):
        r = check_stream(times[topic])
        v = verdict(r)
        if r["n"] < 2:
            print(f"{topic}: {r['n']} messages  {v}")
        else:
            print(f"{topic}: {r['n']} messages, median period {r['period'] * 1e3:.1f} ms, {len(r['gaps'])} gaps, ~{r['lost']} lost  {v}")
            for start, length in sorted(r["gaps"], key=lambda g: -g[1])[:10]:
                print(f"    gap of {length:.2f} s at {start - min(times[topic]):.1f} s from the first message")
        bad |= v == "FAIL"
    return int(bad)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
