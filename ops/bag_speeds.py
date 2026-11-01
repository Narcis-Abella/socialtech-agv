"""Speeds per stage in a nav run bag: bag_speeds.py RUN_DIR_OR_BAG [--moving 0.05]
Per stage (controller /cmd_vel_nav, smoother, collision monitor /cmd_vel, base /odom): max, p95 and mean-while-moving of |vx| and |w|, and the share of samples at the cap.
Runs with ~/venv-ops (rosbags), no ROS needed."""
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

STAGES = {"/cmd_vel_nav": "controller", "/cmd_vel_smoothed": "smoother", "/cmd_vel": "collision_mon", "/odom": "base_odom"}


def twist(msg):
    t = getattr(msg, "twist", msg)
    t = getattr(t, "twist", t)   # Odometry -> TwistWithCovariance -> Twist
    return t.linear.x, t.angular.z


def load(path):
    p = Path(path)
    bag = p / "bag" if (p / "bag").is_dir() else p
    out = {k: [] for k in STAGES}
    with AnyReader([bag]) as r:
        conns = [c for c in r.connections if c.topic in STAGES]
        for c, t, raw in r.messages(connections=conns):
            out[c.topic].append((t * 1e-9, *twist(r.deserialize(raw, c.msgtype))))
    return out


def report(data, moving=0.05):
    print(f"{'stage':<14}{'n':>6}{'max|vx|':>9}{'p95|vx|':>9}{'max|w|':>8}{'p95|w|':>8}{'mean|vx| mv':>13}{'mean|w| mv':>12}")
    for topic, name in STAGES.items():
        a = np.array(data[topic])
        if not len(a):
            print(f"{name:<14}{0:>6}")
            continue
        vx, w = np.abs(a[:, 1]), np.abs(a[:, 2])
        mv = vx > moving
        mw = w > moving
        print(f"{name:<14}{len(a):>6}{vx.max():>9.2f}{np.percentile(vx, 95):>9.2f}{w.max():>8.2f}{np.percentile(w, 95):>8.2f}"
              f"{(vx[mv].mean() if mv.any() else 0):>13.2f}{(w[mw].mean() if mw.any() else 0):>12.2f}")


if __name__ == "__main__":
    report(load(sys.argv[1]))
