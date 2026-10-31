"""Turns in place in a nav run bag: bag_turns.py RUN_DIR. Episode = consecutive /cmd_vel_nav samples with |vx|<0.02 and |w|>0.05 (gap<0.3 s).
Per episode: duration, angle commanded (integral), angle the base reports (/odom), peak and mean |w| of both."""
import sys

import numpy as np

from bag_speeds import load

d = load(sys.argv[1])
cmd = np.array(d["/cmd_vel_nav"])
odo = np.array(d["/odom"])
rot = cmd[(np.abs(cmd[:, 1]) < 0.02) & (np.abs(cmd[:, 2]) > 0.05)]
eps, cur = [], []
for row in rot:
    if cur and row[0] - cur[-1][0] > 0.3:
        eps.append(np.array(cur))
        cur = []
    cur.append(row)
if cur:
    eps.append(np.array(cur))


def angle(x):
    return np.degrees(abs(np.sum(x[:-1, 2] * np.diff(x[:, 0]))))


print(f"{'#':>2}{'dur s':>7}{'cmd deg':>9}{'base deg':>9}{'cmd peak':>9}{'cmd mean':>9}{'base peak':>10}")
for i, e in enumerate(eps):
    a, b = e[0, 0], e[-1, 0]
    if b - a < 0.3:
        continue
    o = odo[(odo[:, 0] >= a) & (odo[:, 0] <= b)]
    print(f"{i:>2}{b - a:>7.1f}{angle(e):>9.0f}{angle(o) if len(o) > 1 else 0:>9.0f}{np.abs(e[:, 2]).max():>9.2f}{np.abs(e[:, 2]).mean():>9.2f}{np.abs(o[:, 2]).max() if len(o) else 0:>10.2f}")
