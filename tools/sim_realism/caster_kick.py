"""Yaw-rate disturbance after an in-place turn (swivel casters left sideways): for each in-place turn (|v cmd| < 0.03, |w cmd| > 0.2 for >= 0.5 s) followed within 1 s by forward motion (v cmd > 0.05, |w cmd| < 0.15),
the actual yaw rate (gyro, bias removed) over the next 0-2.5 s, multiplied by the sign of the turn that preceded it (negative = opposite to the previous turn). usage: caster_kick.py BAG_DIR [BAG_DIR...]"""
import os
import sys

import numpy as np

src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "base_response_from_bag.py")).read().split('print("\\n1. base response')[0]
bags = sys.argv[1:]; allrows = []
for b in bags:
    sys.argv = [sys.argv[0], b]; exec(src)
    inp = (np.abs(U[:, 0]) < 0.03) & (np.abs(U[:, 1]) > 0.2); fwd = (U[:, 0] > 0.05) & (np.abs(U[:, 1]) < 0.15); i = 0; n = len(g)
    while i < n:
        if inp[i]:
            j = i
            while j + 1 < n and inp[j + 1]: j += 1
            if (j - i) * dt >= 0.5:
                s = np.sign(np.mean(U[i:j + 1, 1])); k0 = j + 1; k1 = min(j + int(1.0 / dt), n - 1)
                st = np.where(fwd[k0:k1 + 1])[0]
                if len(st):
                    a = k0 + st[0]; seg = gz[a:a + int(2.5 / dt)]
                    if len(seg) == int(2.5 / dt): allrows.append((s, seg * s, os.path.basename(os.path.dirname(b))))
            i = j + 1
        else: i += 1
if not allrows: print("no sequences found"); sys.exit()
print(f"{len(allrows)} in-place turns followed by forward motion; mean yaw rate x sign(previous turn), by time after the start of the forward motion (rad/s):")
A = np.array([r[1] for r in allrows]); tt = np.arange(A.shape[1]) * dt
for a, b in ((0, 0.25), (0.25, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 2.5)):
    m = (tt >= a) & (tt < b); v = A[:, m].mean(1); print(f"   {a:4.2f}-{b:4.2f} s: mean {v.mean():+.4f}, median {np.median(v):+.4f}, share negative (opposite to the turn) {100*np.mean(v<0):.0f} %, std {v.std():.4f}")
print("by turn sign: left turns:", np.round(A[[r[0] > 0 for r in allrows], :int(1.0/dt)].mean(), 4), " right turns:", np.round(A[[r[0] < 0 for r in allrows], :int(1.0/dt)].mean(), 4), "(already multiplied by the sign: negative = opposite)")
