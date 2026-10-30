"""Structure of the angular gain of the real base (the 'slip'): actual yaw rate / delayed-and-lowpassed command, by |linear command| and by |angular command|, left vs right, and after a recent in-place turn.
usage: slip_structure.py BAG_DIR  (imports the loading code of base_response_from_bag.py by exec: same inputs, same 50 Hz grid)"""
import math
import os
import sys

import numpy as np

src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "base_response_from_bag.py")).read().split('print("\\n1. base response')[0]
sys.argv = [sys.argv[0], sys.argv[1]]; exec(src)
TD, TAU = 0.12, 0.05
sh = int(round(TD / dt)); uw = np.roll(U[:, 1], sh); uw[:sh] = 0; uv = np.roll(U[:, 0], sh); uv[:sh] = 0; xw = lowpass(uw, TAU); y = Y[:, 1]
print("\nangular gain = sum(x*y)/sum(x*x) of the actual yaw rate on the delayed command (Td 120 ms, tau 50 ms), motion only")
def gain(mask): return (xw[mask] @ y[mask]) / max(xw[mask] @ xw[mask], 1e-12), int(mask.sum())
print("by |linear command| (rows) and |angular command| (columns, rad/s):")
vb = [(0.0, 0.03), (0.03, 0.2), (0.2, 0.4), (0.4, 0.8)]; wb = [(0.05, 0.15), (0.15, 0.3), (0.3, 0.5), (0.5, 1.0)]
print("              " + "".join(f"{a:.2f}-{b:.2f}     " for a, b in wb))
for va, vbb in vb:
    line = f"v {va:.2f}-{vbb:.2f}  "
    for wa, wbb in wb:
        m = mv & (np.abs(uv) >= va) & (np.abs(uv) < vbb) & (np.abs(xw) >= wa) & (np.abs(xw) < wbb); gg, n = gain(m)
        line += f"{gg:5.2f} ({n/50:4.0f}s)  " if n > 100 else "      -        "
    print(line)
print("left (+w) vs right (-w), all speeds:", "  ".join(f"{nm} {gain(mv & (np.sign(xw) == s) & (np.abs(xw) > 0.05))[0]:.2f}" for nm, s in (("left", 1), ("right", -1))))
# after an in-place turn: time since the last moment with |v cmd| < 0.03 and |w cmd| > 0.2 ended
inplace = (np.abs(uv) < 0.03) & (np.abs(U[:, 1]) > 0.2); last = np.full(len(g), np.inf); t_last = -1e9
for i in range(len(g)):
    if inplace[i]: t_last = g[i]
    last[i] = g[i] - t_last
print("linear speed ratio actual/commanded (v>0.1) by time since the last in-place turn:")
for a, b in ((0, 0.5), (0.5, 1.5), (1.5, 4), (4, 1e9)):
    m = mv & (np.abs(uv) > 0.1) & (last >= a) & (last < b)
    if m.sum() > 100: xv = lowpass(np.roll(uv, int(0.06 / dt)), 0.1); print(f"   {a}-{b if b < 1e8 else 'inf'} s: gain {(xv[m] @ Y[m, 0]) / (xv[m] @ xv[m]):.3f} ({m.sum()*dt:.0f} s); yaw rate mean while moving straight-ish (|w cmd|<0.05): {np.mean(Y[m & (np.abs(U[:,1])<0.05), 1]) if (m & (np.abs(U[:,1])<0.05)).sum()>50 else float('nan'):+.4f} rad/s")
print("stiction test: actual |w| when the commanded |w| is small (0.02..0.12), in place (|v cmd| < 0.03): mean actual/cmd by cmd bin")
for a, b in ((0.02, 0.05), (0.05, 0.08), (0.08, 0.12), (0.12, 0.2), (0.2, 0.4)):
    m = (np.abs(uv) < 0.03) & (np.abs(xw) >= a) & (np.abs(xw) < b) & mv
    if m.sum() > 50: print(f"   |cmd| {a:.2f}-{b:.2f}: n {m.sum()*dt:4.0f} s, actual {np.mean(np.abs(y[m])):.3f}, cmd {np.mean(np.abs(xw[m])):.3f}, ratio {np.mean(np.abs(y[m]))/np.mean(np.abs(xw[m])):.2f}, share with |actual| < 0.02: {100*np.mean(np.abs(y[m])<0.02):.0f} %")
