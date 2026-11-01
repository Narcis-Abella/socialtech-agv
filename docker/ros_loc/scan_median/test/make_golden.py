"""Writes test/golden.txt: random points (x y z in the target frame, z = height above the floor) and the ranges notes/nav_real_scripts/scan_study.py to_scan gives for three bin rules.
to_scan reproduced the recorded /scan from the raw cloud (93-95 % of beams valid in both, median 16-36 mm), so it is the reference the C++ Binner is checked against.
usage: python3 make_golden.py (from anywhere)"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", ".."))
import scan_study as ss

rng = np.random.default_rng(7)
n = 4000; ang = rng.uniform(-np.pi, np.pi, n); r = np.where(rng.random(n) < 0.9, rng.uniform(0.2, 12.0, n), rng.uniform(0.02, 80.0, n)); z = rng.uniform(-0.3, 2.2, n)
x, y = r * np.cos(ang), r * np.sin(ang)
with open(os.path.join(HERE, "golden.txt"), "w") as f:
    f.write(f"{n}\n")
    for a, b, c in zip(x, y, z): f.write(f"{a!r} {b!r} {c!r}\n")
    for q, rmin in ((0.0, 0.5), (0.5, 0.1), (0.25, 0.1)):
        s, _ = ss.to_scan(x, y, z, 0.0, blind3d=0.0, rmin=rmin, rmax=60.0, every=1, how=q)
        f.write(f"case {q} {rmin}\n" + " ".join(repr(float(v)) for v in s) + "\n")
print("golden.txt written")
