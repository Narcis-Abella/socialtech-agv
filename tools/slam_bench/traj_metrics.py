"""Ground-truth-free trajectory metrics for our own bags (flat floor, robot returns near its start).

Layout: <root>/<method>/<bag>/*.tum   (one TUM file per run)
Per run: poses, path length, end-to-start distance, z range, roll range, pitch range (a robot on a flat floor should barely change them,
so the ranges measure drift), and the largest position difference to the first run at matching timestamps when a method has several runs.
usage: traj_metrics.py <root>
"""
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def metrics(t):
    p = t[:, 1:4]
    eul = Rotation.from_quat(t[:, 4:8]).as_euler("xyz", degrees=True)
    roll, pitch = np.unwrap(np.radians(eul[:, 0])), np.unwrap(np.radians(eul[:, 1]))
    return {
        "poses": len(t),
        "path": float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()),
        "end2start": float(np.linalg.norm(p[-1] - p[0])),
        "z_range": float(np.ptp(p[:, 2])),
        "roll": float(np.degrees(np.ptp(roll))),
        "pitch": float(np.degrees(np.ptp(pitch))),
    }


def divergence(a, b):
    """Largest position difference at matching timestamps (each run starts at its own origin, so both are aligned at the first pose)."""
    idx = np.searchsorted(b[:, 0], a[:, 0])
    idx = np.clip(idx, 1, len(b) - 1)
    near = np.where(np.abs(b[idx - 1, 0] - a[:, 0]) < np.abs(b[idx, 0] - a[:, 0]), idx - 1, idx)
    ok = np.abs(b[near, 0] - a[:, 0]) < 0.05
    if ok.sum() < 10:
        return None
    pa, pb = a[ok, 1:4] - a[ok][0, 1:4], b[near[ok], 1:4] - b[near[ok]][0, 1:4]
    return float(np.linalg.norm(pa - pb, axis=1).max())


def main(root):
    root = Path(root)
    print("| Method | Bag | Runs | Poses | Path m | End-start m | z range m | Roll range deg | Pitch range deg | Max run divergence m |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for method in sorted(p.name for p in root.iterdir() if p.is_dir()):
        for bag_dir in sorted(p for p in (root / method).iterdir() if p.is_dir()):
            runs = [np.loadtxt(f) for f in sorted(bag_dir.glob("*.tum"))]
            runs = [r for r in runs if r.ndim == 2 and len(r) > 10]
            if not runs:
                continue
            ms = [metrics(r) for r in runs]
            med = {k: float(np.median([m[k] for m in ms])) for k in ms[0]}
            div = [d for r in runs[1:] if (d := divergence(runs[0], r)) is not None]
            print(f"| {method} | {bag_dir.name} | {len(runs)} | {med['poses']:.0f} | {med['path']:.1f} | {med['end2start']:.2f} | {med['z_range']:.2f} | "
                  f"{med['roll']:.1f} | {med['pitch']:.1f} | {max(div):.2f} |" if div else
                  f"| {method} | {bag_dir.name} | {len(runs)} | {med['poses']:.0f} | {med['path']:.1f} | {med['end2start']:.2f} | {med['z_range']:.2f} | "
                  f"{med['roll']:.1f} | {med['pitch']:.1f} | - |")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
