"""Ground-truth-free flatness diagnostic for GLIM dumps: fit the dominant horizontal plane of every submap.
Key outputs: floor_global_slope (deg) = tilt of the whole floor from the world horizontal (one rotation, fixable by one global
rotation) and floor_resid_std / floor_resid_range (m) = how far the floor points are from that single plane (real unevenness
or z drift). floor_drift_max = angle between a submap's floor normal and the first one's (noisy on small floor patches).

Assumes the floor is flat, so the plane's tilt in the world frame is drift. Two plane classes per submap:
  floor   = points whose normal faces up   (the sensor sees the floor from above)
  ceiling = points whose normal faces down (the Mid-360 looks mostly upward, so the ceiling dominates indoors)
Ceiling and floor are parallel in a building: both give the tilt, and their disagreement is a consistency check.
Only the floor height is meaningful for z drift (ceilings change height); the per-submap height under the origin mixes tilt with position, use the global fit.

Dump layout (GLIM): <run>/<submap id>/{data.txt, points_compact.bin, normals_compact.bin}; points and normals are
float32 xyz in the submap origin frame, T_world_origin is the first matrix in data.txt.
slope_x / slope_y: degrees of dz/dx, dz/dy of the plane in the world frame; tilt: angle between the plane normal and world z.
usage: floor_tilt.py [-v] <run_dir>...      (-v: one row per submap)
"""
import sys
from pathlib import Path

import numpy as np

MIN_INLIERS = 300  # below this the plane is not reported (floor not seen)
TOL = 0.03         # m, RANSAC inlier distance
MAX_TILT = 10.0    # deg, candidate planes steeper than this are not floors/ceilings
ITERS = 300
SEARCH_MAX = 20000  # points used to score the RANSAC hypotheses


def load_submap(d):
    lines = (d / "data.txt").read_text().split("T_world_origin:")[1].strip().splitlines()[:4]
    T = np.array([[float(v) for v in row.split()] for row in lines])
    p = np.fromfile(d / "points_compact.bin", dtype=np.float32).reshape(-1, 3).astype(np.float64)
    n = np.fromfile(d / "normals_compact.bin", dtype=np.float32).reshape(-1, 3).astype(np.float64)
    return T, p @ T[:3, :3].T + T[:3, 3], n @ T[:3, :3].T


def fit_plane(p, rng, min_inliers=MIN_INLIERS):
    """RANSAC then least squares on the inliers. Returns (unit normal with z > 0, offset d with n.p = d, inlier count) or None."""
    if len(p) < 3:
        return None
    search = p if len(p) <= SEARCH_MAX else p[rng.choice(len(p), SEARCH_MAX, replace=False)]  # hypotheses are scored on a subsample
    idx = rng.integers(0, len(search), (ITERS, 3))
    a, b, c = search[idx[:, 0]], search[idx[:, 1]], search[idx[:, 2]]
    V = np.cross(b - a, c - a)
    norm = np.linalg.norm(V, axis=1)
    V = V / np.where(norm > 1e-9, norm, 1.0)[:, None] * np.where(V[:, 2] < 0, -1.0, 1.0)[:, None]
    ok = (norm > 1e-9) & (V[:, 2] >= np.cos(np.radians(MAX_TILT)))
    if not ok.any():
        return None
    V, off = V[ok], np.einsum("ij,ij->i", V[ok], a[ok])
    k = int(np.argmax((np.abs(search @ V.T - off) < TOL).sum(0)))
    best = np.abs(p @ V[k] - off[k]) < TOL
    if best.sum() < min_inliers:
        return None
    for _ in range(2):  # refit on the inliers, then re-select them with the refined plane
        q = p[best]
        centre = q.mean(0)
        v = np.linalg.svd(q - centre, full_matrices=False)[2][-1]
        v = v * np.sign(v[2])
        best = np.abs(p @ v - v @ centre) < TOL
    return v, float(v @ centre), int(best.sum())


def describe(plane, origin):
    if plane is None:
        return None
    v, d, n = plane
    return {
        "n": n,
        "tilt": float(np.degrees(np.arccos(np.clip(v[2], -1, 1)))),
        "slope_x": float(np.degrees(np.arctan(-v[0] / v[2]))),
        "slope_y": float(np.degrees(np.arctan(-v[1] / v[2]))),
        "z": float((d - v[0] * origin[0] - v[1] * origin[1]) / v[2]),  # plane height under the submap origin
        "normal": v,
    }


def analyse(run):
    """One dict per submap: {"id", "floor", "ceiling"} (a class is None when its plane was not seen)."""
    rng = np.random.default_rng(0)
    out = []
    for d in sorted(x for x in Path(run).iterdir() if x.is_dir() and x.name.isdigit()):
        T, p, n = load_submap(d)
        up, down = p[n[:, 2] > 0.9], p[n[:, 2] < -0.9]
        floor = fit_plane(up, rng) if len(up) >= 3 else None
        ceiling = fit_plane(down, rng) if len(down) >= 3 else None
        pts = up[np.abs(up @ floor[0] - floor[1]) < TOL] if floor else None  # world-frame floor points of this submap
        out.append({"id": d.name, "floor": describe(floor, T[:3, 3]), "ceiling": describe(ceiling, T[:3, 3]), "floor_pts": pts})
    return out


def angle(a, b):
    return float(np.degrees(np.arccos(np.clip(a @ b, -1, 1))))


def summary(results):
    """Drift = angle between a submap's plane normal and the first submap's (the start is assumed flat)."""
    s = {"n_submaps": len(results)}
    for kind in ("floor", "ceiling"):
        found = [r[kind] for r in results if r[kind]]
        s[f"{kind}_n"] = len(found)
        if found:
            tilts = [f["tilt"] for f in found]
            s[f"{kind}_tilt_med"], s[f"{kind}_tilt_max"] = float(np.median(tilts)), float(np.max(tilts))
            s[f"{kind}_drift_max"] = max(angle(f["normal"], found[0]["normal"]) for f in found)
    pts = [r["floor_pts"] for r in results if r["floor_pts"] is not None]
    if pts:  # is the whole floor ONE plane? (constant LiDAR height over a flat floor) and how tilted is it from the world horizontal?
        P = np.vstack(pts)
        A = np.c_[P[:, 0], P[:, 1], np.ones(len(P))]
        a, b, _ = np.linalg.lstsq(A, P[:, 2], rcond=None)[0]
        res = P[:, 2] - A @ np.linalg.lstsq(A, P[:, 2], rcond=None)[0]
        s["floor_global_slope"] = float(np.degrees(np.arctan(np.hypot(a, b))))
        s["floor_resid_std"] = float(res.std())
        s["floor_resid_range"] = float(np.percentile(res, 99) - np.percentile(res, 1))
    both = [angle(r["floor"]["normal"], r["ceiling"]["normal"]) for r in results if r["floor"] and r["ceiling"]]
    if both:
        s["floor_vs_ceiling_med"] = float(np.median(both))
    return s


def fmt(s, key, spec=".2f"):
    return format(s[key], spec) if key in s else "-"


def main(argv):
    verbose = "-v" in argv
    runs = [a for a in argv if a != "-v"]
    if not runs:
        sys.exit(__doc__)
    if not verbose:
        print("| Run | Submaps | Floor seen | Floor tilt med/max deg | Floor drift max deg | Floor global slope deg | Floor resid std / p1-p99 range cm | Ceiling seen | Ceiling tilt med/max deg | Ceiling drift max deg | Floor vs ceiling deg |")
        print("|---|---|---|---|---|---|---|---|---|---|---|")
    for run in runs:
        res = analyse(run)
        if verbose:
            print(f"# {run}\n| Submap | Floor n | Floor tilt | Floor slope x/y | Floor z | Ceiling n | Ceiling tilt | Ceiling slope x/y |\n|---|---|---|---|---|---|---|---|")
            for r in res:
                f, c = r["floor"], r["ceiling"]
                print(f"| {r['id']} | " + (f"{f['n']} | {f['tilt']:.2f} | {f['slope_x']:+.2f}/{f['slope_y']:+.2f} | {f['z']:+.3f}" if f else "- | - | - | -") + " | "
                      + (f"{c['n']} | {c['tilt']:.2f} | {c['slope_x']:+.2f}/{c['slope_y']:+.2f}" if c else "- | - | -") + " |")
            continue
        s = summary(res)
        print(f"| {run} | {s['n_submaps']} | {s['floor_n']} | {fmt(s, 'floor_tilt_med')}/{fmt(s, 'floor_tilt_max')} | {fmt(s, 'floor_drift_max')} | {fmt(s, 'floor_global_slope')} | "
              f"{fmt(s, 'floor_resid_std', '.4f')}/{fmt(s, 'floor_resid_range', '.4f')} m "
              f"| {s['ceiling_n']} | {fmt(s, 'ceiling_tilt_med')}/{fmt(s, 'ceiling_tilt_max')} | {fmt(s, 'ceiling_drift_max')} | {fmt(s, 'floor_vs_ceiling_med')} |")


if __name__ == "__main__":
    main(sys.argv[1:])
