"""The scan pattern of the REAL Mid-360 from a raw bag, and whether the CSV of Livox-SDK/livox_laser_simulation (mid360.csv) is that pattern. usage: livox_pattern_real.py BAG_DIR T0 CSV
The direction of each beam (azimuth, zenith in the sensor frame) does not depend on the scene or on the motion, only its range does, so any stretch of frames gives the pattern. Prints: points per frame, offset_time structure,
lines, zenith/azimuth distribution; periodicity (how far apart in time two frames must be to share directions); and the best alignment of the real points with the CSV (cyclic offset search) with its angular residual."""
import sys

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, "/home/narcis/Documents/GitHub/SocialTech/notes/nav_real_scripts")
import livox_custom as lc

bag, T0, csv = sys.argv[1], float(sys.argv[2]), sys.argv[3]
F = []
for rt, st, p in lc.frames(bag, T0, T0 + 8.0):
    rho = np.sqrt(p["x"].astype(float) ** 2 + p["y"] ** 2 + p["z"] ** 2); ok = rho > 0.05
    q = p[ok]; rho = rho[ok]; az = np.degrees(np.arctan2(q["y"], q["x"])) % 360.0; zen = np.degrees(np.arccos(np.clip(q["z"] / rho, -1, 1)))
    F.append(dict(t=q["t"].astype(np.int64), az=az, zen=zen, line=q["line"], tag=q["tag"], rho=rho, n_all=len(p), stamp=st))
print(f"{len(F)} frames from {T0:.0f} s; points per frame (valid / message): {np.mean([len(f['t']) for f in F]):.0f} / {np.mean([f['n_all'] for f in F]):.0f}; frame period {np.median(np.diff([f['stamp'] for f in F]))*1000:.1f} ms")
f0 = F[2]; o = np.argsort(f0["t"]); t = f0["t"][o]; print(f"offset_time of a frame: first {t[0]/1e6:.2f} ms, last {t[-1]/1e6:.2f} ms; median step {np.median(np.diff(t))/1000:.2f} us, min step {np.min(np.diff(t))/1000:.2f} us")
print("lines (the 4 laser channels?) share:", {int(l): round(float((f0['line'] == l).mean()), 3) for l in np.unique(f0['line'])}, "| tags:", {int(l): round(float((f0['tag'] == l).mean()), 3) for l in np.unique(f0['tag'])})
allz = np.concatenate([f["zen"] for f in F]); h, e = np.histogram(allz, bins=np.arange(30, 100, 5)); print("zenith histogram (deg: share):", {f"{int(a)}-{int(a)+5}": round(float(x) / len(allz), 3) for a, x in zip(e[:-1], h)}, " elevation = 90 - zenith")
print(f"zenith range of the points: {allz.min():.1f}..{allz.max():.1f} deg -> elevation {90-allz.max():.1f}..{90-allz.min():.1f} deg")
# periodicity: directions of frame k against those of frame k+m
def unit(f): a, z = np.radians(f["az"]), np.radians(f["zen"]); return np.c_[np.sin(z) * np.cos(a), np.sin(z) * np.sin(a), np.cos(z)]
tree = [cKDTree(unit(f)) for f in F]; base = 0
print("mean angular distance (deg) from each point of frame 0 to the nearest direction of frame m (a repeating pattern would give ~0 at some m):")
res = {}
for m in range(1, min(len(F), 80)):
    d, _ = tree[m].query(unit(F[base])[::10]); res[m] = np.degrees(2 * np.arcsin(np.minimum(d / 2, 1))).mean()
print("  ", {m: round(v, 3) for m, v in list(res.items())[:12]}, "... best m:", min(res, key=res.get), round(min(res.values()), 3))
# alignment with the CSV (cyclic): rows i = time i*5 us
C = np.loadtxt(csv, delimiter=",", skiprows=1); caz, czen = C[:, 1], C[:, 2]; N = len(C); print(f"CSV: {N} rows ({N/200000:.1f} s at 200 kpts/s); zenith {czen.min():.1f}..{czen.max():.1f}; azimuth {caz.min():.1f}..{caz.max():.1f}")
sel = np.where(f0["t"] < 30_000_000)[0][::6][:300]; idx_rel = np.round(f0["t"][sel] / 5000.0).astype(np.int64); az_r, zen_r = f0["az"][sel], f0["zen"][sel]
best = (1e9, 0)
for s0 in range(0, N, 20000):
    S = np.arange(s0, min(s0 + 20000, N))[:, None]; rows = (S + idx_rel[None, :]) % N
    da = np.abs((caz[rows] - az_r[None, :] + 180) % 360 - 180); dz = np.abs(czen[rows] - zen_r[None, :]); e = np.sqrt((da * np.sin(np.radians(zen_r))[None, :]) ** 2 + dz ** 2).mean(1)
    k = int(np.argmin(e))
    if e[k] < best[0]: best = (float(e[k]), int(S[k, 0]))
print(f"best cyclic alignment of the first 30 ms of frame 2 with the CSV: start row {best[1]}, mean angular error {best[0]:.3f} deg (random alignment would give ~{np.degrees(1)*0.0+25:.0f} deg)")
