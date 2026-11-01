"""Is mid360.csv (Livox-SDK/livox_laser_simulation) the pattern of our Mid-360? usage: livox_csv_vs_real.py BAG_DIR T0 CSV
Matches the ZENITH of the real valid points of a frame (first 3 ms, slot index = offset_time / 4.9475 us) against every cyclic start row of the CSV (zenith does not depend on the azimuth convention); then, at the best start,
the azimuth: real = +/- csv + const. A pattern match shows as a mean zenith error of the order of the angular noise (~0.1 deg) at one start row, and a constant (mirrored or not) azimuth offset with a small spread."""
import sys

import numpy as np

sys.path.insert(0, "/home/narcis/Documents/GitHub/SocialTech/notes/nav_real_scripts")
import livox_custom as lc

bag, T0, csv = sys.argv[1], float(sys.argv[2]), sys.argv[3]
C = np.loadtxt(csv, delimiter=",", skiprows=1); caz, czen = C[:, 1], C[:, 2]; N = len(C); SLOT = 4.9475e3   # ns per slot
frames = [(st, p) for _, st, p in lc.frames(bag, T0, T0 + 6.0)]


def real_points(p):
    rho = np.sqrt(p["x"].astype(float) ** 2 + p["y"] ** 2 + p["z"] ** 2); ok = rho > 0.05
    az = np.degrees(np.arctan2(p["y"], p["x"])) % 360; zen = np.degrees(np.arccos(np.clip(p["z"] / np.maximum(rho, 1e-9), -1, 1)))
    return p["t"].astype(np.int64)[ok], az[ok], zen[ok]


def search(p, window_ns=3_000_000, step=4000):
    t, az, zen = real_points(p); m = t < window_ns; t, az, zen = t[m], az[m], zen[m]; k = np.round(t / SLOT).astype(np.int64); best = (1e9, 0)
    for s0 in range(0, N, step):
        S = np.arange(s0, min(s0 + step, N))[:, None]; e = np.abs(czen[(S + k[None, :]) % N] - zen[None, :]).mean(1); i = int(np.argmin(e))
        if e[i] < best[0]: best = (float(e[i]), int(S[i, 0]))
    return best, (t, az, zen, k)


(e0, s0), (t, az, zen, k) = search(frames[2][1])
print(f"frame 2: best start row {s0}, mean |zenith error| {e0:.3f} deg over {len(t)} points in 3 ms (chance level for this FOV would be ~12 deg)")
rows = (s0 + k) % N
for name, sign in (("same sense", 1), ("mirrored", -1)):
    d = (az - sign * caz[rows] + 180) % 360 - 180; mu = np.degrees(np.angle(np.mean(np.exp(1j * np.radians(d))))); r = (d - mu + 180) % 360 - 180
    print(f"   azimuth {name}: offset {mu % 360:.2f} deg, residual std {np.std(r):.3f} deg, share within 0.5 deg {100*np.mean(np.abs(r) < 0.5):.0f} %")
# a later frame: same pattern, and how many rows per second
for fi in (12, 32, 52):
    (e, s), _ = search(frames[fi][1]); dt = frames[fi][0] - frames[2][0]; ds = (s - s0) % N
    print(f"frame {fi}: best start row {s} (error {e:.3f} deg); {dt:.1f} s after frame 2 -> {ds} rows modulo {N} => {(ds + N*round(dt*200000/N) )/dt:.0f} rows/s if no extra cycles")
