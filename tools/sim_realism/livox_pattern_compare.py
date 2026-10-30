"""Scene-independent comparison of Mid-360 scan patterns (sensor-frame directions of the VALID points, 1 deg cells over azimuth x zenith).
usage: livox_pattern_compare.py NAME=KIND:PATH[:TOPIC[:T0]] ...     KIND: bag (raw frames, TOPIC, window start T0 s) | csv (official mid360.csv, random 68 % valid)
Per stream, over non-overlapping windows of 2 s:
  cov(T)   = cells hit in the first T s / cells hit in the whole 2 s window   (T = 0.1 0.2 0.5 1.0 s)   -> speed at which the pattern fills the scene's own valid cone
  ovl(lag) = cells of frame k also hit in frame k+lag / cells of frame k      (lag = 1 2 5 10 frames)  -> repetitiveness: the exact 10 Hz CSV returns to itself, a drifting vertical phase does not
Mean over windows (+-std)."""
import sys

import numpy as np

sys.path.insert(0, "/home/narcis/Documents/GitHub/SocialTech/notes/sim_realism")
import livox_io as io

NA, NZ = 360, 70


def cells(az_deg, ze_deg):
    return (az_deg % 360).astype(int) * NZ + np.clip((ze_deg - 30).astype(int), 0, NZ - 1)


def load_bag(path, topic, t0, sec):
    out = []
    for rt, st, d in io.frames(path, topic, t0, t0 + sec):
        rho = np.sqrt(d["x"].astype(float) ** 2 + d["y"].astype(float) ** 2 + d["z"].astype(float) ** 2); ok = rho > 0.05
        out.append((rt, cells(np.degrees(np.arctan2(d["y"][ok], d["x"][ok])), np.degrees(np.arccos(d["z"][ok] / rho[ok])))))
    return out


def load_csv(path, sec, share=0.68):
    a = np.loadtxt(path, delimiter=",", skiprows=1); rng = np.random.default_rng(0); out = []
    nper = 20000
    for k in range(int(sec * 10)):
        r = a[(k * nper) % len(a):(k * nper) % len(a) + nper]; ok = rng.random(len(r)) < share
        out.append((k * 0.1, cells(r[ok, 1], r[ok, 2])))
    return out


def metrics(fr):
    W = 20; covs, ovls = {T: [] for T in (1, 2, 5, 10)}, {L: [] for L in (1, 2, 5, 10)}; dens = []
    for w in range(0, len(fr) - W + 1, W):
        S = [np.unique(c) for _, c in fr[w:w + W]]; U = np.unique(np.concatenate(S))
        for T in covs: covs[T].append(len(np.unique(np.concatenate(S[:T]))) / len(U))
        for L in ovls: ovls[L] += [len(np.intersect1d(S[k], S[k + L], assume_unique=True)) / len(S[k]) for k in range(0, W - L, 2)]
        dens.append(len(S[0]) / len(U))
    return covs, ovls, dens


rows = []
for spec in sys.argv[1:]:
    name, rest = spec.split("=", 1); parts = rest.split(":"); kind = parts[0]
    fr = load_csv(parts[1], 6.0) if kind == "csv" else load_bag(parts[1], parts[2], float(parts[3]) if len(parts) > 3 else 5.0, 12.0)
    rows.append((name, len(fr), *metrics(fr)))
print(f"{'stream':26s}{'frames':>7s} | " + " ".join(f"cov({t/10:.1f}s)" .rjust(10) for t in (1, 2, 5, 10)) + " | " + " ".join(f"ovl(+{l})".rjust(10) for l in (1, 2, 5, 10)))
for name, n, covs, ovls, dens in rows:
    print(f"{name:26s}{n:7d} | " + " ".join(f"{np.mean(covs[t]):.3f}±{np.std(covs[t]):.3f}".rjust(10) for t in covs) + " | " + " ".join(f"{np.mean(ovls[l]):.3f}".rjust(10) for l in ovls))
