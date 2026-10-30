"""Is the beam direction of each Mid-360 laser line a fixed function of two phases, (f_a t, f_z t) mod 1? usage: livox_torus_model.py BAG_DIR TOPIC T0 SECONDS F_A F_Z [BINS=360]
Builds a (BINS x BINS) table per line from the first half of the frames (cell = circular mean of the azimuth, mean of the zenith) and predicts the directions of the second half (nearest cell and bilinear), reporting the error."""
import sys

import numpy as np

sys.path.insert(0, "/home/narcis/Documents/GitHub/SocialTech/notes/sim_realism")
import livox_io as io

bag, topic, T0, SEC, FA, FZ = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6]); B = int(sys.argv[7]) if len(sys.argv) > 7 else 360
T, L, AZ, ZE = [], [], [], []
for rt, st, d in io.frames(bag, topic, T0, T0 + SEC):
    rho = np.sqrt(d["x"].astype(float) ** 2 + d["y"].astype(float) ** 2 + d["z"].astype(float) ** 2); ok = (rho > 0.05) & (d["tag"] == 0)
    T.append(d["t"][ok]); L.append(d["line"][ok].astype(int)); AZ.append(np.degrees(np.arctan2(d["y"][ok], d["x"][ok])) % 360); ZE.append(np.degrees(np.arccos(np.clip(d["z"][ok] / rho[ok], -1, 1))))
t = np.concatenate(T); l = np.concatenate(L); az = np.concatenate(AZ); ze = np.concatenate(ZE); t = t - t.min(); half = SEC / 2
pa = (FA * t) % 1.0; pz = (FZ * t) % 1.0; tr = t < half; te = ~tr
print(f"{len(t)} points; train {tr.sum()}, test {te.sum()}; table {B}x{B} per line")


def build(mask, ln):
    ia = np.minimum((pa[mask & (l == ln)] * B).astype(int), B - 1); iz = np.minimum((pz[mask & (l == ln)] * B).astype(int), B - 1); k = ia * B + iz
    n = np.bincount(k, minlength=B * B).astype(float); s = np.exp(1j * np.radians(az[mask & (l == ln)])); c = np.bincount(k, weights=s.real, minlength=B * B) + 1j * np.bincount(k, weights=s.imag, minlength=B * B)
    zs = np.bincount(k, weights=ze[mask & (l == ln)], minlength=B * B); with_ = n > 0; A = np.full(B * B, np.nan); Z = np.full(B * B, np.nan)
    A[with_] = np.degrees(np.angle(c[with_])) % 360; Z[with_] = zs[with_] / n[with_]; return A.reshape(B, B), Z.reshape(B, B), (n.reshape(B, B))


rows = []
for ln in range(4):
    A, Z, N = build(tr, ln); m = te & (l == ln); ia = np.minimum((pa[m] * B).astype(int), B - 1); iz = np.minimum((pz[m] * B).astype(int), B - 1); ap = A[ia, iz]; zp = Z[ia, iz]
    have = np.isfinite(ap); da = ((az[m][have] - ap[have] + 180) % 360 - 180) * np.sin(np.radians(ze[m][have])); dz = ze[m][have] - zp[have]
    print(f"line {ln}: test points {m.sum()}, cell filled for {100*have.mean():.0f} %; azimuth error (x sin zen) median {np.median(np.abs(da)):.3f} p90 {np.percentile(np.abs(da),90):.3f} deg; zenith error median {np.median(np.abs(dz)):.3f} p90 {np.percentile(np.abs(dz),90):.3f} deg; table cells with data {100*np.mean(N>0):.0f} %")
