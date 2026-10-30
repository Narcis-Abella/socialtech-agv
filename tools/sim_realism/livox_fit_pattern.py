"""Analytic model of the Mid-360 scan pattern fitted to raw frames of a real unit. usage: livox_fit_pattern.py BAG_DIR TOPIC T0 SECONDS OUT.json
zenith_l(t) = c_l + A_l sin(2 pi f_z t + p_l) + (2nd harmonic), azimuth_l(t) = a_l + 360 f_a t (mod 360), with l the laser line (4) and t the point time. f_z by a linear least-squares scan (sin, cos, centre per line), f_a by a circular periodogram
refined over growing spans. Reports the residuals of every valid tag-0 point, the slot period, the slots per frame and the share of valid slots."""
import json
import sys

import numpy as np

sys.path.insert(0, "/home/narcis/Documents/GitHub/SocialTech/notes/sim_realism")
import livox_io as io

bag, topic, T0, SEC, out = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4]), sys.argv[5]
T, L, AZ, ZE = [], [], [], []; slots, valid, steps = [], [], []
for rt, st, d in io.frames(bag, topic, T0, T0 + SEC):
    rho = np.sqrt(d["x"].astype(float) ** 2 + d["y"].astype(float) ** 2 + d["z"].astype(float) ** 2); v = rho > 0.05; slots.append(d["n_slots"]); valid.append(v.sum())
    o = np.argsort(d["t"]); ts = d["t"][o]; df = np.diff(ts); steps.append(np.median(df[(df > 2e-6) & (df < 9e-6)]) if ((df > 2e-6) & (df < 9e-6)).any() else np.nan)
    ok = v & (d["tag"] == 0); T.append(d["t"][ok]); L.append(d["line"][ok].astype(int)); AZ.append(np.degrees(np.arctan2(d["y"][ok], d["x"][ok])) % 360); ZE.append(np.degrees(np.arccos(np.clip(d["z"][ok] / rho[ok], -1, 1))))
t = np.concatenate(T); l = np.concatenate(L); az = np.concatenate(AZ); ze = np.concatenate(ZE); t0 = t.min(); t = t - t0
slot = float(np.nanmedian(steps)); print(f"{len(slots)} frames, {len(t)} valid tag-0 points over {t.max():.1f} s; slots/frame {np.mean(slots):.0f}, valid share {np.mean(valid)/np.mean(slots):.2f}; slot step (median of the 2-9 us gaps) {slot*1e6:.3f} us -> {1/slot/1e3:.1f} kslots/s")
rng = np.random.default_rng(0); idx = rng.choice(len(t), size=min(len(t), 250000), replace=False); ts_, ls_, zs_ = t[idx], l[idx], ze[idx]; oh = np.eye(4)[ls_]


def lin(f, per_line_amp):
    w = 2 * np.pi * f * ts_
    cols = [np.sin(w), np.cos(w), np.sin(2 * w), np.cos(2 * w)] if not per_line_amp else [np.sin(w)[:, None] * oh, np.cos(w)[:, None] * oh, np.sin(2 * w), np.cos(2 * w)]
    X = np.column_stack([c if c.ndim == 1 else c for c in cols[:0]] + [np.column_stack(cols[:2]) if not per_line_amp else np.hstack([cols[0], cols[1]]), np.column_stack(cols[2:]), oh]) if False else None
    if per_line_amp: X = np.hstack([cols[0], cols[1], np.column_stack(cols[2:]), oh])
    else: X = np.hstack([np.column_stack(cols), oh])
    beta, *_ = np.linalg.lstsq(X, zs_, rcond=None); r = zs_ - X @ beta; return float(np.sqrt(np.mean(r ** 2))), beta


best = None
for pla in (False, True):
    fs = np.arange(9.60, 10.20, 0.002); rms = np.array([lin(f, pla)[0] for f in fs]); f0 = fs[int(np.argmin(rms))]
    fs2 = np.arange(f0 - 0.004, f0 + 0.004, 0.0001); rms2 = np.array([lin(f, pla)[0] for f in fs2]); fz = float(fs2[int(np.argmin(rms2))]); r_, beta = lin(fz, pla)
    print(f"zenith, {'per-line amplitude' if pla else 'common amplitude'}: f_z {fz:.4f} Hz, residual RMS {r_:.3f} deg")
    if best is None or pla: best = (fz, r_, beta, pla)
fz, rz, beta, pla = best
if pla: amp = np.hypot(beta[0:4], beta[4:8]); ph = np.arctan2(beta[4:8], beta[0:4]); cen = beta[-4:]
else: amp = np.full(4, np.hypot(beta[0], beta[1])); ph = np.full(4, np.arctan2(beta[1], beta[0])); cen = beta[-4:]
print(f"   amplitude by line {np.round(amp, 2)} deg, phase by line {np.round(np.degrees(ph), 1)} deg, centre by line {np.round(cen, 2)} deg")
w = 2 * np.pi * fz * t; Xall = np.hstack([np.column_stack([np.sin(w)[:, None] * np.eye(4)[l], np.cos(w)[:, None] * np.eye(4)[l]]) if pla else np.column_stack([np.sin(w), np.cos(w)]), np.column_stack([np.sin(2 * w), np.cos(2 * w)]), np.eye(4)[l]])
rall = ze - Xall @ beta; print(f"   all points: residual RMS {np.sqrt(np.mean(rall**2)):.3f} deg, median |r| {np.median(np.abs(rall)):.3f}, p95 {np.percentile(np.abs(rall), 95):.2f}; zenith range {ze.min():.1f}..{ze.max():.1f}")


def circ(f, tt, aa): return abs(np.mean(np.exp(1j * (np.radians(aa) - 2 * np.pi * f * tt))))


m0 = np.where(l == 0)[0]; fbest = None
for span, step in ((0.4, 0.05), (3.0, 0.004), (SEC, 0.0003)):
    mm = m0[t[m0] < span]; ii = mm[:: max(1, len(mm) // 30000)]
    fg = np.arange(150, 220, step) if fbest is None else np.arange(fbest - 4 * step * (0.4 / span) - 0.3 if span < SEC else fbest - 0.05, fbest + 0.3 if span < SEC else fbest + 0.05, step)
    sc = np.array([circ(x, t[ii], az[ii]) for x in fg]); fbest = float(fg[int(np.argmax(sc))]); print(f"azimuth rotation, span {span:5.1f} s: {fbest:.4f} Hz (coherence {sc.max():.3f})")
fa = fbest; a_off = {}; res = {}
for ln in range(4):
    mm = l == ln; ph_ = np.angle(np.mean(np.exp(1j * (np.radians(az[mm]) - 2 * np.pi * fa * t[mm])))); a_off[ln] = float(np.degrees(ph_) % 360); r = (az[mm] - (a_off[ln] + 360 * fa * t[mm]) + 180) % 360 - 180; res[ln] = float(r.std())
print("azimuth offset by line (deg):", {k: round(v, 2) for k, v in a_off.items()}, "| residual std (deg):", {k: round(v, 3) for k, v in res.items()})
json.dump(dict(bag=bag, topic=topic, f_a_hz=fa, f_z_hz=fz, slot_period_s=slot, slots_per_frame=float(np.mean(slots)), valid_share=float(np.mean(valid) / np.mean(slots)), z_amp_deg=[float(x) for x in amp], z_phase_rad=[float(x) for x in ph], z_centre_deg=[float(x) for x in cen],
               z_second_harmonic=[float(x) for x in beta[8:10]] if pla else [float(x) for x in beta[2:4]], az_offset_deg=a_off, az_residual_std_deg=res, zen_residual_rms_deg=float(np.sqrt(np.mean(rall ** 2))), span_s=float(t.max())), open(out, "w"), indent=1)
