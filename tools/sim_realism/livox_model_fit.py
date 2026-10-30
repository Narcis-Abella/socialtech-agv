"""Closed-form Livox Mid-360 scan model on the SLOT clock (not the reported point time). usage:
  livox_model_fit.py fit  BAG TOPIC T0 SEC OUT.json          fit shape + unit constants on [T0, T0+SEC]
  livox_model_fit.py eval MODEL.json BAG TOPIC T0 SEC [--full] re-estimate only the per-recording constants (f_a, f_z, rotor phase, oscillator phase; --full also zenith amplitude
                                                             and centre) on the first 1 s of the span, keep the shape of MODEL, report the error on the rest
  livox_model_fit.py emit MODEL.json N OUT.npy               directions of slots 0..N-1: columns t_s, line, azimuth_deg, zenith_deg
  livox_model_fit.py --selftest
Model (slot k, line l = k % 4, t = k * 5 us; rotor angle a = phi0 + 2 pi f_a t; oscillator phase z = psi + 2 pi f_z t):
  azimuth_l = deg(a) + l * 1 deg + B(a, z) . c_az,  zenith_l = B(a, z) . c_ze,  B = [onehot(l), harmonics 1..3 of z, harmonics 1..3 of a, onehot(l) x (sin a, cos a)]  (24 terms each).
Why the slot clock: the driver's point times are packet time + linear interpolation, and the packet times (96 points) jitter ~0.5 ms rms against the true firing
(a uniform 5.000 us per point, 480.00 us per packet), which at 65 deg/ms of rotor speed is the +-27 deg azimuth 'modulation'. Needs every packet of the span (no drops)."""
import json
import sys

import numpy as np

T_SLOT = 5e-6
NH = 3


def wrap(a): return (a + 180) % 360 - 180


def basis(a, z, line):
    oh = np.eye(4)[line]; cols = [oh]
    for k in range(1, NH + 1): cols += [np.sin(k * z)[:, None], np.cos(k * z)[:, None]]
    for k in range(1, NH + 1): cols += [np.sin(k * a)[:, None], np.cos(k * a)[:, None]]
    return np.hstack(cols + [oh * np.sin(a)[:, None], oh * np.cos(a)[:, None]])


def zen_fund(s, ze, line, fz):
    """per-line centre + fundamental at fz -> (rms, amplitude, phase psi so that zenith ~ c + A sin(2 pi fz s + psi), centres)"""
    w = 2 * np.pi * fz * s; X = np.hstack([np.eye(4)[line], np.sin(w)[:, None], np.cos(w)[:, None], np.sin(2 * w)[:, None], np.cos(2 * w)[:, None]])
    b = np.linalg.lstsq(X, ze, rcond=None)[0]; r = ze - X @ b; return float(np.sqrt(np.mean(r ** 2))), float(np.hypot(b[4], b[5])), float(np.arctan2(b[5], b[4])), b[:4]


def unit_constants(s, az, ze, line, fz0=9.85, fa0=181.0):
    """per-recording constants from a span (>= ~1 s): f_z by a scan of the zenith fit, f_a and phi0 by a circular periodogram of the line-0 azimuth"""
    sub = slice(None, None, max(1, len(s) // 60000))
    fs = np.arange(fz0 - 0.35, fz0 + 0.35, 0.002); fz = fs[np.argmin([zen_fund(s[sub], ze[sub], line[sub], f)[0] for f in fs])]
    fs = np.arange(fz - 0.003, fz + 0.003, 0.0001); fz = float(fs[np.argmin([zen_fund(s[sub], ze[sub], line[sub], f)[0] for f in fs])])
    _, A, psi, cen = zen_fund(s, ze, line, fz)
    m = line == 0; ss, aa = s[m][::4], np.radians(az[m][::4])
    def coh(f): return abs(np.mean(np.exp(1j * (aa - 2 * np.pi * f * ss))))
    for lo, hi, st in ((fa0 - 1.5, fa0 + 1.5, 0.01), (-0.02, 0.02, 0.0002), (-0.0003, 0.0003, 0.00001)):
        fg = np.arange(lo, hi, st) + (0 if lo == fa0 - 1.5 else fa); fa = float(fg[np.argmax([coh(f) for f in fg])])
    phi0 = float(np.angle(np.mean(np.exp(1j * (np.radians(az[m]) - 2 * np.pi * fa * s[m])))))
    return dict(f_a_hz=fa, f_z_hz=fz, phi0_rad=phi0, psi_rad=psi, z_amp_deg=A, z_centre_deg=[float(c) for c in cen])


def phases(C, s): return C["phi0_rad"] + 2 * np.pi * C["f_a_hz"] * s, C["psi_rad"] + 2 * np.pi * C["f_z_hz"] * s


def az_target(C, s, az, line):
    """azimuth minus the uniform rotor angle and the 1 deg/line step, continuous (unwrapped in slot order)"""
    a, _ = phases(C, s); r = wrap(az - np.degrees(a) - line); return np.degrees(np.unwrap(np.radians(r)))


def fit_shape(C, s, az, ze, line):
    a, z = phases(C, s); X = basis(a, z, line)
    return dict(c_az=np.linalg.lstsq(X, az_target(C, s, az, line), rcond=None)[0].tolist(), c_ze=np.linalg.lstsq(X, ze, rcond=None)[0].tolist())


def refine(M, C, s, az, ze, line, it=4):
    """shape-aware estimate of the per-recording constants: the azimuth residual's linear trend corrects (phi0, f_a); the zenith residual,
    linearised in (sin z, cos z, s cos z), corrects (amplitude, psi, f_z). The periodogram estimates alone are biased by the +-25 deg shape terms."""
    C = dict(C)
    for _ in range(it):
        pa, pz = predict(M, C, s, line); ra = np.degrees(np.unwrap(np.radians(wrap(az - pa)))); b = np.polyfit(s, ra, 1)
        C["phi0_rad"] += np.radians(b[1]); C["f_a_hz"] += b[0] / 360
        _, z = phases(C, s); X = np.column_stack([np.sin(z), np.cos(z), s * np.cos(z)]); A = C["z_amp_deg"]; k = np.linalg.lstsq(X, ze - pz, rcond=None)[0]
        C["psi_rad"] += k[1] / A; C["f_z_hz"] += k[2] / (2 * np.pi * A)
    return C


def predict(M, C, s, line, full=False):
    a, z = phases(C, s); X = basis(a, z, line); caz, cze = np.array(M["c_az"]), np.array(M["c_ze"]).copy()
    if full:   # rescale the zenith shape to this recording's amplitude and centres (line 0 index terms 0..3, fundamental terms 4..5)
        cze[4:4 + 2 * NH] *= C["z_amp_deg"] / M["unit"]["z_amp_deg"]; cze[:4] += np.array(C["z_centre_deg"]) - np.array(M["unit"]["z_centre_deg"])
    return (np.degrees(a) + line + X @ caz) % 360, X @ cze


def err_arrays(C, M, s, az, ze, line, full=False):
    pa, pz = predict(M, C, s, line, full); da = wrap(az - pa) * np.sin(np.radians(ze))
    return da - np.median(da), ze - pz   # the absolute rotor phase is a per-recording constant; its estimate absorbs the mean shape offset


def fmt(da, dz):
    q = lambda e: f"median {np.median(abs(e)):.3f} p90 {np.percentile(abs(e), 90):.3f} p99 {np.percentile(abs(e), 99):.2f}"
    return f"azimuth x sin(zenith) {q(da)} | zenith {q(dz)} deg (N {len(da)})"


def errors(C, M, s, az, ze, line, full=False): return fmt(*err_arrays(C, M, s, az, ze, line, full))


def load(bag, topic, t0, sec):
    sys.path.insert(0, "/home/narcis/Documents/GitHub/SocialTech/notes/sim_realism"); import livox_io as io
    S, L, AZ, ZE, V, P = [], [], [], [], [], []; g = 0
    for _, _, d in io.frames(bag, topic, t0, t0 + sec):
        n = len(d["t"]); assert n % 96 == 0 and (d["line"] == np.arange(n) % 4).all(), "message is not whole packets of 4-line groups"
        x, y, z = (d[c].astype(float) for c in "xyz"); rho = np.sqrt(x * x + y * y + z * z)
        S.append(np.arange(g, g + n) * T_SLOT); P.append(d["t"]); L.append(d["line"].astype(int)); V.append((rho > 0.05) & (d["tag"] == 0)); g += n
        AZ.append(np.degrees(np.arctan2(y, x)) % 360); ZE.append(np.degrees(np.arccos(np.clip(z / np.maximum(rho, 1e-9), -1, 1))))
    s, tr, l, v, az, ze = (np.concatenate(x) for x in (S, P, L, V, AZ, ZE))
    first = np.arange(0, len(s), 96); e = tr[first] - s[first]; e -= np.median(e)
    # a dropped packet shifts the slot clock by 480 us = a 31 deg azimuth jump between consecutive packets (normal packet-to-packet change < 5 deg, p99)
    m = v & (l == 0); pk = (s[m] / (96 * T_SLOT)).astype(int); up, inv = np.unique(pk, return_inverse=True); r = np.exp(1j * np.radians(az[m] - 360 * 181.0 * s[m]))
    pm = np.degrees(np.angle(np.bincount(inv, r.real) + 1j * np.bincount(inv, r.imag))); jumps = int((np.abs(wrap(np.diff(pm))) > 15).sum())
    print(f"{len(s)} slots ({len(s) * T_SLOT:.2f} s), valid {v.mean():.2f}; reported packet time - slot clock: std {e.std() * 1e6:.0f} us; azimuth jumps > 15 deg between packets {jumps}"
          + ("  WARNING: dropped packets, the slot clock is broken" if jumps else ""))
    return s[v], az[v], ze[v], l[v]


def selftest():
    rng = np.random.default_rng(1); C = dict(f_a_hz=181.0, f_z_hz=9.9, phi0_rad=0.3, psi_rad=-1.0, z_amp_deg=26.0, z_centre_deg=[68.0, 68.5, 69.0, 69.5])
    s = np.arange(400000) * T_SLOT; line = np.arange(len(s)) % 4; a, z = phases(C, s)
    ca = rng.normal(0, 1, 24); ca[:4] = 0; cz = np.zeros(24); cz[:4] = C["z_centre_deg"]; cz[4] = 26.0; cz[6:8] = 0.8
    X = basis(a, z, line); az = (np.degrees(a) + line + X @ ca) % 360; ze = X @ cz
    U = unit_constants(s, az, ze, line); M = dict(unit=U, **fit_shape(U, s, az, ze, line)); pa, pz = predict(M, U, s, line)
    assert abs(U["f_z_hz"] - 9.9) < 1e-3 and abs(U["f_a_hz"] - 181.0) < 1e-3, U
    # f_a comes from a 1e-5 Hz grid: ~0.02 deg over 2 s, far below the 0.3 deg data floor
    assert np.abs(wrap(az - pa)).max() < 0.05 and np.abs(ze - pz).max() < 1e-3, (np.abs(wrap(az - pa)).max(), np.abs(ze - pz).max())
    print("selftest ok")


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "--selftest": selftest()
    elif cmd == "fit":
        bag, topic, t0, sec, out = sys.argv[2], sys.argv[3], float(sys.argv[4]), float(sys.argv[5]), sys.argv[6]
        s, az, ze, l = load(bag, topic, t0, sec); C = unit_constants(s, az, ze, l)
        for _ in range(3): M = dict(unit=C, bag=bag, topic=topic, t0=t0, sec=sec, **fit_shape(C, s, az, ze, l)); C = refine(M, C, s, az, ze, l)
        M = dict(unit=C, bag=bag, topic=topic, t0=t0, sec=sec, **fit_shape(C, s, az, ze, l))
        print("unit constants:", {k: (round(v, 5) if isinstance(v, float) else v) for k, v in C.items()}); print("in-sample:", errors(C, M, s, az, ze, l))
        json.dump(M, open(out, "w"), indent=1)
    elif cmd == "eval":
        M = json.load(open(sys.argv[2])); s, az, ze, l = load(sys.argv[3], sys.argv[4], float(sys.argv[5]), float(sys.argv[6])); full = "--full" in sys.argv
        k = s < s[0] + 1.0; C = refine(M, unit_constants(s[k], az[k], ze[k], l[k], M["unit"]["f_z_hz"], M["unit"]["f_a_hz"]), s[k], az[k], ze[k], l[k])
        print("constants re-estimated on the first 1 s:", {k_: (round(v, 5) if isinstance(v, float) else v) for k_, v in C.items()})
        print(f"held-out ({'+amplitude/centre' if full else '4 constants'}):", errors(C, M, s[~k], az[~k], ze[~k], l[~k], full))
        # shape error alone: the 4 constants re-estimated in every 0.5 s window (removes the rotor/oscillator phase wander, which is a random process)
        DA, DZ = [], []
        for w0 in np.arange(s[~k][0], s[-1] - 0.25, 0.5):
            w = (s >= w0) & (s < w0 + 0.5); Cw = refine(M, C, s[w], az[w], ze[w], l[w]); da, dz = err_arrays(Cw, M, s[w], az[w], ze[w], l[w], full); DA.append(da); DZ.append(dz)
        print("held-out, constants per 0.5 s window:", fmt(np.concatenate(DA), np.concatenate(DZ)))
    elif cmd == "emit":
        M = json.load(open(sys.argv[2])); n = int(sys.argv[3]); s = np.arange(n) * T_SLOT; l = np.arange(n) % 4; pa, pz = predict(M, M["unit"], s, l)
        np.save(sys.argv[4], np.column_stack([s, l, pa, pz])); print(f"{n} slots -> {sys.argv[4]}")
