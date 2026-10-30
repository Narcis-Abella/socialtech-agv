"""Per-unit Mid-360 scan-pattern table from real raw frames (CustomMsg or PointCloud2 of the driver).
usage: livox_build_pattern.py BAG_DIR TOPIC T0 SECONDS F_Z_HZ OUT.pat

Why a table: the azimuth law is not recoverable in closed form (vertical phase alone leaves 18 deg rms, the (f_a, f_z) torus predicts it with 11 deg median error;
see calibracion_resultados.md). The table keeps, for every point of SECONDS/0.1 consecutive real frames, its time offset in the frame, line (0..3), azimuth and zenith.
Invalid points (the driver zeroes x,y,z, direction unknown) are filled: residual against a per-line model (azimuth: 360 f_a t, zenith: 2 harmonics at f_z) is interpolated in time.

.pat layout (little endian): magic 'MIDP', u32 version=1, u32 n_frames, u32 n_points, f64 f_z, f64 f_a, f64 frame_period, u32 frame_start[n_frames+1],
then arrays of n_points: f32 t_off [s from the first point of the frame], u16 az [deg*65536/360], u16 ze [deg*65536/180], u8 line.
"""
import struct
import sys

import numpy as np

sys.path.insert(0, "/home/narcis/Documents/GitHub/SocialTech/notes/sim_realism")
import livox_io as io

bag, topic, T0, SEC, FZ, out = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5]), sys.argv[6]
FRAMES = []
for rt, st, d in io.frames(bag, topic, T0, T0 + SEC):
    rho = np.sqrt(d["x"].astype(float) ** 2 + d["y"].astype(float) ** 2 + d["z"].astype(float) ** 2)
    FRAMES.append(dict(t=d["t"].copy(), line=d["line"].astype(np.uint8), rho=rho, az=np.degrees(np.arctan2(d["y"], d["x"])) % 360, ze=np.degrees(np.arccos(np.clip(d["z"] / np.maximum(rho, 1e-9), -1, 1)))))
nf = len(FRAMES); tt0 = FRAMES[0]["t"].min()
t = np.concatenate([f["t"] for f in FRAMES]) - tt0; line = np.concatenate([f["line"] for f in FRAMES]); az = np.concatenate([f["az"] for f in FRAMES]); ze = np.concatenate([f["ze"] for f in FRAMES]); ok = np.concatenate([f["rho"] for f in FRAMES]) > 0.05
fid = np.concatenate([np.full(len(f["t"]), i) for i, f in enumerate(FRAMES)])
print(f"{nf} frames, {len(t)} points, {ok.mean()*100:.0f} % valid, span {t.max():.2f} s")

# f_a: circular periodogram refined on the valid points of line 0 (the azimuth is 360 f_a t + offset + bounded modulation)
m0 = ok & (line == 0)
def coh(f): return abs(np.mean(np.exp(1j * (np.radians(az[m0]) - 2 * np.pi * f * t[m0]))))
fs = np.arange(180.5, 181.7, 0.002); f0 = fs[int(np.argmax([coh(f) for f in fs]))]; fs = np.arange(f0 - 0.004, f0 + 0.004, 0.0001); FA = float(fs[int(np.argmax([coh(f) for f in fs]))])
print(f"f_a {FA:.4f} Hz (coherence {coh(FA):.3f}), f_z {FZ:.4f} Hz given")

az_f = np.empty_like(az); ze_f = np.empty_like(ze)
w = 2 * np.pi * FZ * t
for ln in range(4):
    m = line == ln; v = m & ok; idx = np.where(m)[0]
    X = np.column_stack([np.sin(w), np.cos(w), np.sin(2 * w), np.cos(2 * w), np.ones_like(w)])
    beta, *_ = np.linalg.lstsq(X[v], ze[v], rcond=None); zres = ze - X @ beta
    off = np.degrees(np.angle(np.mean(np.exp(1j * (np.radians(az[v]) - 2 * np.pi * FA * t[v])))))
    ares = (az - 360 * FA * t - off + 180) % 360 - 180                                   # wrapped residual, |.| <~ 90 deg
    tv = t[v]; o = np.argsort(tv)                                                         # points are in time order within a line already; sort for np.interp
    ze_f[idx] = np.where(ok[idx], ze[idx], X[idx] @ beta + np.interp(t[idx], tv[o], zres[v][o]))
    az_f[idx] = np.where(ok[idx], az[idx], (360 * FA * t[idx] + off + np.interp(t[idx], tv[o], ares[v][o])) % 360)
    print(f"line {ln}: zenith model rms {np.sqrt(np.mean(zres[v]**2)):.2f} deg, azimuth residual rms {np.sqrt(np.mean(np.radians(ares[v])**2))*57.3:.1f} deg")

# per-frame time offset from the first point of the frame
toff = np.empty(len(t), np.float32); starts = np.zeros(nf + 1, np.uint32)
for i in range(nf):
    s = np.where(fid == i)[0]; starts[i + 1] = starts[i] + len(s); toff[s] = (t[s] - t[s].min()).astype(np.float32)
with open(out, "wb") as f:
    f.write(b"MIDP"); f.write(struct.pack("<III", 1, nf, len(t))); f.write(struct.pack("<ddd", FZ, FA, 0.1)); f.write(starts.tobytes())
    f.write(toff.tobytes()); f.write(np.round((az_f % 360) * 65536 / 360).astype(np.uint32).astype(np.uint16).tobytes()); f.write(np.round(ze_f * 65536 / 180).clip(0, 65535).astype(np.uint16).tobytes()); f.write(line.astype(np.uint8).tobytes())
print("wrote", out, f"{(12+28+4*(nf+1)+len(t)*9)/1e6:.1f} MB")
