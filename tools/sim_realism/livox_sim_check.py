"""Output check of the simulated Mid-360 (socialtech_livox_sim) against the pattern table it was built from and against a real bag.
usage: livox_sim_check.py SIM_BAG PATTERN.pat REAL_BAG REAL_TOPIC
SIM_BAG holds /livox/lidar (CustomMsg) and /livox/pc2 (PointCloud2, driver xfer_format 0). Prints one table: sim | real."""
import struct
import sys

import numpy as np

sys.path.insert(0, "/home/narcis/Documents/GitHub/SocialTech/notes/sim_realism")
import livox_io as io

sim, patf, real, rtopic = sys.argv[1:5]


def read_pat(p):
    b = open(p, "rb").read(); assert b[:4] == b"MIDP"; ver, nf, n = struct.unpack_from("<III", b, 4); fz, fa, per = struct.unpack_from("<ddd", b, 16); o = 40
    start = np.frombuffer(b, "<u4", nf + 1, o); o += 4 * (nf + 1); toff = np.frombuffer(b, "<f4", n, o); o += 4 * n
    az = np.frombuffer(b, "<u2", n, o); o += 2 * n; ze = np.frombuffer(b, "<u2", n, o); o += 2 * n; line = np.frombuffer(b, "u1", n, o)
    return dict(start=start, toff=toff, az=az * 360.0 / 65536, ze=ze * 180.0 / 65536, line=line, fz=fz, fa=fa, nf=nf)


def stats(fr):
    n, v, off, mono, lineok, tags, rmin, rmax, dts = [], [], [], True, True, set(), 1e9, 0, []
    for rt, st, d in fr:
        rho = np.sqrt(d["x"].astype(float) ** 2 + d["y"].astype(float) ** 2 + d["z"].astype(float) ** 2); ok = rho > 0
        n.append(len(rho)); v.append(ok.mean()); o = d["t"] - d["t"].min(); off.append(o.max()); mono &= bool((np.diff(d["t"]) >= -1e-9).all())
        lineok &= bool((d["line"] == (np.arange(len(rho)) % 4)).all()); tags |= set(np.unique(d["tag"]).tolist())
        if ok.any(): rmin = min(rmin, rho[ok].min()); rmax = max(rmax, rho[ok].max())
        dts.append(rt)
    return dict(frames=len(n), points=(np.mean(n), min(n), max(n)), valid=np.mean(v), offmax=np.mean(off), mono=mono, line_seq=lineok, tags=sorted(tags), rho=(rmin, rmax), rate=1 / np.median(np.diff(dts)))


pat = read_pat(patf)
C = list(io.frames(sim, "/livox/lidar")); P = list(io.frames(sim, "/livox/pc2"))
print(f"messages: custom {len(C)}, pc2 {len(P)}")
# 1) custom vs pc2: same stamps, points, times
nm = min(len(C), len(P)); worst = dict(xyz=0.0, t=0.0, line=0, n=0)
for (rc, sc, dc), (rp, sp, dp) in list(zip(C, P))[:nm]:
    if len(dc["x"]) != len(dp["x"]): worst["n"] += 1; continue
    worst["xyz"] = max(worst["xyz"], float(np.abs(dc["x"] - dp["x"]).max()), float(np.abs(dc["y"] - dp["y"]).max()), float(np.abs(dc["z"] - dp["z"]).max()))
    worst["t"] = max(worst["t"], float(np.abs(dc["t"] - dp["t"]).max())); worst["line"] += int((dc["line"] != dp["line"]).sum())
print("custom vs pc2: frames with different point count", worst["n"], " max |dxyz| %.2e m  max |dt| %.2e s  line mismatches %d" % (worst["xyz"], worst["t"], worst["line"]))
# 2) directions of the output equal the table, frame by frame. The recorder starts after the node: find the table frame of bag frame 0 first.
def frame_err(k0, d, k):
    rho = np.sqrt(d["x"].astype(float) ** 2 + d["y"].astype(float) ** 2 + d["z"].astype(float) ** 2); ok = rho > 0; kk = (k0 + k) % pat["nf"]; a, b = pat["start"][kk], pat["start"][kk + 1]
    if b - a != len(rho) or not ok.any(): return np.inf, np.inf
    az = np.degrees(np.arctan2(d["y"][ok], d["x"][ok])) % 360; ze = np.degrees(np.arccos(d["z"][ok] / rho[ok]))
    da = ((az - pat["az"][a:b][ok] + 180) % 360 - 180) * np.sin(np.radians(pat["ze"][a:b][ok])); dz = ze - pat["ze"][a:b][ok]
    return max(np.abs(da).max(), np.abs(dz).max()), np.abs((d["t"] - d["t"].min()) - (pat["toff"][a:b] - pat["toff"][a:b].min())).max()
k0 = min(range(pat["nf"]), key=lambda c: frame_err(c, C[0][2], 0)[0])
E = [frame_err(k0, d, k) for k, (rt, st, d) in enumerate(C)]
print(f"directions vs table (bag frame 0 = table frame {k0}, {len(E)} frames, wraps the table {len(E)//pat['nf']} time(s)): worst angular error {max(e[0] for e in E):.4f} deg, worst time-offset error {max(e[1] for e in E):.2e} s")
# 3) frame statistics, sim vs real
S, R = stats(C), stats(io.frames(real, rtopic, 5, 25))
print(f"{'':28s}{'sim':>26s}{'real':>26s}")
for key, f in (("frames", "{:d}"), ("rate", "{:.3f} Hz"), ("valid", "{:.3f}"), ("offmax", "{:.4f} s"), ("mono", "{}"), ("line_seq", "{}"), ("tags", "{}")):
    print(f"{key:28s}{f.format(S[key]):>26s}{f.format(R[key]):>26s}")
print(f"{'points/frame mean,min,max':28s}{'%.0f,%d,%d' % S['points']:>26s}{'%.0f,%d,%d' % R['points']:>26s}")
print(f"{'valid range min,max (m)':28s}{'%.2f,%.1f' % S['rho']:>26s}{'%.2f,%.1f' % R['rho']:>26s}")
