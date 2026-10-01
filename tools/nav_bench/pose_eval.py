"""Pose-error metrics for the nav bench, stage A: AMCL poses vs the Voxel-SLAM trajectory of the same bag, in the map frame.
usage:
  pose_eval.py init   --ref alidarState.txt --map map.yaml [--dist 1.0] [--dyaw 20]   -> "x y yaw_rad" (map frame) of the deliberately wrong initial pose
  pose_eval.py check  --ref alidarState.txt --map map.yaml [--ref-dt 0]   -> JSON: share of the reference path on free / occupied map cells (same run as the map?)
  pose_eval.py level  --ref alidarState.txt --map map.yaml   -> "qx qy qz qw" of R_L, the rotation from FAST-LIO2's world frame to the floor-levelled, wall-aligned frame
  pose_eval.py report --est amcl.tum --odom odom.tum --ref alidarState.txt --map map.yaml [--ref-dt 0] [thresholds]   -> JSON
Pose files: "t x y z qx qy qz qw [cov_xx cov_yy cov_yawyaw]" (record_poses.py); alidarState.txt has the same first 8 columns.
The map YAML must carry the `# T_map_world` comment block written by ply_to_map (p_map = T p_ply). numpy only (the robot image has no scipy)."""
import argparse
import json
import re

import numpy as np

# Provisional: only used to produce the first numbers; the acceptance thresholds are fixed with Narcis after the first baseline run.
CONV_POS, CONV_YAW_DEG, CONV_HOLD_S = 0.30, 8.0, 5.0
LOSS_POS, LOSS_S = 0.50, 3.0
JUMP_POS, JUMP_YAW_DEG = 0.15, 3.0
FREE = 250  # PGM value of a free cell (map_server trinary: free 254, unknown 205, occupied 0)


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def quat_to_mat(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_to_quat(R):
    """Unit quaternion (x, y, z, w), w >= 0, of a rotation matrix."""
    tr = np.trace(R)
    if tr > 0:
        k = 2 * np.sqrt(tr + 1)
        q = np.array([(R[2, 1] - R[1, 2]) / k, (R[0, 2] - R[2, 0]) / k, (R[1, 0] - R[0, 1]) / k, k / 4])
    else:
        i = int(np.argmax(np.diag(R)))
        j, l = (i + 1) % 3, (i + 2) % 3
        k = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[l, l])
        q = np.empty(4)
        q[i], q[3], q[j], q[l] = k / 4, (R[l, j] - R[j, l]) / k, (R[j, i] + R[i, j]) / k, (R[l, i] + R[i, l]) / k
    return q if q[3] >= 0 else -q


def read_poses(path):
    """(t (N,), xyz (N, 3), quat xyzw (N, 4), extra (N, k)); extra = columns after the 8th (covariances for AMCL files, junk for alidarState)."""
    a = np.loadtxt(path, ndmin=2)
    return a[:, 0], a[:, 1:4], a[:, 4:8], a[:, 8:]


def planar(xyz, quat, T=None):
    """(N, 3) x, y, yaw of the body; with T (4x4) the poses are first moved to the T target frame (p' = T p, R' = T_R R)."""
    out = np.empty((len(xyz), 3))
    for i, (p, q) in enumerate(zip(xyz, quat)):
        R = quat_to_mat(q)
        if T is not None:
            p, R = T[:3, :3] @ p + T[:3, 3], T[:3, :3] @ R
        out[i] = p[0], p[1], np.arctan2(R[1, 0], R[0, 0])
    return out


def read_map_yaml(path):
    """dict(resolution, origin (x, y), image (path next to the YAML), T_map_world (4x4)). ValueError without the T_map_world block: an identity
    would silently compare poses in the wrong frame."""
    from pathlib import Path
    txt = Path(path).read_text()
    res = float(re.search(r"^resolution:\s*(\S+)", txt, re.M).group(1))
    ox, oy = (float(v) for v in re.search(r"^origin:\s*\[([^\]]+)\]", txt, re.M).group(1).split(",")[:2])
    img = re.search(r"^image:\s*(\S+)", txt, re.M).group(1)
    rows = re.findall(r"^#\s*\[([^\]]+)\]\s*$", txt.split("T_map_world", 1)[1], re.M) if "T_map_world" in txt else []
    if len(rows) < 4:
        raise ValueError(f"{path}: no '# T_map_world' block (4 rows) in the comments: not made by ply_to_map, cannot move the reference into the map frame")
    T = np.array([[float(v) for v in r.split(",")] for r in rows[:4]])
    return {"resolution": res, "origin": (ox, oy), "image": str(Path(path).with_name(img)), "T_map_world": T}


def read_pgm(path):
    b = open(path, "rb").read()
    toks, i = [], 0
    while len(toks) < 4:
        while b[i:i + 1].isspace():
            i += 1
        if b[i:i + 1] == b"#":
            i = b.index(b"\n", i)
            continue
        j = i
        while not b[j:j + 1].isspace():
            j += 1
        toks.append(b[i:j])
        i = j
    if toks[0] != b"P5" or int(toks[3]) > 255:
        raise ValueError(f"{path}: not an 8-bit binary PGM (P5)")
    w, h = int(toks[1]), int(toks[2])
    return np.frombuffer(b, np.uint8, w * h, i + 1).reshape(h, w)


def cells(grid, res, origin, xy):
    """Pixel values under xy points (map frame, m); 255 outside the image (counted as neither free nor occupied)."""
    col = np.floor((xy[:, 0] - origin[0]) / res).astype(int)
    row = grid.shape[0] - 1 - np.floor((xy[:, 1] - origin[1]) / res).astype(int)
    ok = (col >= 0) & (col < grid.shape[1]) & (row >= 0) & (row < grid.shape[0])
    v = np.full(len(xy), 255, np.uint8)
    v[ok] = grid[row[ok], col[ok]]
    return v


def interp(t_src, pose_src, t):
    """Linear interpolation of (N, 3) x, y, yaw (yaw unwrapped); points outside the source span are NaN."""
    out = np.full((len(t), 3), np.nan)
    ok = (t >= t_src[0]) & (t <= t_src[-1])
    yaw = np.unwrap(pose_src[:, 2])
    for k, col in enumerate((pose_src[:, 0], pose_src[:, 1], yaw)):
        out[ok, k] = np.interp(t[ok], t_src, col)
    out[:, 2] = wrap(out[:, 2])
    return out


def map_to_odom(base_map, base_odom):
    """SE(2) T_map_odom = T_map_base * inv(T_odom_base), one row (x, y, yaw) per sample."""
    th = wrap(base_map[:, 2] - base_odom[:, 2])
    c, s = np.cos(th), np.sin(th)
    x = base_map[:, 0] - (c * base_odom[:, 0] - s * base_odom[:, 1])
    y = base_map[:, 1] - (s * base_odom[:, 0] + c * base_odom[:, 1])
    return np.c_[x, y, th]


def runs(mask, t):
    """[(t_first, t_last)] of each maximal run of True in mask."""
    out, start = [], None
    for i, m in enumerate(mask):
        if m and start is None:
            start = i
        if start is not None and (not m or i == len(mask) - 1):
            out.append((t[start], t[i if m else i - 1]))
            start = None
    return out


def evaluate(t, est, odom, ref, conv_pos=CONV_POS, conv_yaw_deg=CONV_YAW_DEG, conv_hold_s=CONV_HOLD_S, loss_pos=LOSS_POS, loss_s=LOSS_S,
             jump_pos=JUMP_POS, jump_yaw_deg=JUMP_YAW_DEG):
    """t (N,) s; est/odom/ref (N, 3) x, y, yaw: AMCL in map, FAST-LIO odometry in odom, reference in map, all at the times t (NaN rows are dropped)."""
    ok = ~(np.isnan(est).any(1) | np.isnan(odom).any(1) | np.isnan(ref).any(1))
    t, est, odom, ref = t[ok], est[ok], odom[ok], ref[ok]
    if len(t) < 2:
        raise ValueError("fewer than 2 samples overlap in time: estimate and reference clocks do not match")
    perr = np.hypot(*(est[:, :2] - ref[:, :2]).T)
    yerr = np.degrees(np.abs(wrap(est[:, 2] - ref[:, 2])))
    good = (perr <= conv_pos) & (yerr <= conv_yaw_deg)
    conv = next((i for i in range(len(t)) if t[-1] - t[i] >= conv_hold_s and good[(t >= t[i]) & (t <= t[i] + conv_hold_s)].all()), None)
    after = slice(conv, None) if conv is not None else slice(0, 0)
    bad = runs(perr[after] > loss_pos, t[after])
    mo = map_to_odom(est, odom)
    dmo = np.diff(mo, axis=0)
    jumps = (np.hypot(dmo[:, 0], dmo[:, 1]) > jump_pos) | (np.degrees(np.abs(wrap(dmo[:, 2]))) > jump_yaw_deg)
    a = slice(conv, None) if conv is not None else slice(0, None)  # error statistics: after convergence when it happened, else the whole run
    return {"samples": int(len(t)), "duration_s": float(t[-1] - t[0]),
            "converged": conv is not None, "convergence_s": None if conv is None else float(t[conv] - t[0]),
            "pos_err_m": {"median": float(np.median(perr[a])), "p95": float(np.percentile(perr[a], 95)), "max": float(perr[a].max())},
            "yaw_err_deg": {"median": float(np.median(yerr[a])), "p95": float(np.percentile(yerr[a], 95)), "max": float(yerr[a].max())},
            "losses": sum(1 for s, e in bad if e - s >= loss_s), "map_odom_jumps": int(jumps.sum()),
            "initial_pos_err_m": float(perr[0])}


def reference_map(path, T, dt=0.0):
    t, xyz, quat, _ = read_poses(path)
    return t + dt, planar(xyz, quat, T)


def level_quat(ref_path, T):
    """Quaternion of R_L = T_R @ R(q0): FAST-LIO2's world frame is the IMU frame at start (NOT gravity-aligned), Voxel-SLAM's is, and its first pose q0 is
    that IMU frame in it. Vectors of FAST-LIO2's frame go to Voxel-SLAM's world with R(q0) and from there to the map frame with T_R (ply_to_map's
    levelling + Manhattan yaw), so the floor normal ends up on z exactly as in the PGM. Assumes both SLAMs start on the same IMU orientation."""
    _, _, quat, _ = read_poses(ref_path)
    return mat_to_quat(T[:3, :3] @ quat_to_mat(quat[0]))


def initial_pose(ref_xyyaw, dist=1.0, dyaw_deg=20.0):
    """First reference pose pushed `dist` m along its heading and turned `dyaw_deg`: AMCL must pull itself back."""
    x, y, yaw = ref_xyyaw[0]
    return x + dist * np.cos(yaw), y + dist * np.sin(yaw), float(wrap(yaw + np.radians(dyaw_deg)))


def ref_vs_map(ref_path, map_yaml, ref_dt=0.0):
    """Share (%) of the reference positions on free and on occupied PGM cells (a path that crosses walls comes from another run than the map) and
    the sensor height above the floor: the median z of the reference in the map frame, where the floor is z = 0."""
    m = read_map_yaml(map_yaml)
    T = m["T_map_world"]
    _, ref = reference_map(ref_path, T, ref_dt)
    _, xyz, _, _ = read_poses(ref_path)
    v = cells(read_pgm(m["image"]), m["resolution"], m["origin"], ref[:, :2])
    return {"ref_poses": len(ref), "ref_on_free_pct": float((v >= FREE).mean() * 100), "ref_on_occupied_pct": float((v < 50).mean() * 100),
            "sensor_height_m": float(np.median(xyz @ T[2, :3] + T[2, 3]))}


def report(est_path, odom_path, ref_path, map_yaml, ref_dt=0.0, **thr):
    m = read_map_yaml(map_yaml)
    t_ref, ref = reference_map(ref_path, m["T_map_world"], ref_dt)
    t_est, xyz, quat, extra = read_poses(est_path)
    est = planar(xyz, quat)
    t_od, oxyz, oquat, _ = read_poses(odom_path)
    out = evaluate(t_est, est, interp(t_od, planar(oxyz, oquat), t_est), interp(t_ref, ref, t_est), **thr)
    out.update(ref_vs_map(ref_path, map_yaml, ref_dt))
    if extra.shape[1] >= 2:
        sd = np.sqrt(extra[:, 0] + extra[:, 1])
        out["amcl_std_xy_m"] = {"median": float(np.median(sd)), "max": float(sd.max())}
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    i = sub.add_parser("init")
    i.add_argument("--ref", required=True)
    i.add_argument("--map", required=True)
    i.add_argument("--dist", type=float, default=1.0)
    i.add_argument("--dyaw", type=float, default=20.0)
    lv = sub.add_parser("level")
    lv.add_argument("--ref", required=True)
    lv.add_argument("--map", required=True)
    c = sub.add_parser("check")
    c.add_argument("--ref", required=True)
    c.add_argument("--map", required=True)
    c.add_argument("--ref-dt", type=float, default=0.0)
    r = sub.add_parser("report")
    for k in ("est", "odom", "ref", "map"):
        r.add_argument(f"--{k}", required=True)
    r.add_argument("--ref-dt", type=float, default=0.0)
    for k, v in (("conv-pos", CONV_POS), ("conv-yaw-deg", CONV_YAW_DEG), ("conv-hold-s", CONV_HOLD_S), ("loss-pos", LOSS_POS), ("loss-s", LOSS_S),
                 ("jump-pos", JUMP_POS), ("jump-yaw-deg", JUMP_YAW_DEG)):
        r.add_argument(f"--{k}", type=float, default=v)
    a = ap.parse_args()
    if a.cmd == "level":
        print(*level_quat(a.ref, read_map_yaml(a.map)["T_map_world"]))
    elif a.cmd == "check":
        print(json.dumps(ref_vs_map(a.ref, a.map, a.ref_dt), indent=2))
    elif a.cmd == "init":
        _, ref = reference_map(a.ref, read_map_yaml(a.map)["T_map_world"])
        print(*initial_pose(ref, a.dist, a.dyaw))
    else:
        print(json.dumps(report(a.est, a.odom, a.ref, a.map, a.ref_dt, **{k: getattr(a, k) for k in (
            "conv_pos", "conv_yaw_deg", "conv_hold_s", "loss_pos", "loss_s", "jump_pos", "jump_yaw_deg")}), indent=2))
