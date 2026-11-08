"""Poses and the map frame: the floor-levelling rotation of FAST-LIO2's world frame, and the reference trajectory of a map's Voxel-SLAM run in the map frame.
usage:
  map_frame.py level --ref alidarState.txt --map map.yaml   -> "qx qy qz qw" of R_L, the rotation from FAST-LIO2's world frame to the floor-levelled, wall-aligned frame
Pose files: "t x y z qx qy qz qw ..." (alidarState.txt has the same first 8 columns).
The map YAML must carry the `# T_map_world` comment block written by make_map.sh (p_map = T p_ply). numpy only (the robot image has no scipy).
(Until 2026-11-08 this was pose_eval.py, the AMCL bench's evaluator: init/initpose/check/report/evaluate are in the git history.)"""
import argparse
import re

import numpy as np


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
        raise ValueError(f"{path}: no '# T_map_world' block (4 rows) in the comments: not made by make_map.sh, cannot move the reference into the map frame")
    T = np.array([[float(v) for v in r.split(",")] for r in rows[:4]])
    return {"resolution": res, "origin": (ox, oy), "image": str(Path(path).with_name(img)), "T_map_world": T}


def reference_map(path, T, dt=0.0):
    t, xyz, quat, _ = read_poses(path)
    return t + dt, planar(xyz, quat, T)


def level_quat(ref_path, T):
    """Quaternion of R_L = T_R @ R(q0): FAST-LIO2's world frame is the IMU frame at start (NOT gravity-aligned), Voxel-SLAM's is, and its first pose q0 is
    that IMU frame in it. Vectors of FAST-LIO2's frame go to Voxel-SLAM's world with R(q0) and from there to the map frame with T_R (make_map's
    levelling + Manhattan yaw), so the floor normal ends up on z exactly as in the PGM. Assumes both SLAMs start on the same IMU orientation."""
    _, _, quat, _ = read_poses(ref_path)
    return mat_to_quat(T[:3, :3] @ quat_to_mat(quat[0]))


def parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    lv = sub.add_parser("level")
    lv.add_argument("--ref", required=True)
    lv.add_argument("--map", required=True)
    return ap


if __name__ == "__main__":
    a = parser().parse_args()
    print(*level_quat(a.ref, read_map_yaml(a.map)["T_map_world"]))
