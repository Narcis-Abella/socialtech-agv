"""Point-cloud and pose I/O, and the world cloud of a Voxel-SLAM run (the part of vs_to_ply.py that builds it). numpy + scipy only."""
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

PLY_TYPES = {"char": "i1", "uchar": "u1", "short": "<i2", "ushort": "<u2", "int": "<i4", "uint": "<u4", "float": "<f4", "double": "<f8"}
DATA = b"DATA binary\n"


def read_ply(path):
    """(N, 3) float64 xyz of a binary little-endian PLY (other properties ignored)."""
    raw = Path(path).read_bytes()
    end = raw.index(b"end_header\n") + len(b"end_header\n")
    head = raw[:end].decode().splitlines()
    if "format binary_little_endian 1.0" not in head:
        sys.exit(f"error: {path} is not a binary little-endian PLY")
    n = int(next(l.split()[2] for l in head if l.startswith("element vertex")))
    props = [(l.split()[2], PLY_TYPES[l.split()[1]]) for l in head if l.startswith("property")]
    a = np.frombuffer(raw, dtype=np.dtype(props), count=n, offset=end)
    return np.column_stack([a["x"], a["y"], a["z"]]).astype(np.float64)


def read_traj(path):
    """(positions (N, 3), quaternions xyzw (N, 4)) of a TUM trajectory (t x y z qx qy qz qw)."""
    a = np.loadtxt(path)
    return a[:, 1:4], a[:, 4:8]


def write_ply(path, P):
    """Binary little-endian PLY with x, y, z (float32)."""
    Path(path).write_bytes(b"ply\nformat binary_little_endian 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nend_header\n" % len(P) + P.astype("<f4").tobytes())


def read_pcd_xyz(path):
    """x, y, z (N x 3, float64) of a binary PCD whose points are four float32 (x y z intensity)."""
    b = Path(path).read_bytes()
    return np.frombuffer(b[b.index(DATA) + len(DATA):], "<f4").reshape(-1, 4)[:, :3].astype(np.float64)


def first_per_voxel(p, vox):
    """The first point of each voxel, in (ix, iy, iz) order. A 1-D key in that same order: np.unique(axis=0) sorts structured rows, 80 % of the run on the Nano."""
    i = np.floor(p / vox).astype(np.int64)
    i -= i.min(0)
    m = i.max(0) + 1
    if int(m[0]) * int(m[1]) * int(m[2]) >= 2 ** 62:       # far outliers: the 1-D key would wrap in int64 and scramble the order
        _, k = np.unique(i, axis=0, return_index=True)
    else:
        _, k = np.unique((i[:, 0] * m[1] + i[:, 1]) * m[2] + i[:, 2], return_index=True)   # return_index sorts stably: k is the first occurrence
    return p[k]


def assemble_world_cloud(run_dir, vox):
    """(poses (n, 8), cloud (m, 3)) of a Voxel-SLAM run dir: every <i>.pcd body-frame frame moved to the world with pose i of alidarState.txt, then thinned to one
    point per voxel. The cloud holds float32 values (what world.ply stores), so the stages after it see the same numbers whether or not the file is written."""
    run = Path(run_dir)
    poses = np.loadtxt(run / "alidarState.txt", usecols=range(8), ndmin=2)
    rot = Rotation.from_quat(poses[:, 4:8])
    parts = [first_per_voxel(rot[i].apply(read_pcd_xyz(run / f"{i}.pcd")) + poses[i, 1:4], vox) for i in range(len(poses))]
    return poses, first_per_voxel(np.vstack(parts), vox).astype(np.float32).astype(np.float64)
