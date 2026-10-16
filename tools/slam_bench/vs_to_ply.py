"""Voxel-SLAM output -> one world-frame PLY.
The run dir holds one body-frame PCD per frame (<i>.pcd: binary, x y z intensity as float32) and alidarState.txt (line i = pose of frame i: t x y z qx qy qz qw; more columns are ignored).
Each frame is moved to the world with its pose and the cloud is thinned to one point per voxel. usage: vs_to_ply.py <run dir> <out.ply> [voxel_m=0.05]"""
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

import ply_to_map

DATA = b"DATA binary\n"


def read_pcd_xyz(path):
    """x, y, z (N x 3, float64) of a binary PCD whose points are four float32 (x y z intensity)."""
    b = Path(path).read_bytes()
    return np.frombuffer(b[b.index(DATA) + len(DATA):], "<f4").reshape(-1, 4)[:, :3].astype(np.float64)


def first_per_voxel(p, vox):
    _, k = np.unique(np.floor(p / vox).astype(np.int64), axis=0, return_index=True)
    return p[k]


def vs_to_ply(run_dir, out, vox=0.05):
    """Write the world-frame cloud of a Voxel-SLAM run to `out`; returns (frames, points)."""
    run = Path(run_dir)
    poses = np.loadtxt(run / "alidarState.txt", usecols=range(8), ndmin=2)
    rot = Rotation.from_quat(poses[:, 4:8])
    parts = [first_per_voxel(rot[i].apply(read_pcd_xyz(run / f"{i}.pcd")) + poses[i, 1:4], vox) for i in range(len(poses))]
    cloud = first_per_voxel(np.vstack(parts), vox)
    ply_to_map.write_ply(out, cloud)
    return len(poses), len(cloud)


if __name__ == "__main__":
    n, m = vs_to_ply(sys.argv[1], sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 0.05)
    print(n, "frames ->", m, "points")
