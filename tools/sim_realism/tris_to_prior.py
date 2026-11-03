"""The 3D prior of a simulated world for the relocalizer: the triangle soup of world_to_tris.py sampled and voxelized, as map.xyz (map frame = world frame, floor at z = 0).
usage: tris_to_prior.py WORLD.tris OUT.xyz [voxel_m=0.2] [spacing_m=0.1]"""
import struct
import sys

import numpy as np


def read_tris(path):
    with open(path, 'rb') as f:
        assert f.read(4) == b'TRIS', 'not a .tris file'
        n, = struct.unpack('<I', f.read(4))
        return np.frombuffer(f.read(36 * n), '<f4').reshape(n, 3, 3).astype(float)


def sample(tris, spacing):
    """Points on every triangle, about `spacing` apart (barycentric grid over each triangle's longest edge)."""
    out = []
    for a, b, c in tris:
        n = max(1, int(np.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b)) / spacing)))
        i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1))
        keep = (i + j <= n).ravel()
        u, v = (i.ravel()[keep] / n)[:, None], (j.ravel()[keep] / n)[:, None]
        out.append(a + u * (b - a) + v * (c - a))
    return np.concatenate(out)


def voxelize(points, voxel):
    """One point (the voxel centroid) per occupied voxel."""
    key = np.floor(points / voxel).astype(np.int64)
    _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
    inv = inv.ravel()
    sums = np.zeros((len(cnt), 3))
    np.add.at(sums, inv, points)
    return sums / cnt[:, None]


if __name__ == '__main__':
    voxel = float(sys.argv[3]) if len(sys.argv) > 3 else .2
    spacing = float(sys.argv[4]) if len(sys.argv) > 4 else .1
    pts = voxelize(sample(read_tris(sys.argv[1]), spacing), voxel)
    np.savetxt(sys.argv[2], pts, fmt='%.4f')
    print(f'{len(pts)} voxels of {voxel} m from {sys.argv[1]}')
