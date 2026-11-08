"""The 3D map of a simulated world for level_corrector: the triangle soup sampled and voxelized (tris_to_prior.py), as <out dir>/map.ply (binary little-endian float x y z) + map.yaml with an identity
T_map_world: what ops/make_map.sh writes for a real map. Map frame = world frame, floor at z = 0.
usage: tris_to_ply.py WORLD.tris OUT_DIR [voxel_m=0.1] [spacing_m=0.05]"""
import sys
from pathlib import Path

import numpy as np

from tris_to_prior import read_tris, sample, voxelize


def write(points, out_dir):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    hdr = f"ply\nformat binary_little_endian 1.0\nelement vertex {len(points)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n"
    (out / "map.ply").write_bytes(hdr.encode() + np.asarray(points, "<f4").tobytes())
    (out / "map.yaml").write_text("image: map.pgm\n# T_map_world (row-major; p_map = T p_ply):\n# [1.0, 0.0, 0.0, 0.0]\n# [0.0, 1.0, 0.0, 0.0]\n# [0.0, 0.0, 1.0, 0.0]\n# [0.0, 0.0, 0.0, 1.0]\n")


if __name__ == "__main__":
    voxel = float(sys.argv[3]) if len(sys.argv) > 3 else .1
    spacing = float(sys.argv[4]) if len(sys.argv) > 4 else .05
    pts = voxelize(sample(read_tris(sys.argv[1]), spacing), voxel)
    write(pts, sys.argv[2])
    print(f"{len(pts)} voxels of {voxel} m from {sys.argv[1]}")
