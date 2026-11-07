"""tris_to_ply.py writes a PLY and a yaml that level_corrector's reader accepts (header, count, identity T). Run: python3 test_tris_to_ply.py"""
import struct
import tempfile
from pathlib import Path

import numpy as np

import tris_to_ply as t


def test_ply_has_one_float_xyz_row_per_voxel_and_an_identity_transform():
    tris = np.array([[[0, 0, 0], [2, 0, 0], [0, 2, 0]]], "<f4")   # one triangle on the floor
    d = Path(tempfile.mkdtemp()) / "w.tris"
    d.write_bytes(b"TRIS" + struct.pack("<I", 1) + tris.tobytes())
    out = Path(tempfile.mkdtemp())
    pts = t.voxelize(t.sample(t.read_tris(str(d)), 0.05), 0.1)
    t.write(pts, out)
    raw = (out / "map.ply").read_bytes()
    head, body = raw.split(b"end_header\n", 1)
    assert b"format binary_little_endian 1.0" in head and f"element vertex {len(pts)}".encode() in head
    got = np.frombuffer(body, "<f4").reshape(-1, 3)
    assert len(got) == len(pts) > 100 and np.allclose(got[:, 2], 0) and got[:, :2].min() >= -1e-6 and got[:, :2].max() <= 2 + 1e-6
    rows = [l for l in (out / "map.yaml").read_text().splitlines() if l.startswith("# [")]
    assert rows == ["# [1.0, 0.0, 0.0, 0.0]", "# [0.0, 1.0, 0.0, 0.0]", "# [0.0, 0.0, 1.0, 0.0]", "# [0.0, 0.0, 0.0, 1.0]"]


if __name__ == "__main__":
    test_ply_has_one_float_xyz_row_per_voxel_and_an_identity_transform()
    print("ok")
