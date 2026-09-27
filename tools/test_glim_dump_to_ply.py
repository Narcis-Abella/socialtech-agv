#!/usr/bin/env python3
"""Self-check for glim_dump_to_ply.py on a synthetic two-submap dump. Run: python3 test_glim_dump_to_ply.py"""
import pathlib
import subprocess
import sys
import tempfile

import numpy as np

SCRIPT = pathlib.Path(__file__).with_name("glim_dump_to_ply.py")


def write_submap(path, T, points, intensities, compact):
    path.mkdir(parents=True)
    rows = "\n".join(" ".join(f"{v:g}" for v in row) for row in T)  # Eigen's default matrix print
    (path / "data.txt").write_text(f"id: 0\nT_world_origin: \n{rows}\nT_origin_endpoint_L: \n{rows}\n")
    if compact:
        np.asarray(points[:, :3], dtype="<f4").tofile(path / "points_compact.bin")
        if intensities is not None:
            np.asarray(intensities, dtype="<f4").tofile(path / "intensities_compact.bin")
    else:
        np.asarray(points, dtype="<f8").tofile(path / "points.bin")
        if intensities is not None:
            np.asarray(intensities, dtype="<f8").tofile(path / "intensities.bin")


def read_ply(path):
    data = path.read_bytes()
    end = data.index(b"end_header\n") + len(b"end_header\n")
    header = data[:end].decode().splitlines()
    names = [line.split()[-1] for line in header if line.startswith("property")]
    return header, np.frombuffer(data[end:], dtype=[(n, "<f4") for n in names])


def run(dump, with_second_intensities):
    # Submap 0: compact, 90 deg about z + translation.
    # Submap 1: double points; NaN first point and w=0.5 last point trigger SubMap::load's repair
    # (w forced to 1, non-finite points dropped, intensities left unshifted).
    T0 = np.array([[0, -1, 0, 1], [1, 0, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]], dtype=float)
    T1 = np.array([[1, 0, 0, 10], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
    p0 = np.array([[1, 0, 0, 1], [0, 2, 0, 1]], dtype=float)
    p1 = np.array([[np.nan, 0, 0, 1], [1, 1, 1, 1], [2, 0, 0, 0.5]], dtype=float)
    write_submap(dump / "000000", T0, p0, [5.0, 6.0], compact=True)
    write_submap(dump / "000001", T1, p1, [9.0, 7.0, 8.0] if with_second_intensities else None, compact=False)
    (dump / "graph.txt").write_text("num_submaps: 2\nnum_all_frames: 4\nnum_matching_cost_factors: 0\n")

    out = dump / "map.ply"
    subprocess.run([sys.executable, str(SCRIPT), str(dump), str(out)], check=True, capture_output=True)
    header, cloud = read_ply(out)

    expected = np.array([[1, 3, 3], [-1, 2, 3], [11, 1, 1], [12, 0, 0]], dtype=np.float32)
    got = np.stack([cloud["x"], cloud["y"], cloud["z"]], axis=1)
    assert np.allclose(got, expected), got
    props = ["property float x", "property float y", "property float z"]
    props += ["property float intensity"] if with_second_intensities else []
    assert header == ["ply", "format binary_little_endian 1.0", "comment generated with glim_dump_to_ply.py",
                      "element vertex 4", *props, "end_header"], header
    return cloud


with tempfile.TemporaryDirectory() as tmp:
    cloud = run(pathlib.Path(tmp) / "a", with_second_intensities=True)
    assert list(cloud["intensity"]) == [5, 6, 9, 7], cloud["intensity"]  # GLIM keeps the leading ones

    # One submap without intensities -> GLIM drops the field for the whole export.
    run(pathlib.Path(tmp) / "b", with_second_intensities=False)

    # A submap whose only point is removed by the repair keeps GLIM's intensity buffer, so the
    # field survives; the export holds just the other submap's point.
    dump = pathlib.Path(tmp) / "c"
    write_submap(dump / "000000", np.eye(4), np.array([[np.nan, 0, 0, 1]]), [4.0], compact=False)
    write_submap(dump / "000001", np.eye(4), np.array([[1, 2, 3, 1]], dtype=float), [5.0], compact=True)
    (dump / "graph.txt").write_text("num_submaps: 2\n")
    subprocess.run([sys.executable, str(SCRIPT), str(dump), str(dump / "map.ply")], check=True, capture_output=True)
    header, cloud = read_ply(dump / "map.ply")
    assert "property float intensity" in header and len(cloud) == 1 and cloud["intensity"][0] == 5, cloud

print("ok")
