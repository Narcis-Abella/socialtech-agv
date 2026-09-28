#!/usr/bin/env python3
"""Self-check for glim_dump_to_ply.py on synthetic dumps. Run: python3 test_glim_dump_to_ply.py (or pytest)"""
import pathlib
import subprocess
import sys
import tempfile

import numpy as np

SCRIPT = pathlib.Path(__file__).with_name("glim_dump_to_ply.py")


def write_pose(path, T):
    path.mkdir(parents=True)
    rows = "\n".join(" ".join(f"{v:g}" for v in row) for row in T)  # Eigen's default matrix print
    (path / "data.txt").write_text(f"id: 0\nT_world_origin: \n{rows}\nT_origin_endpoint_L: \n{rows}\n")


def write_submap(path, T, points, intensities, compact):
    write_pose(path, T)
    if compact:
        np.asarray(points[:, :3], dtype="<f4").tofile(path / "points_compact.bin")
        if intensities is not None:
            np.asarray(intensities, dtype="<f4").tofile(path / "intensities_compact.bin")
    else:
        np.asarray(points, dtype="<f8").tofile(path / "points.bin")
        if intensities is not None:
            np.asarray(intensities, dtype="<f8").tofile(path / "intensities.bin")


def write_graph(dump, num_submaps):
    (dump / "graph.txt").write_text(f"num_submaps: {num_submaps}\nnum_all_frames: 4\nnum_matching_cost_factors: 0\n")


def read_ply(path):
    data = path.read_bytes()
    end = data.index(b"end_header\n") + len(b"end_header\n")
    header = data[:end].decode().splitlines()
    names = [line.split()[-1] for line in header if line.startswith("property")]
    return header, np.frombuffer(data[end:], dtype=[(n, "<f4") for n in names])


def export(dump):
    """Run the script on `dump`; return (stderr, header, cloud) and require success."""
    out = dump / "map.ply"
    proc = subprocess.run([sys.executable, str(SCRIPT), str(dump), str(out)], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr
    return (proc.stderr, *read_ply(out))


def export_fails(dump):
    """Run the script on `dump`; require a clean `error:` exit, not a traceback. Return stderr."""
    proc = subprocess.run([sys.executable, str(SCRIPT), str(dump), str(dump / "map.ply")],
                          capture_output=True, text=True)
    assert proc.returncode == 1 and proc.stderr.startswith("error:") and "Traceback" not in proc.stderr, proc
    return proc.stderr


def xyz(cloud):
    return np.stack([cloud["x"], cloud["y"], cloud["z"]], axis=1)


def two_submap_dump(dump, with_second_intensities):
    # Submap 0: compact, 90 deg about z + translation.
    # Submap 1: double points; NaN first point and w=0.5 last point trigger SubMap::load's repair
    # (w forced to 1, non-finite points dropped, intensities left unshifted).
    T0 = np.array([[0, -1, 0, 1], [1, 0, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]], dtype=float)
    T1 = np.array([[1, 0, 0, 10], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
    p0 = np.array([[1, 0, 0, 1], [0, 2, 0, 1]], dtype=float)
    p1 = np.array([[np.nan, 0, 0, 1], [1, 1, 1, 1], [2, 0, 0, 0.5]], dtype=float)
    write_submap(dump / "000000", T0, p0, [5.0, 6.0], compact=True)
    write_submap(dump / "000001", T1, p1, [9.0, 7.0, 8.0] if with_second_intensities else None, compact=False)
    write_graph(dump, 2)
    _, header, cloud = export(dump)

    expected = np.array([[1, 3, 3], [-1, 2, 3], [11, 1, 1], [12, 0, 0]], dtype=np.float32)
    assert np.allclose(xyz(cloud), expected), cloud
    props = ["property float x", "property float y", "property float z"]
    props += ["property float intensity"] if with_second_intensities else []
    assert header == ["ply", "format binary_little_endian 1.0", "comment generated with glim_dump_to_ply.py",
                      "element vertex 4", *props, "end_header"], header
    return cloud


def test_transform_repair_and_intensities(tmp_path):
    cloud = two_submap_dump(tmp_path, with_second_intensities=True)
    assert list(cloud["intensity"]) == [5, 6, 9, 7], cloud["intensity"]  # GLIM keeps the leading ones


def test_one_submap_without_intensities_drops_the_field(tmp_path):
    two_submap_dump(tmp_path, with_second_intensities=False)


def test_repair_emptying_a_submap_keeps_intensities(tmp_path):
    # A submap whose only point is removed by the repair keeps GLIM's intensity buffer, so the
    # field survives; the export holds just the other submap's point.
    write_submap(tmp_path / "000000", np.eye(4), np.array([[np.nan, 0, 0, 1]]), [4.0], compact=False)
    write_submap(tmp_path / "000001", np.eye(4), np.array([[1, 2, 3, 1]], dtype=float), [5.0], compact=True)
    write_graph(tmp_path, 2)
    _, header, cloud = export(tmp_path)
    assert "property float intensity" in header and len(cloud) == 1 and cloud["intensity"][0] == 5, cloud


def test_empty_points_file_drops_intensities(tmp_path):
    # A submap whose points file holds no points has a null intensity buffer in GLIM, so the
    # export loses the field even though the file exists.
    write_submap(tmp_path / "000000", np.eye(4), np.empty((0, 4)), [], compact=False)
    write_submap(tmp_path / "000001", np.eye(4), np.array([[1, 2, 3, 1]], dtype=float), [5.0], compact=False)
    write_graph(tmp_path, 2)
    _, header, cloud = export(tmp_path)
    assert "property float intensity" not in header and len(cloud) == 1, cloud


def test_intensity_count_mismatch_zero_fills_or_truncates(tmp_path):
    points = np.array([[1, 0, 0, 1], [2, 0, 0, 1], [3, 0, 0, 1]], dtype=float)
    write_submap(tmp_path / "000000", np.eye(4), points, [4.0], compact=False)  # short: zero-filled
    write_submap(tmp_path / "000001", np.eye(4), points[:1], [6.0, 7.0, 8.0], compact=True)  # long: truncated
    write_graph(tmp_path, 2)
    stderr, _, cloud = export(tmp_path)
    assert list(cloud["intensity"]) == [4, 0, 0, 6], cloud["intensity"]
    assert stderr.count("warning:") == 2, stderr


def test_partial_trailing_record_is_ignored(tmp_path):
    write_pose(tmp_path / "000000", np.eye(4))
    np.array([1, 2, 3, 1, 9, 9], dtype="<f8").tofile(tmp_path / "000000" / "points.bin")
    write_pose(tmp_path / "000001", np.eye(4))
    np.array([4, 5, 6, 9], dtype="<f4").tofile(tmp_path / "000001" / "points_compact.bin")
    write_graph(tmp_path, 2)
    _, _, cloud = export(tmp_path)
    assert np.allclose(xyz(cloud), [[1, 2, 3], [4, 5, 6]]), cloud


def test_points_bin_takes_precedence_over_compact(tmp_path):
    sub = tmp_path / "000000"
    write_submap(sub, np.eye(4), np.array([[1, 2, 3, 1]], dtype=float), [7.0], compact=False)
    np.array([9, 9, 9], dtype="<f4").tofile(sub / "points_compact.bin")
    np.array([9], dtype="<f4").tofile(sub / "intensities_compact.bin")
    write_graph(tmp_path, 1)
    _, _, cloud = export(tmp_path)
    assert np.allclose(xyz(cloud), [[1, 2, 3]]) and list(cloud["intensity"]) == [7], cloud


def test_bad_input_exits_cleanly(tmp_path):
    assert "graph.txt" in export_fails(tmp_path)  # not a dump

    (tmp_path / "graph.txt").write_text("num_all_frames: 4\n")
    assert "num_submaps:" in export_fails(tmp_path)

    write_graph(tmp_path, 1)
    write_pose(tmp_path / "000000", np.eye(4))
    np.array([1, 2, 3, 1], dtype="<f8").tofile(tmp_path / "000000" / "points.bin")
    (tmp_path / "000000" / "data.txt").write_text("id: 0\nT_world_origin: \n1 0 0\n")  # truncated
    assert "T_world_origin:" in export_fails(tmp_path)


if __name__ == "__main__":
    for name, test in list(globals().items()):
        if name.startswith("test_"):
            with tempfile.TemporaryDirectory() as tmp:
                test(pathlib.Path(tmp))
    print("ok")
