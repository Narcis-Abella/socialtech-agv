#!/usr/bin/env python3
"""Export a GLIM dump directory to a PLY point cloud, without a display.

GLIM 1.2.2 only exports points from the offline_viewer GUI (File > Save > Export Points).
This replicates that export from the files GLIM saves (references: glim, gtsam_points 1.2.2):

- GlobalMapping::export_points: each submap's points transformed by its T_world_origin
  (full 4x4 times homogeneous point, as Eigen::Isometry3d * Vector4d), concatenated in submap
  order; intensities kept only if every submap has them.
- Submap pose: T_world_origin from <submap>/data.txt. GlobalMapping::save() optimizes and writes
  the same estimate to data.txt and values.bin; data.txt prints it with 6 significant digits
  (sub-mm for maps within ~100 m). The GUI takes the pose from values.bin (full precision), so
  answering "No" to its "Do optimization?" prompt gives this same pose unrounded; "Yes"
  re-optimizes a rebuilt graph and poses may move. values.bin (boost-serialized gtsam::Values)
  is not practical to read here.
- Points (PointCloudCPU::load + SubMap::load): points.bin (double x,y,z,w) takes precedence over
  points_compact.bin (float x,y,z, w=1); intensities from the matching intensities file. If the
  first or last point is not homogeneous or not finite, w is forced to 1 for all points and
  non-finite points are dropped.
- Output (iridescence glk::save_ply_binary): binary little-endian, float x,y,z[,intensity].

Usage: glim_dump_to_ply.py <dump_dir> <out.ply>
Needs numpy (apt: python3-numpy).
"""
import os
import pathlib
import sys

import numpy as np


def read_matrix_after(path, label, rows=4, cols=4):
    """Read a row-major matrix that follows `label` in a whitespace-split text file (like GLIM's read_matrix)."""
    try:
        tokens = path.read_text(encoding="utf-8").split()
    except UnicodeDecodeError as e:
        sys.exit(f"error: cannot read {path}: {e}")
    try:
        i = tokens.index(label) + 1
        return np.array(tokens[i:i + rows * cols], dtype=np.float64).reshape(rows, cols)
    except ValueError:
        sys.exit(f"error: {path} has no {rows}x{cols} matrix after '{label}'")


def read_array(path, dtype, count):
    """Read `count` values like gtsam_points: missing values stay zero, extra values are ignored."""
    data = np.fromfile(path, dtype=dtype)
    if len(data) != count:
        print(f"warning: {path} has {len(data)} values for {count} points", file=sys.stderr)
    out = np.zeros(count, dtype=np.float64)
    out[:min(count, len(data))] = data[:count]
    return out


def load_submap(path):
    """Return (T_world_origin, points Nx4 double, intensities N double or None) for one submap dir."""
    if not (path / "data.txt").exists():
        sys.exit(f"error: {path}/data.txt not found")
    T_world_origin = read_matrix_after(path / "data.txt", "T_world_origin:")

    if (path / "points.bin").exists():
        raw = np.fromfile(path / "points.bin", dtype="<f8")
        points = raw[: len(raw) // 4 * 4].reshape(-1, 4)  # a partial trailing record is ignored
        intensity_file, intensity_dtype = path / "intensities.bin", "<f8"
    elif (path / "points_compact.bin").exists():
        raw = np.fromfile(path / "points_compact.bin", dtype="<f4")
        xyz = raw[: len(raw) // 3 * 3].reshape(-1, 3).astype(np.float64)
        points = np.hstack([xyz, np.ones((len(xyz), 1))])
        intensity_file, intensity_dtype = path / "intensities_compact.bin", "<f4"
    else:
        sys.exit(f"error: {path} contains neither points.bin nor points_compact.bin")

    # has_intensities() tests the buffer pointer: null only if the file held no points
    # (the repair below removes points but keeps GLIM's intensity buffer allocated).
    intensities = None
    if len(points) and intensity_file.exists():
        intensities = read_array(intensity_file, intensity_dtype, len(points))

    # SubMap::load repair of corrupted submaps, checked on the first and last point only.
    def valid(p):
        return abs(p[3] - 1.0) < 1e-3 and np.isfinite(p[:3]).all()

    if len(points) and not (valid(points[0]) and valid(points[-1])):
        print(f"warning: corrupted points detected in {path}", file=sys.stderr)
        points[:, 3] = 1.0
        points = points[np.isfinite(points).all(axis=1)]
        if intensities is not None:
            # GLIM drops points but not their intensities, so the leading ones stay; mirrored as is.
            intensities = intensities[: len(points)]
    return T_world_origin, points, intensities


def export_points(dump):
    """GlobalMapping::export_points over the submaps listed in graph.txt."""
    if not (dump / "graph.txt").exists():
        sys.exit(f"error: {dump} is not a GLIM dump (no graph.txt)")
    num_submaps = int(read_matrix_after(dump / "graph.txt", "num_submaps:", 1, 1)[0, 0])

    submaps = [load_submap(dump / f"{i:06d}") for i in range(num_submaps)]
    with_intensities = bool(submaps) and all(s[2] is not None for s in submaps)

    points = [(T @ p.T).T for T, p, _ in submaps]
    points = np.vstack(points) if points else np.empty((0, 4))
    intensities = np.concatenate([s[2] for s in submaps]) if with_intensities else None
    return points, intensities


def write_ply(f, points, intensities):
    """glk::save_ply_binary layout for vertices (+ intensities)."""
    names = ["x", "y", "z"] + (["intensity"] if intensities is not None else [])
    record = np.empty(len(points), dtype=[(n, "<f4") for n in names])
    for axis, n in enumerate("xyz"):
        record[n] = points[:, axis]
    if intensities is not None:
        record["intensity"] = intensities

    header = ["ply", "format binary_little_endian 1.0", "comment generated with glim_dump_to_ply.py",
              f"element vertex {len(points)}"] + [f"property float {n}" for n in names] + ["end_header"]
    f.write(("\n".join(header) + "\n").encode())
    f.write(record.tobytes())


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: glim_dump_to_ply.py <dump_dir> <out.ply>")
    dump, out = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
    # The map is written to a temporary file next to `out`, opened before loading so an unwritable
    # destination fails fast, and renamed over `out` only once complete.
    if os.path.isdir(out):
        sys.exit(f"error: cannot write {out}: is a directory")
    tmp = out.with_name(f".{out.name}.tmp")
    try:
        f = open(tmp, "wb")
    except OSError as e:
        sys.exit(f"error: cannot write {out}: {e}")
    try:
        with f:
            points, intensities = export_points(dump)
            if len(points) == 0:
                sys.exit("error: no points available for export")  # same condition offline_viewer refuses on
            write_ply(f, points, intensities)
        os.replace(tmp, out)
    except OSError as e:
        sys.exit(f"error: {e}")
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)
    print(f"wrote {len(points)} points{' with intensity' if intensities is not None else ''} to {out}")


if __name__ == "__main__":
    main()
