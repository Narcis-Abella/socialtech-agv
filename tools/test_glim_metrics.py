#!/usr/bin/env python3
"""Self-check for glim_metrics.py on synthetic dumps. Run: python3 test_glim_metrics.py (or pytest)"""
import math
import pathlib
import sys
import tempfile

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import glim_metrics  # noqa: E402


def write_submap(path, T, points, stamps):
    path.mkdir(parents=True)
    rows = "\n".join(" ".join(f"{v:.9g}" for v in row) for row in T)
    frames = "".join(f"frame_{i}\nid: {i}\nstamp: {s:.9f}\n" for i, s in enumerate(stamps))
    (path / "data.txt").write_text(f"id: 0\nT_world_origin: \n{rows}\nnum_frames: {len(stamps)}\n{frames}")
    np.asarray(points, dtype="<f4").tofile(path / "points_compact.bin")


def write_traj(dump, rows):
    np.savetxt(dump / "traj_lidar.txt", rows, fmt="%.9f")


def wall(x0):
    """Points on the plane x = x0, 4 m x 2 m, on a 5 cm grid (denser than the metric's voxel)."""
    y, z = np.meshgrid(np.arange(0, 4, 0.05), np.arange(0, 2, 0.05))
    return np.stack([np.full(y.size, x0), y.ravel(), z.ravel()], axis=1)


def make_dump(root, name, second_offset, traj_z=0.0, roll_deg=0.0, noise=0.0):
    """Two passes over the same wall, 100 s apart; the second pass is shifted by `second_offset` in x."""
    dump = root / name
    dump.mkdir()
    (dump / "graph.txt").write_text("num_submaps: 2\nnum_all_frames: 2\nnum_matching_cost_factors: 7\n")
    T1 = np.eye(4)
    T1[0, 3] = second_offset
    rng = np.random.default_rng(1)
    write_submap(dump / "000000", np.eye(4), wall(1.0) + rng.normal(0, noise, wall(1.0).shape), [0.0, 1.0])
    write_submap(dump / "000001", T1, wall(1.0) + rng.normal(0, noise, wall(1.0).shape), [100.0, 101.0])
    q = [math.sin(math.radians(roll_deg) / 2), 0, 0, math.cos(math.radians(roll_deg) / 2)]
    write_traj(dump, [[0, 0, 0, 0, 0, 0, 0, 1], [50, 3, 4, traj_z, *q], [100, 0, 0, 0, 0, 0, 0, 1]])
    return dump


def test_metrics():
    with tempfile.TemporaryDirectory() as tmp:
        root = pathlib.Path(tmp)
        good = glim_metrics.dump_metrics(make_dump(root, "good", 0.0))
        doubled = glim_metrics.dump_metrics(make_dump(root, "doubled", 0.5, traj_z=0.3, roll_deg=4.0))

        # Same surface seen twice: revisit distances are sampling-limited, well under a voxel.
        assert good["revisit_median_m"] < 0.05, good
        assert good["revisit_fraction"] > 0.9, good
        # Doubled wall 0.5 m apart: the second pass is 0.5 m from the first.
        assert abs(doubled["revisit_median_m"] - 0.5) < 0.02, doubled

        assert good["z_range_m"] == 0 and good["roll_range_deg"] == 0, good
        assert abs(doubled["z_range_m"] - 0.3) < 1e-6 and abs(doubled["roll_range_deg"] - 4.0) < 1e-6, doubled
        assert abs(good["path_m"] - 10.0) < 1e-6 and good["end_to_start_m"] == 0, good
        assert good["submaps"] == 2 and good["matching_factors"] == 7, good


def test_mme():
    rng = np.random.default_rng(0)
    base = np.vstack([wall(0.0), wall(2.0)])
    crisp = base + rng.normal(0, 0.005, base.shape)
    blurred = base + rng.normal(0, 0.05, base.shape)
    m_crisp, m_blurred = glim_metrics.mme(crisp), glim_metrics.mme(blurred)
    assert m_crisp < m_blurred, (m_crisp, m_blurred)
    # Density-independent: more samples of the same surface give about the same value after voxelization.
    dense = np.vstack([crisp, base + rng.normal(0, 0.005, base.shape)])
    assert abs(glim_metrics.mme(dense) - m_crisp) < 0.3, (glim_metrics.mme(dense), m_crisp)

    with tempfile.TemporaryDirectory() as tmp:
        # An exact plane has a degenerate covariance (skipped); real surfaces are never exact.
        m = glim_metrics.dump_metrics(make_dump(pathlib.Path(tmp), "d", 0.0, noise=0.005))
        assert m["mme"] is not None and np.isfinite(m["mme"]), m


def test_runs_divergence():
    with tempfile.TemporaryDirectory() as tmp:
        root = pathlib.Path(tmp)
        a, b = make_dump(root, "a", 0.0), make_dump(root, "b", 0.0)
        write_traj(b, [[0, 0, 0, 0, 0, 0, 0, 1], [50, 3, 4.5, 0, 0, 0, 0, 1], [100, 0, 0, 0, 0, 0, 0, 1],
                       [150, 9, 9, 9, 0, 0, 0, 1]])  # extra stamp only in b: ignored
        div = glim_metrics.run_divergence([a, b])
        assert abs(div["max_m"] - 0.5) < 1e-6 and abs(div["mean_m"] - 0.5 / 3) < 1e-6, div


if __name__ == "__main__":
    test_metrics()
    test_mme()
    test_runs_divergence()
    print("ok")
