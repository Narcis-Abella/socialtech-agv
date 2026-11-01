"""cloud_io: PLY / pose I/O and the world cloud of a Voxel-SLAM run."""
import tempfile
from pathlib import Path

import numpy as np

import cloud_io
from synth_room import RNG, write_test_ply

HEADER = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
          "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n")  # what Voxel-SLAM writes (checked on a real run)


def write_pcd(path, pts):
    pts = np.asarray(pts, float)
    path.write_bytes((HEADER % (len(pts), len(pts))).encode() + np.hstack([pts, np.ones((len(pts), 1))]).astype("<f4").tobytes())


def make_run(tmp, frames):
    """frames: list of (pose [x y z qx qy qz qw], body-frame points). alidarState.txt gets two extra columns: Voxel-SLAM writes 26."""
    run = Path(tmp) / "run"
    run.mkdir()
    lines = []
    for i, (pose, pts) in enumerate(frames):
        write_pcd(run / f"{i}.pcd", pts)
        lines.append(" ".join(str(v) for v in [100.0 + i, *pose, 0.5, 0.25]))
    (run / "alidarState.txt").write_text("\n".join(lines) + "\n")
    return run


S = np.sqrt(0.5)
IDENT = [0, 0, 0, 0, 0, 0, 1]
TURN = [10, 0, 0, 0, 0, S, S]    # 10 m along x, yawed 90 deg


def sorted_rows(a):
    return a[np.lexsort(a.T[::-1])]


def test_read_ply_roundtrip_with_and_without_intensity():
    P = RNG.uniform(-3, 3, (500, 3))
    with tempfile.TemporaryDirectory() as tmp:
        for intensity in (True, False):
            write_test_ply(Path(tmp) / "a.ply", P, intensity)
            assert np.allclose(cloud_io.read_ply(Path(tmp) / "a.ply"), P, atol=1e-5), intensity


def test_write_ply_stores_float32_and_reads_back_exactly():
    P = RNG.uniform(-3, 3, (200, 3))
    with tempfile.TemporaryDirectory() as tmp:
        cloud_io.write_ply(Path(tmp) / "a.ply", P)
        assert np.array_equal(cloud_io.read_ply(Path(tmp) / "a.ply"), P.astype(np.float32).astype(np.float64))


def test_read_traj_gives_positions_and_quaternions():
    with tempfile.TemporaryDirectory() as tmp:
        f = Path(tmp) / "t.txt"
        f.write_text("1.0 1 2 3 0 0 0 1\n2.0 4 5 6 0 0 0.5 0.5\n")
        pos, quat = cloud_io.read_traj(f)
        assert pos.tolist() == [[1, 2, 3], [4, 5, 6]] and quat.tolist() == [[0, 0, 0, 1], [0, 0, 0.5, 0.5]]


def test_frames_land_in_the_world_with_their_pose_and_repeats_collapse():
    with tempfile.TemporaryDirectory() as tmp:
        run = make_run(tmp, [(IDENT, [[1, 0, 0], [0, 1, 0]]), (TURN, [[1, 0, 0]]), (IDENT, [[1, 0, 0], [0, 1, 0]])])
        poses, cloud = cloud_io.assemble_world_cloud(run, 0.05)
        assert len(poses) == 3 and poses.shape[1] == 8       # frame 2 repeats frame 0: 5 points in, 3 unique voxels out
        want = sorted_rows(np.array([[1, 0, 0], [0, 1, 0], [10, 1, 0]], float))   # yaw 90 deg: body x -> world y
        assert np.allclose(sorted_rows(cloud), want, atol=1e-4), cloud


def test_the_assembled_cloud_holds_float32_values():
    # make_map keeps it in memory instead of writing and re-reading world.ply: the values must be the ones that file would give
    with tempfile.TemporaryDirectory() as tmp:
        run = make_run(tmp, [(IDENT, [[1.1234567, 0.3, 0.2], [0.7654321, 1.3, 0.1]]), (TURN, [[1.9999999, 0.1, 0.05]])])
        _, cloud = cloud_io.assemble_world_cloud(run, 0.05)
        assert np.array_equal(cloud, cloud.astype(np.float32).astype(np.float64))


def test_a_run_with_one_frame_works():
    with tempfile.TemporaryDirectory() as tmp:
        run = make_run(tmp, [(IDENT, [[2, 0, 0]])])
        poses, cloud = cloud_io.assemble_world_cloud(run, 0.05)
        assert len(poses) == 1 and len(cloud) == 1


def test_far_outliers_do_not_break_the_voxel_order():
    # a drifted run leaves stray points ~1e5 m away: at 0.025 m the 1-D key (ix * m1 + iy) * m2 + iz exceeds int64 and wraps, which scrambles the order
    rng = np.random.default_rng(3)
    p = np.vstack([rng.uniform(0, 5, (500, 3)), [[1e5, 1e5, 1e5], [-1e5, -1e5, -1e5], [1e5, -1e5, 0.0]]])
    got = cloud_io.first_per_voxel(p, 0.025)
    i = np.floor(p / 0.025).astype(np.int64)
    _, k = np.unique(i, axis=0, return_index=True)        # the reference: lexicographic (ix, iy, iz), first point of each voxel
    assert np.array_equal(got, p[k])
