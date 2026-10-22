"""Checks vs_to_ply.py on a synthetic Voxel-SLAM run. Run: python3 test_vs_to_ply.py"""
import tempfile
from pathlib import Path

import numpy as np

import ply_to_map
import vs_to_ply

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


def test_frames_land_in_the_world_with_their_pose_and_repeats_collapse():
    with tempfile.TemporaryDirectory() as tmp:
        run = make_run(tmp, [(IDENT, [[1, 0, 0], [0, 1, 0]]), (TURN, [[1, 0, 0]]), (IDENT, [[1, 0, 0], [0, 1, 0]])])
        out = Path(tmp) / "world.ply"
        assert vs_to_ply.vs_to_ply(run, out, 0.05) == (3, 3)     # frame 2 repeats frame 0: 5 points in, 3 unique voxels out
        got = sorted_rows(ply_to_map.read_ply(out))
        want = sorted_rows(np.array([[1, 0, 0], [0, 1, 0], [10, 1, 0]], float))   # yaw 90 deg: body x -> world y
        assert np.allclose(got, want, atol=1e-4), got


def test_a_run_with_one_frame_works():
    with tempfile.TemporaryDirectory() as tmp:
        run = make_run(tmp, [(IDENT, [[2, 0, 0]])])
        assert vs_to_ply.vs_to_ply(run, Path(tmp) / "w.ply", 0.05) == (1, 1)


def test_command_line_prints_frames_and_points():
    import subprocess
    import sys
    with tempfile.TemporaryDirectory() as tmp:
        run = make_run(tmp, [(IDENT, [[1, 0, 0]]), (TURN, [[1, 0, 0]])])
        r = subprocess.run([sys.executable, str(Path(__file__).with_name("vs_to_ply.py")), str(run), str(Path(tmp) / "w.ply")], capture_output=True, text=True)
        assert r.returncode == 0 and r.stdout.strip() == "2 frames -> 2 points", (r.returncode, r.stdout, r.stderr)


if __name__ == "__main__":
    test_frames_land_in_the_world_with_their_pose_and_repeats_collapse()
    test_a_run_with_one_frame_works()
    test_command_line_prints_frames_and_points()
    print("ok")
