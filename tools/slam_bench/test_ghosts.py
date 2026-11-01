"""ghosts: the beam test through the C++ engine and the ghost rule. Moved from test_map_visibility.py."""
import numpy as np
import pytest

import beams_oracle
import ghosts
from synth_bag import Writer, write_bag
from synth_corridor import PHANTOM, T, drive, planes, world_cloud

NO_ENGINE = ghosts.engine_error()
needs_engine = pytest.mark.skipif(NO_ENGINE is not None or Writer is None, reason="needs libsocialtech_mapping_beams.so (mapper image) and rosbags")


def counter(scans, extr=(0, 0, 0)):
    """The beam counter ghost_points takes: the numpy oracle over analytic scans (the lidar at the IMU origin)."""
    return lambda C, margin: beams_oracle.count_beams(C, scans, extr, margin)

def test_the_ghost_rule_needs_enough_observations_and_a_hit_ratio_under_the_threshold():
    hit, passed = np.array([0, 1, 5, 0]), np.array([20, 9, 5, 5])      # observations 20, 10, 10, 5; ratios 0, 0.1, 0.5, 0
    assert ghosts.ghost_voxels(hit, passed).tolist() == [True, False, False, False]

def test_points_the_beams_pass_through_are_flagged_and_the_walls_are_not():
    scans, _ = drive(planes())
    P, n, Q = world_cloud(scans, PHANTOM)
    ghost = ghosts.ghost_points(P, Q, counter(scans))
    assert ghost[n:].all()                              # the phantom: every beam went through it
    assert ghost[:n].mean() < 0.02, ghost[:n].mean()    # the real surfaces: the beams end there

def test_ghost_points_band_limits_which_voxels_are_judged():
    scans, _ = drive(planes())
    high = np.random.default_rng(4).uniform([4.85, 0.35, 2.0], [5.15, 0.65, 2.3], (300, 3))     # a phantom above the 1.80 m of the obstacle band
    P, n, Q = world_cloud(scans, high)
    count = counter(scans)
    assert not ghosts.ghost_points(P, Q, count, band=(0.10, 1.80))[n:].any()      # out of the band: not judged
    assert ghosts.ghost_points(P, Q, count, band=(0.10, 2.35))[n:].all()          # 2.35: above the phantom, under the 2.4 m corridor ceiling
    assert ghosts.ghost_points(P, Q, count)[n:].all()                             # the default judges the whole height
    P2, n2, Q2 = world_cloud(scans, PHANTOM)
    assert ghosts.ghost_points(P2, Q2, count, band=(0.10, 1.80))[n2:].all()       # a phantom inside the band is found with the band too

def test_a_voxel_centroid_is_tested_where_the_surface_is(monkeypatch):
    monkeypatch.setattr(ghosts, "RATIO_MARGIN", 0.02)
    monkeypatch.setattr(ghosts, "RATIO_VOX", 0.1)
    scans, _ = drive(planes())
    P, n, Q = world_cloud(scans)
    assert ghosts.ghost_points(P, Q, counter(scans)).mean() < 0.02      # a wall off the voxel centre is hit at its own points
    P2, n2, Q2 = world_cloud(scans, PHANTOM)
    assert ghosts.ghost_points(P2, Q2, counter(scans))[n2:].all()       # and a phantom is still found

def test_voxels_seen_in_few_scans_are_not_flagged():
    scans, _ = drive(planes())
    P, n, Q = world_cloud(scans, PHANTOM)
    assert not ghosts.ghost_points(P, Q, counter(scans[:3]))[n:].any()   # 3 scans = at most 3 observations of a voxel

def test_the_ghost_test_counts_with_the_hit_margin_of_the_rule():
    scans, _ = drive(planes())
    P, n, Q = world_cloud(scans, PHANTOM)
    seen = []
    base = counter(scans)
    ghosts.ghost_points(P, Q, lambda C, margin: (seen.append(margin), base(C, margin))[1])
    assert seen == [ghosts.RATIO_MARGIN] and ghosts.RATIO_MARGIN == 0.03

@needs_engine
@pytest.mark.parametrize("kind", ["custom", "pc2"])
def test_cpp_counts_equal_a_numpy_oracle_from_custommsg_and_pointcloud2_bags(tmp_path, kind):
    from scipy.spatial.transform import Rotation
    scans, traj = drive(planes(pillar=True))
    poses = np.c_[np.arange(len(scans)) * 0.1 + 1.0 + ghosts.SCAN_MID, traj, np.tile(Rotation.from_matrix(scans[0][1]).as_quat(), (len(scans), 1))]   # pose at header + SCAN_MID
    poses = np.r_[poses[:1] - [0.1, *np.zeros(7)], poses]          # one pose before the first scan, so every scan has one
    P, n, Q = world_cloud(scans, PHANTOM)
    cells = np.random.default_rng(0).uniform([-1, -1.5, 0], [11, 1.5, 2], (3000, 3))           # map frame, around the corridor
    C = np.r_[P[::50], (cells - T[:3, 3]) @ T[:3, :3]]                                         # PLY (SLAM world) frame
    write_bag(tmp_path / "bag", [s[0] for s in scans], kind)
    ref = beams_oracle.count_beams(C, [(s[0].astype(np.float32).astype(float), s[1], s[2]) for s in scans], ghosts.EXTR, 0.05)
    got = ghosts.count_beams(C, ghosts.BagScans(tmp_path / "bag", poses, "/livox/lidar"), 0.05)
    assert ref[0].sum() > 100 and ref[1].sum() > 100, kind
    assert np.array_equal(got[0], ref[0]) and np.array_equal(got[1], ref[1]), kind


@needs_engine
def test_a_bag_without_scans_in_the_pose_window_declares_no_ghost(tmp_path):
    scans, traj = drive(planes())
    write_bag(tmp_path / "bag", [s[0] for s in scans], "custom")
    poses = np.c_[np.arange(5) + 1000.0, np.zeros((5, 3)), np.tile([0, 0, 0, 1.0], (5, 1))]      # poses 1000 s after the bag: no scan has one
    C = np.random.default_rng(1).uniform(-2, 2, (200, 3))
    hit, passed = ghosts.count_beams(C, ghosts.BagScans(tmp_path / "bag", poses, "/livox/lidar"), 0.03)
    assert hit.sum() == 0 and passed.sum() == 0 and not ghosts.ghost_voxels(hit, passed).any()


def test_a_missing_library_is_reported_with_the_way_out(monkeypatch, capsys):
    monkeypatch.setenv("BEAMS_LIB", "/nonexistent/libsocialtech_mapping_beams.so")
    assert "BEAMS_LIB" in ghosts.engine_error() and "mapper image" in ghosts.engine_error()
    assert ghosts.main(["--check-engine"]) == 1
    assert "BEAMS_LIB" in capsys.readouterr().err
