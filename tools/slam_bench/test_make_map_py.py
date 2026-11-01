"""make_map.py: the whole post-SLAM pipeline in one process, on the analytic corridor (the beam counter is the numpy oracle; the end-to-end test needs the engine)."""
import numpy as np
import pytest

import beams_oracle
import cloud_io
import ghosts
import grid2d
import make_map
from synth_bag import Writer, write_bag
from synth_corridor import PHANTOM, drive, planes, world_cloud
from synth_room import path, room

NO_ENGINE = ghosts.engine_error()
needs_engine = pytest.mark.skipif(NO_ENGINE is not None or Writer is None, reason="needs libsocialtech_mapping_beams.so (mapper image) and rosbags")


def counter(scans):
    return lambda C, margin: beams_oracle.count_beams(C, scans, (0, 0, 0), margin)


def poses_of(traj, t0=1.0):
    """t x y z qx qy qz qw rows: the corridor drive, one pose per scan, identity rotation."""
    return np.c_[np.arange(len(traj)) * 0.1 + t0 + ghosts.SCAN_MID, traj, np.tile([0, 0, 0, 1.0], (len(traj), 1))]


def read_pgm(p):
    raw = p.read_bytes()
    w, h = map(int, raw.split(b"\n")[1].split())
    return np.frombuffer(raw[-w * h:], np.uint8).reshape(h, w)


def test_build_writes_the_files_and_map_clean_keeps_floor_and_ceiling_but_not_the_phantom(tmp_path):
    scans, traj = drive(planes())
    P, n, Q = world_cloud(scans, PHANTOM)
    data = make_map.build(P, poses_of(traj), "unused", tmp_path, res=0.05, count=counter(scans))
    assert all((tmp_path / f).exists() for f in ("map.ply", "map_clean.ply", "map.pgm", "map.yaml"))
    clean = cloud_io.read_ply(tmp_path / "map_clean.ply")
    assert (clean[:, 2] < 0.1).sum() > 100 and (clean[:, 2] > 1.8).sum() > 100       # the whole height is kept, floor and ceiling included
    assert not ((np.abs(clean[:, 0] - 5.0) < 0.2) & (np.abs(clean[:, 1] - 0.5) < 0.2) & (clean[:, 2] > 0.7) & (clean[:, 2] < 1.2)).any()   # the phantom is gone
    assert len(cloud_io.read_ply(tmp_path / "map.ply")) == len(P) and data["ghost_points"] >= 250 and data["points"] == len(P)


def test_the_grid_is_built_from_the_cleaned_cloud_in_memory_and_the_file_gives_the_same_walls(tmp_path):
    scans, traj = drive(planes())
    P, n, Q = world_cloud(scans, PHANTOM)
    make_map.build(P, poses_of(traj), "unused", tmp_path, res=0.05, count=counter(scans))
    from_file, _ = grid2d.rasterize(cloud_io.read_ply(tmp_path / "map_clean.ply"), 0.05, (grid2d.OBST_LO, 1.80))
    pgm = read_pgm(tmp_path / "map.pgm")
    assert pgm.shape == from_file.shape
    differ = int(((pgm == grid2d.OCC) != (from_file == grid2d.OCC)).sum())
    assert differ <= 0.005 * (pgm == grid2d.OCC).sum(), differ      # float32 on disk may move a point across a cell border: a handful of cells at most


def test_ghost_band_limits_the_judgement_and_does_not_change_the_map_above_the_obstacle_band(tmp_path):
    scans, traj = drive(planes())
    high = np.random.default_rng(4).uniform([4.85, 0.35, 2.0], [5.15, 0.65, 2.3], (300, 3))    # a phantom above the 1.80 m obstacle band
    P, n, Q = world_cloud(scans, np.vstack([PHANTOM, high]))
    make_map.build(P, poses_of(traj), "unused", tmp_path / "whole", res=0.05, count=counter(scans))
    make_map.build(P, poses_of(traj), "unused", tmp_path / "band", res=0.05, ghost_band=(0.10, 1.80), count=counter(scans))
    whole, band = (cloud_io.read_ply(tmp_path / d / "map_clean.ply") for d in ("whole", "band"))
    assert len(band) - len(whole) >= 250                                                      # the high phantom survives only with the old band
    assert (tmp_path / "whole/map.pgm").read_bytes() == (tmp_path / "band/map.pgm").read_bytes()   # and above 1.80 m it does not reach the map


def test_a_floor_that_is_not_found_stops_the_run_and_leaves_no_file(tmp_path):
    P = room(floor_pts=0)
    with pytest.raises(ValueError, match="floor"):
        make_map.build(P, np.zeros((3, 8)), "unused", tmp_path / "out", count=lambda C, m: (np.zeros(len(C), np.int32),) * 2)
    assert not (tmp_path / "out").exists() or list((tmp_path / "out").iterdir()) == []


def test_the_floor_tilt_and_the_sensor_height_come_back_for_the_report(tmp_path):
    P = room()
    pos, quat = path()
    poses = np.c_[np.arange(len(pos)) * 0.1, pos, quat]
    zero = lambda C, m: (np.zeros(len(C), np.int32), np.zeros(len(C), np.int32))
    data = make_map.build(P, poses, "unused", tmp_path, res=0.05, count=zero)
    assert abs(data["floor_tilt_deg"] - 0.6) < 0.1 and abs(data["sensor_height_m"] - 0.6) < 0.05, data     # the floor vote of the 22 m circle
    scans, traj = drive(planes())
    Pc, _, _ = world_cloud(scans)
    assert make_map.build(Pc, poses_of(traj), "unused", tmp_path / "c", res=0.05, count=counter(scans))["sensor_height_m"] is None   # a straight 10 m drive cannot vote


def test_lost_scans_warn_and_recommend_real_time_replay():
    w = make_map.scan_warning(poses=900, scans=1000, play_rate=2.0)
    assert "900 of 1000" in w and "PLAY_RATE=1" in w
    assert make_map.scan_warning(poses=990, scans=1000, play_rate=2.0) is None                  # 99 %: normal (eco01 at 1x: 99.0-99.5 %)
    assert make_map.scan_warning(poses=10, scans=None, play_rate=2.0) is None                   # no bag metadata: nothing to compare
    at_1x = make_map.scan_warning(poses=900, scans=1000, play_rate=1.0)
    assert "PLAY_RATE=1" not in at_1x and "bag" in at_1x                                      # already real time: the bag itself lost scans


@needs_engine
def test_main_runs_end_to_end_from_a_cloud_and_poses_with_the_engine(tmp_path):
    scans, traj = drive(planes(pillar=True))
    P, n, Q = world_cloud(scans, PHANTOM)
    poses = np.r_[poses_of(traj)[:1] - [0.1, *np.zeros(7)], poses_of(traj)]
    cloud_io.write_ply(tmp_path / "map.ply", P)
    np.savetxt(tmp_path / "alidarState.txt", poses)
    write_bag(tmp_path / "bag", [s[0] for s in scans], "custom")
    rc = make_map.main(["--cloud", str(tmp_path / "map.ply"), "--poses", str(tmp_path / "alidarState.txt"), "--bag", str(tmp_path / "bag"), "--out", str(tmp_path / "out"), "--res", "0.05"])
    assert rc == 0 and all((tmp_path / "out" / f).exists() for f in ("map.ply", "map_clean.ply", "map.pgm", "map.yaml", "report.json"))
    import json
    r = json.loads((tmp_path / "out/report.json").read_text())
    assert r["pipeline"] == 2 and r["scans_in_bag"] == len(scans) and r["poses"] == len(poses)
