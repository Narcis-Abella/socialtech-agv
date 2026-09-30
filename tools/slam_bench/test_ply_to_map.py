"""Checks ply_to_map.py on a synthetic room with a known floor tilt. Run: python3 test_ply_to_map.py"""
import tempfile
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

import ply_to_map

RNG = np.random.default_rng(7)
FLOOR_Z = -0.6    # floor below the sensor (the PLY origin is the sensor, not the floor)
ROOM_H = 2.5      # floor-to-ceiling
TILT = 0.6        # deg, the whole map is rotated about y (what a biased gravity init does)
RES = 0.05


def tilt_rot(deg):
    a = np.radians(deg)
    return np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])


def room(tilt=TILT, floor_noise=0.01, floor_pts=60000, ceiling_pts=100000, dz=0.0):
    """Points of a 10 x 8 m room in its level frame, then tilted as a whole. Walls at x=+-5 and y=+-4; the y=+4 wall has a
    doorway (|x|<1) that only keeps its lintel above 2.0 m. More ceiling than floor points: the ceiling is the bigger plane.
    The floor has a 1 m hole at (2..3, -3..-2): the LiDAR saw nothing there. dz shifts the whole cloud (origin below the floor when dz > 0.6)."""
    pts = []
    f = RNG.uniform([-5, -4], [5, 4], (floor_pts, 2))
    f = f[~((f[:, 0] > 2) & (f[:, 0] < 3) & (f[:, 1] > -3) & (f[:, 1] < -2))]
    floor_pts = len(f)
    pts.append(np.c_[f, FLOOR_Z + RNG.normal(0, floor_noise, floor_pts)])
    c = RNG.uniform([-5, -4], [5, 4], (ceiling_pts, 2))
    pts.append(np.c_[c, np.full(ceiling_pts, FLOOR_Z + ROOM_H)])
    for axis, val in ((0, -5.0), (0, 5.0), (1, -4.0), (1, 4.0)):
        n = 6000
        along = RNG.uniform(-4 if axis == 0 else -5, 4 if axis == 0 else 5, n)
        z = FLOOR_Z + RNG.uniform(0.0, ROOM_H, n)
        if axis == 1 and val == 4.0:
            z = np.where(np.abs(along) < 1.0, FLOOR_Z + RNG.uniform(2.0, ROOM_H, n), z)  # doorway: only the lintel
        pts.append(np.c_[np.full(n, val), along, z] if axis == 0 else np.c_[along, np.full(n, val), z])
    P = np.vstack(pts) + [0, 0, dz]
    return P @ tilt_rot(tilt).T


def path(tilt=TILT, extra=0.0, straight=False, arc=None):
    """LiDAR poses (positions (N, 3), quaternions xyzw (N, 4)) in the same tilted frame as room(): a 22 m circle at the sensor height
    (z=0 in the level frame) with the robot heading along the path, a 10 m line, or an arc of `arc` degrees. The robot's yaw axis is the floor
    normal; extra tilts the whole trajectory further (a robot that is not parallel to the floor)."""
    yaw = np.linspace(0, np.radians(arc) if arc else 2 * np.pi, 400)
    P = np.c_[np.linspace(-5, 5, 400), np.zeros(400), np.zeros(400)] if straight else np.c_[3.5 * np.cos(yaw), 3.5 * np.sin(yaw), np.zeros(400)]
    if straight:
        yaw = np.zeros(400)
    R = Rotation.from_matrix(tilt_rot(tilt) @ tilt_rot(extra)) * Rotation.from_rotvec(np.c_[np.zeros(400), np.zeros(400), yaw])
    return P @ tilt_rot(extra).T @ tilt_rot(tilt).T, R.as_quat()


def write_ply(path, P, intensity=True):
    cols = [("x", "<f4"), ("y", "<f4"), ("z", "<f4")] + ([("intensity", "<f4")] if intensity else [])
    a = np.zeros(len(P), dtype=cols)
    a["x"], a["y"], a["z"] = P[:, 0], P[:, 1], P[:, 2]
    head = "ply\nformat binary_little_endian 1.0\nelement vertex %d\n" % len(P) + "".join(f"property float {n}\n" for n, _ in cols) + "end_header\n"
    Path(path).write_bytes(head.encode() + a.tobytes())


def cell(grid, origin, x, y, res=RES):
    """Value of the cell containing (x, y); image row 0 is the top (max y), as in map_server PGMs."""
    return grid[grid.shape[0] - 1 - int((y - origin[1]) / res), int((x - origin[0]) / res)]


def test_read_ply_roundtrip_with_and_without_intensity():
    P = RNG.uniform(-3, 3, (500, 3))
    with tempfile.TemporaryDirectory() as tmp:
        for intensity in (True, False):
            write_ply(Path(tmp) / "a.ply", P, intensity)
            assert np.allclose(ply_to_map.read_ply(Path(tmp) / "a.ply"), P, atol=1e-5), intensity


def test_level_to_floor_measures_tilt_and_puts_the_floor_at_zero():
    Q, info = ply_to_map.level_to_floor(room())
    assert abs(info["tilt_deg"] - TILT) < 0.05, info  # measured on the floor, although the ceiling has twice the points
    assert info["resid_std"] < 0.02, info
    floor = Q[(Q[:, 2] > -0.1) & (Q[:, 2] < 0.1)]
    assert len(floor) >= 50000 and abs(np.median(floor[:, 2])) < 0.01, len(floor)
    assert abs(np.percentile(Q[:, 2], 99.9) - ROOM_H) < 0.05  # the ceiling is ROOM_H above the floor


def expect_error(P, *words, **kw):
    try:
        ply_to_map.level_to_floor(P, **kw)
    except ValueError as e:
        assert all(w in str(e).lower() for w in words), e
    else:
        raise AssertionError(f"no error, expected {words}")


def test_floor_is_found_when_it_is_a_small_share_and_the_ceiling_is_much_bigger():
    _, info = ply_to_map.level_to_floor(room(floor_pts=6000, ceiling_pts=100000))  # a Mid-360 looking up sees little floor
    assert abs(info["tilt_deg"] - TILT) < 0.1, info


def test_origin_below_the_floor_is_fine():
    Q, info = ply_to_map.level_to_floor(room(dz=1.5))  # no 'floor is below z=0' assumption
    assert abs(info["tilt_deg"] - TILT) < 0.05 and abs(np.percentile(Q[:, 2], 99.9) - ROOM_H) < 0.05, info


def test_start_tilt_up_to_10_deg_is_levelled_and_12_deg_is_refused():
    _, info = ply_to_map.level_to_floor(room(tilt=6.0))
    assert abs(info["tilt_deg"] - 6.0) < 0.1, info
    expect_error(room(tilt=12.0), "floor")


def test_no_floor_fails_loudly_instead_of_returning_the_ceiling():
    expect_error(room(floor_pts=0), "floor")


def test_too_little_floor_is_refused_by_min_floor():
    expect_error(room(floor_pts=150), "support")
    _, info = ply_to_map.level_to_floor(room(floor_pts=150), min_floor=50)  # the user can lower the gate
    assert info["n_floor"] < 250


def test_a_rough_floor_is_refused():
    expect_error(room(floor_noise=0.12), "floor")


def test_trajectory_gate_accepts_a_parallel_path_and_refuses_a_tilted_one():
    P = room()
    _, info = ply_to_map.level_to_floor(P, traj=path())
    pos, ori = info["traj"]["position"], info["traj"]["orientation"]
    assert pos["angle_deg"] < 0.3 and pos["height"] > 0.5 and ori["angle_deg"] < 0.3, info
    expect_error(P, "trajectory", traj=path(extra=3.0))
    _, info = ply_to_map.level_to_floor(P, traj=path(straight=True))  # a straight line defines no plane and no yaw axis: skipped, not failed
    assert info["traj"] == {"position": "skipped", "orientation": "skipped"}, info


def test_yaw_axis_is_the_floor_normal_whatever_the_sensor_mount():
    _, quat = path()
    axis, ratio, yaw = ply_to_map.yaw_axis(quat)
    floor_normal = tilt_rot(TILT) @ [0, 0, 1]
    assert abs(axis @ floor_normal) > np.cos(np.radians(0.05)) and ratio < 0.01 and abs(yaw - 360) < 2, (axis, ratio, yaw)
    mount = Rotation.from_euler("xyz", [25, -40, 70], degrees=True)  # the LiDAR sits rotated on the robot: R_lidar = R_robot * M
    axis2, _, _ = ply_to_map.yaw_axis((Rotation.from_quat(quat) * mount).as_quat())
    assert abs(axis2 @ axis) > np.cos(np.radians(0.01)), (axis, axis2)


def test_a_short_turn_does_not_vote_with_the_yaw_axis():
    _, info = ply_to_map.level_to_floor(room(), traj=path(arc=60))
    assert info["traj"]["orientation"] == "skipped", info  # under 90 deg of turning the axis is not trustworthy


def test_rasterize_keeps_walls_drops_lintel_and_ceiling_and_marks_floor_free():
    Q, _ = ply_to_map.level_to_floor(room())
    grid, origin = ply_to_map.rasterize(Q, RES, ceil=1.80, floor_band=0.10, min_hits=2)
    assert cell(grid, origin, 5.0 - 0.02, 0.0) == ply_to_map.OCC        # side wall
    assert cell(grid, origin, 3.0, 4.0 - 0.02) == ply_to_map.OCC        # wall beside the doorway
    assert cell(grid, origin, 0.0, 4.0 - 0.02) != ply_to_map.OCC        # doorway: the lintel is above 1.80 m
    open_floor = [cell(grid, origin, x, y) for x in np.arange(-3, 1, 0.05) for y in np.arange(-2, 2, 0.05)]
    assert np.mean(np.array(open_floor) == ply_to_map.FREE) > 0.7        # open floor: free where a floor point fell (about 1.9 per 5 cm cell)
    assert cell(grid, origin, 2.5, -2.5) == ply_to_map.UNK             # the floor hole: nothing seen there
    occ = (grid == ply_to_map.OCC).sum()
    assert occ < 0.05 * grid.size, occ                                   # the ceiling did not turn the room solid


def test_small_gaps_in_explored_floor_are_filled_but_a_big_hole_is_not():
    Q, _ = ply_to_map.level_to_floor(room())
    raw, origin = ply_to_map.rasterize(Q, RES, fill=0)
    filled, _ = ply_to_map.rasterize(Q, RES, fill=0.3)
    open_floor = lambda g: np.mean([cell(g, origin, x, y) == ply_to_map.FREE for x in np.arange(-3, 1, 0.05) for y in np.arange(-2, 2, 0.05)])
    assert open_floor(raw) < 0.9 and open_floor(filled) > 0.97, (open_floor(raw), open_floor(filled))  # the speckle is closed
    assert cell(filled, origin, 2.5, -2.5) == ply_to_map.UNK            # a 1 m hole is not invented
    assert ((raw == ply_to_map.OCC) == (filled == ply_to_map.OCC)).all()  # never touches obstacles


def test_isolated_outliers_do_not_inflate_the_grid():
    P = room()
    far = np.r_[P, RNG.uniform([-100, -100, -0.5], [100, 100, 1.5], (30, 3))]  # 30 stray points scattered far away
    Q, _ = ply_to_map.level_to_floor(far)
    grid, _ = ply_to_map.rasterize(Q, RES)
    assert grid.shape[0] < 300 and grid.shape[1] < 300, grid.shape      # the room is 202 x 162 cells, not 3000


def test_write_map_makes_pgm_and_map_server_yaml():
    grid = np.full((6, 8), ply_to_map.UNK, dtype=np.uint8)
    grid[0, 0], grid[5, 7] = ply_to_map.OCC, ply_to_map.FREE
    with tempfile.TemporaryDirectory() as tmp:
        ply_to_map.write_map(Path(tmp) / "m", grid, RES, (-1.5, 2.0))
        raw = (Path(tmp) / "m.pgm").read_bytes()
        assert raw.startswith(b"P5\n8 6\n255\n") and raw[-48:] == grid.tobytes()
        y = (Path(tmp) / "m.yaml").read_text()
        # map_server reads a pixel as free when (255 - pixel) / 255 <= free_thresh: 205 (unknown) is 0.19608, so 0.25 would load unknown as free
        for line in ("image: m.pgm", "resolution: 0.05", "origin: [-1.5, 2.0, 0.0]", "negate: 0", "occupied_thresh: 0.65", "free_thresh: 0.196"):
            assert line in y, (line, y)


def test_main_runs_end_to_end_with_an_optional_tum_trajectory():
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        write_ply(tmp / "room.ply", room())
        ply_to_map.main([str(tmp / "room.ply"), str(tmp / "room")])
        assert (tmp / "room.pgm").exists() and (tmp / "room.yaml").exists()
        xyz, quat = path()
        np.savetxt(tmp / "traj.txt", np.c_[np.arange(400) * 0.1, xyz, quat])
        assert all(np.allclose(a, b) for a, b in zip(ply_to_map.read_traj(tmp / "traj.txt"), (xyz, quat)))
        ply_to_map.main([str(tmp / "room.ply"), str(tmp / "room2"), "--traj", str(tmp / "traj.txt")])
        assert (tmp / "room2.pgm").exists()


if __name__ == "__main__":
    test_read_ply_roundtrip_with_and_without_intensity()
    test_level_to_floor_measures_tilt_and_puts_the_floor_at_zero()
    test_floor_is_found_when_it_is_a_small_share_and_the_ceiling_is_much_bigger()
    test_origin_below_the_floor_is_fine()
    test_start_tilt_up_to_10_deg_is_levelled_and_12_deg_is_refused()
    test_no_floor_fails_loudly_instead_of_returning_the_ceiling()
    test_too_little_floor_is_refused_by_min_floor()
    test_a_rough_floor_is_refused()
    test_trajectory_gate_accepts_a_parallel_path_and_refuses_a_tilted_one()
    test_yaw_axis_is_the_floor_normal_whatever_the_sensor_mount()
    test_a_short_turn_does_not_vote_with_the_yaw_axis()
    test_rasterize_keeps_walls_drops_lintel_and_ceiling_and_marks_floor_free()
    test_small_gaps_in_explored_floor_are_filled_but_a_big_hole_is_not()
    test_isolated_outliers_do_not_inflate_the_grid()
    test_write_map_makes_pgm_and_map_server_yaml()
    test_main_runs_end_to_end_with_an_optional_tum_trajectory()
    print("ok")
