"""floor_level: floor finding, levelling, wall alignment and the trajectory vote, on synthetic rooms."""
import numpy as np
from scipy.spatial.transform import Rotation

import floor_level
from synth_room import FLOOR_Z, RES, RNG, ROOM_H, TILT, extent, furnished_room, path, room, round_room, tilt_rot

def test_level_to_floor_measures_tilt_and_puts_the_floor_at_zero():
    Q, info = floor_level.level_to_floor(room())
    assert abs(info["tilt_deg"] - TILT) < 0.05, info  # measured on the floor, although the ceiling has twice the points
    assert info["resid_std"] < 0.02, info
    floor = Q[(Q[:, 2] > -0.1) & (Q[:, 2] < 0.1)]
    assert len(floor) >= 50000 and abs(np.median(floor[:, 2])) < 0.01, len(floor)
    assert abs(np.percentile(Q[:, 2], 99.9) - ROOM_H) < 0.05


def expect_error(P, *words, **kw):
    try:
        floor_level.level_to_floor(P, **kw)
    except ValueError as e:
        assert all(w in str(e).lower() for w in words), e
    else:
        raise AssertionError(f"no error, expected {words}")


def test_floor_is_found_when_it_is_a_small_share_and_the_ceiling_is_much_bigger():
    _, info = floor_level.level_to_floor(room(floor_pts=6000, ceiling_pts=100000))  # a Mid-360 looking up sees little floor
    assert abs(info["tilt_deg"] - TILT) < 0.1, info


def test_origin_below_the_floor_is_fine():
    Q, info = floor_level.level_to_floor(room(dz=1.5))  # no 'floor is below z=0' assumption
    assert abs(info["tilt_deg"] - TILT) < 0.05 and abs(np.percentile(Q[:, 2], 99.9) - ROOM_H) < 0.05, info


def test_start_tilt_up_to_10_deg_is_levelled_and_12_deg_is_refused():
    _, info = floor_level.level_to_floor(room(tilt=6.0))
    assert abs(info["tilt_deg"] - 6.0) < 0.1, info
    expect_error(room(tilt=12.0), "floor")


def test_no_floor_fails_loudly_instead_of_returning_the_ceiling():
    expect_error(room(floor_pts=0), "floor")


def test_too_little_floor_is_refused_by_min_floor():
    expect_error(room(floor_pts=150), "support")
    _, info = floor_level.level_to_floor(room(floor_pts=150), min_floor=50)  # the user can lower the gate
    assert info["n_floor"] < 250


def test_a_rough_floor_is_refused():
    expect_error(room(floor_noise=0.12), "floor")


def test_trajectory_gate_accepts_a_parallel_path_and_refuses_a_tilted_one():
    P = room()
    _, info = floor_level.level_to_floor(P, traj=path())
    pos, ori = info["traj"]["position"], info["traj"]["orientation"]
    assert pos["angle_deg"] < 0.3 and pos["height"] > 0.5 and ori["angle_deg"] < 0.3, info
    expect_error(P, "trajectory", traj=path(extra=3.0))
    _, info = floor_level.level_to_floor(P, traj=path(straight=True))  # a straight line defines no plane and no yaw axis: skipped, not failed
    assert info["traj"] == {"position": "skipped", "orientation": "skipped"}, info


def test_yaw_axis_is_the_floor_normal_whatever_the_sensor_mount():
    _, quat = path()
    axis, ratio, yaw = floor_level.yaw_axis(quat)
    floor_normal = tilt_rot(TILT) @ [0, 0, 1]
    assert abs(axis @ floor_normal) > np.cos(np.radians(0.05)) and ratio < 0.01 and abs(yaw - 360) < 2, (axis, ratio, yaw)
    mount = Rotation.from_euler("xyz", [25, -40, 70], degrees=True)  # the LiDAR sits rotated on the robot: R_lidar = R_robot * M
    axis2, _, _ = floor_level.yaw_axis((Rotation.from_quat(quat) * mount).as_quat())
    assert abs(axis2 @ axis) > np.cos(np.radians(0.01)), (axis, axis2)


def test_a_short_turn_does_not_vote_with_the_yaw_axis():
    _, info = floor_level.level_to_floor(room(), traj=path(arc=60))
    assert info["traj"]["orientation"] == "skipped", info


def test_manhattan_yaw_recovers_the_rotation_and_reports_how_manhattan_it_is():
    th0 = np.radians(23.0)
    normals = np.concatenate([np.c_[np.cos(th0 + k * np.pi / 2 + RNG.normal(0, 0.01, 500)), np.sin(th0 + k * np.pi / 2 + RNG.normal(0, 0.01, 500))] for k in range(4)])
    yaw, strength = floor_level.manhattan_yaw(normals)
    assert abs(yaw + 23.0) < 0.2 and strength > 0.95, (yaw, strength)   # turn the cloud by -23 deg and the walls are axis-aligned
    a = RNG.uniform(0, 2 * np.pi, 4000)
    assert floor_level.manhattan_yaw(np.c_[np.cos(a), np.sin(a)])[1] < 0.1


def test_long_walls_decide_the_turn_not_the_furniture_between_them():
    _, info = floor_level.level_to_floor(furnished_room(2.5))
    assert abs(info["yaw_deg"] + 2.5) < 0.3, info


def test_the_long_side_ends_up_horizontal():
    for yaw in (0.0, 90.0, 90.0 + 23.0, -90.0 + 7.0):  # the 10 x 8 m room as SLAM saw it along x or along y, and skewed
        P = room(yaw=yaw)
        Q, info = floor_level.level_to_floor(P)
        ex, ey = extent(Q)
        assert ex > 9 and ey < 9, (yaw, ex, ey)  # long side along x, walls still on the axes
        T = info["T"]
        assert np.allclose(P @ T[:3, :3].T + T[:3, 3], Q, atol=1e-9)


def test_a_round_room_is_not_turned():
    _, info = floor_level.level_to_floor(round_room())
    assert info["yaw_deg"] == 0 and info["manhattan"] < 0.5, info


def test_transform_maps_the_input_cloud_onto_the_output():
    P = room(yaw=23.0)
    Q, info = floor_level.level_to_floor(P)
    T = info["T"]
    assert np.allclose(P @ T[:3, :3].T + T[:3, 3], Q, atol=1e-9), "T_map_world must reproduce the output cloud"
    assert np.allclose(T[3], [0, 0, 0, 1]) and abs(np.linalg.det(T[:3, :3]) - 1) < 1e-9


def test_refine_yaw_without_wall_points_keeps_the_given_yaw():
    # --min-manhattan 0 on a cloud with nothing between 0.2 and 1.6 m: no wall points, so the turn stays the one the normals gave
    assert floor_level.refine_yaw(np.zeros((0, 2)), 3.0) == 3.0


def test_a_square_room_is_not_turned():
    P = room(tilt=0.0)
    P[:, 1] *= 1.25  # 10 x 10 m
    _, info = floor_level.level_to_floor(P)
    assert abs(info["yaw_deg"]) < 1.0, info


def test_the_cloud_is_always_aligned_and_landscape_when_it_is_manhattan_enough():
    Q, info = floor_level.level_to_floor(room(yaw=90.0))   # no option turns this off any more
    ex, ey = extent(Q)
    assert ex > 9 and ey < 9 and info["manhattan"] > 0.8, (ex, ey, info)

