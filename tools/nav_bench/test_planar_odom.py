"""Checks planar_odom.footprint (numpy only, no ROS). Run: python3 test_planar_odom.py"""
import numpy as np

import planar_odom as po
import map_frame as pe


def rz(a):
    return np.array([0, 0, np.sin(a / 2), np.cos(a / 2)])


def test_footprint_keeps_yaw_drops_tilt_and_puts_the_floor_at_zero():
    yaw, pitch = np.radians(30), np.radians(5)  # body yawed 30 deg and pitched 5 deg, 0.1 m above where odometry started
    q = pe.mat_to_quat(pe.quat_to_mat(rz(yaw)) @ pe.quat_to_mat([0, np.sin(pitch / 2), 0, np.cos(pitch / 2)]))
    x, y, z, sz, cw = po.footprint((1.0, 2.0, 0.1), q, [0, 0, 0, 1], 0.575)  # identity level
    assert (x, y) == (1.0, 2.0) and abs(z - (0.1 - 0.575)) < 1e-12
    assert abs(2 * np.arctan2(sz, cw) - yaw) < 1e-9 and abs(sz * sz + cw * cw - 1) < 1e-12  # pure yaw, the pitch is gone


def test_base_footprint_goes_under_the_lidar_when_given_its_offset_in_the_body():
    yaw = np.radians(90)  # the IMU (body) is 1.1 / 2.3 cm from the lidar: turning in place, the point under the lidar must not move
    x, y, _, _, _ = po.footprint((1.0, 2.0, 0.0), rz(yaw), [0, 0, 0, 1], 0.575, (-0.011, -0.02329))
    assert np.allclose([x, y], [1.0 + 0.02329, 2.0 - 0.011])
    for a in np.radians([0, 90, 180, 270]):  # the lidar fixed at the origin, the IMU orbiting it: the footprint stays put
        imu = (0.011 * np.cos(a) - 0.02329 * np.sin(a), 0.011 * np.sin(a) + 0.02329 * np.cos(a), 0.0)
        assert np.allclose(po.footprint(imu, rz(a), [0, 0, 0, 1], 0.5, (-0.011, -0.02329))[:2], 0.0)


def test_level_rotation_is_applied_to_position_and_heading():
    x, y, z, sz, cw = po.footprint((1.0, 0.0, 0.0), [0, 0, 0, 1], rz(np.pi / 2), 0.5)  # odom is camera_init turned 90 deg about z
    assert np.allclose([x, y, z], [0.0, 1.0, -0.5]) and abs(2 * np.arctan2(sz, cw) - np.pi / 2) < 1e-9


def test_a_tilted_start_frame_is_levelled_so_the_floor_is_flat():
    tilt = np.radians(1.0)  # FAST-LIO2's world frame is tilted 1 deg about y: the floor is a plane z = -h + x*tan(tilt) there
    R_t = pe.quat_to_mat([0, np.sin(tilt / 2), 0, np.cos(tilt / 2)])  # level = R_t^T takes that frame to the levelled one
    level = pe.mat_to_quat(R_t.T)
    for xs in (0.0, 5.0, 10.0):
        p_floor_point = R_t @ np.array([xs, 0.0, -0.575])  # a floor point 0.575 m below the sensor, xs m ahead, in the tilted frame
        assert abs((pe.quat_to_mat(level) @ p_floor_point)[2] + 0.575) < 1e-12  # back at the same height below: flat


def test_corrected_level_is_the_correction_times_the_level_and_lifts_the_floor_back_to_flat():
    tilt = np.radians(0.7)
    c = np.array([np.sin(tilt / 2), 0, 0, np.cos(tilt / 2)])  # C: a roll of 0.7 deg, what level_corrector measured
    lvl = rz(np.radians(40))
    got = pe.quat_to_mat(po.corrected_level(lvl, c))
    assert np.allclose(got, pe.quat_to_mat(c) @ pe.quat_to_mat(lvl), atol=1e-12)
    # a robot 10 m out along the (levelled) y axis, on a floor that FAST-LIO2 sees tilted by C^T: with the corrected level its z is the same as at the origin
    p_tilted = pe.quat_to_mat(lvl).T @ pe.quat_to_mat(c).T @ np.array([0.0, 10.0, 0.0])
    z_corrected = po.footprint(p_tilted, [0, 0, 0, 1], po.corrected_level(lvl, c), 0.5)[2]
    z_static = po.footprint(p_tilted, [0, 0, 0, 1], lvl, 0.5)[2]
    assert abs(z_corrected + 0.5) < 1e-12 and abs(z_static + 0.5) > 0.1  # 10 m * sin(0.7 deg) = 12 cm of phantom height without the correction


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
