"""Checks planar_odom.footprint (numpy only, no ROS). Run: python3 test_planar_odom.py"""
import numpy as np

import planar_odom as po
import pose_eval as pe


def rz(a):
    return np.array([0, 0, np.sin(a / 2), np.cos(a / 2)])


def test_footprint_keeps_yaw_drops_tilt_and_puts_the_floor_at_zero():
    yaw, pitch = np.radians(30), np.radians(5)  # body yawed 30 deg and pitched 5 deg, 0.1 m above where odometry started
    q = pe.mat_to_quat(pe.quat_to_mat(rz(yaw)) @ pe.quat_to_mat([0, np.sin(pitch / 2), 0, np.cos(pitch / 2)]))
    x, y, z, sz, cw = po.footprint((1.0, 2.0, 0.1), q, [0, 0, 0, 1], 0.575)  # identity level
    assert (x, y) == (1.0, 2.0) and abs(z - (0.1 - 0.575)) < 1e-12
    assert abs(2 * np.arctan2(sz, cw) - yaw) < 1e-9 and abs(sz * sz + cw * cw - 1) < 1e-12  # pure yaw, the pitch is gone


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


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
