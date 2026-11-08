"""Python reference of the C++ node docker/ros/planar_odom: the odom -> base_footprint TF from FAST-LIO2's /Odometry (camera_init -> body), in the floor-levelled odom frame.
odom = camera_init rotated by `level` (static TF from the launch file; map_frame.py level): z is the floor normal and the axes follow the walls, as in
the PGM. base_footprint sits under the point lidar_xy of the body (default: the body itself): same yaw (in odom), z = body z - sensor_height (the floor is z = 0 there), no roll/pitch.
A cloud transformed into base_footprint is therefore levelled and its z means 'height above the floor' (the frame make_map cuts in).
"""
import numpy as np

import map_frame as pe


def footprint(p, q, level, sensor_height, lidar_xy=(0.0, 0.0)):
    """(x, y, z, qz, qw) of base_footprint in odom. p, q: body position and quaternion (x, y, z, w) in camera_init; level: quaternion of odom<-camera_init; lidar_xy: the lidar in the body."""
    R_l = pe.quat_to_mat(level)
    po, Ro = R_l @ np.asarray(p, float), R_l @ pe.quat_to_mat(q)
    yaw = np.arctan2(Ro[1, 0], Ro[0, 0])
    c, s = np.cos(yaw), np.sin(yaw)
    return po[0] + c * lidar_xy[0] - s * lidar_xy[1], po[1] + s * lidar_xy[0] + c * lidar_xy[1], po[2] - sensor_height, np.sin(yaw / 2), np.cos(yaw / 2)


def corrected_level(level, c):
    """Quaternion of odom<-camera_init once level_corrector's roll/pitch correction c (a rotation of odom, quaternion x, y, z, w) is applied: C * level."""
    return pe.mat_to_quat(pe.quat_to_mat(c) @ pe.quat_to_mat(level))
