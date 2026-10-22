"""Python reference of the C++ node docker/ros/planar_odom: the odom -> base_footprint TF from FAST-LIO2's /Odometry (camera_init -> body), in the floor-levelled odom frame.
odom = camera_init rotated by `level` (static TF from the launch file; pose_eval.py level): z is the floor normal and the axes follow the walls, as in
the PGM. base_footprint sits under the body: same x, y and yaw (in odom), z = body z - sensor_height (the floor is z = 0 there), no roll/pitch.
A cloud transformed into base_footprint is therefore levelled and its z means 'height above the floor' (the frame ply_to_map cuts in).
"""
import numpy as np

import pose_eval as pe


def footprint(p, q, level, sensor_height):
    """(x, y, z, qz, qw) of base_footprint in odom. p, q: body position and quaternion (x, y, z, w) in camera_init; level: quaternion of odom<-camera_init."""
    R_l = pe.quat_to_mat(level)
    po, Ro = R_l @ np.asarray(p, float), R_l @ pe.quat_to_mat(q)
    yaw = np.arctan2(Ro[1, 0], Ro[0, 0])
    return po[0], po[1], po[2] - sensor_height, np.sin(yaw / 2), np.cos(yaw / 2)
