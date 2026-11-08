"""ROS-free maths of the relocalizer's handoff: the accepted map<-source pose carried to the body NOW, and the planar mount.

Used by reloc_live.py. (Until 2026-11-08 this module also held the state machine of the previous online node, reloc_node.py, and its
gates; see git history.)"""
import json
import math

import numpy as np
from scipy.spatial.transform import Rotation

S2_EXPOSURE_S = 3.   # seconds of raw frames the search is given


def handoff_pose(T_map_src, T_level_local, T_ci_first, T_ci_now):
    """Accepted map<-source pose -> (x, y, yaw) of the body proxy in map NOW, and the odom levelling quaternion.

    T_map_ci = T_map_src @ T_level_local @ inv(T_ci_first);
    the footprint proxy is the IMU origin (the sensor->base extrinsic is the caller's `ext`).
    """
    T_src_ci = np.asarray(T_level_local) @ np.linalg.inv(T_ci_first)
    T_map_ci = np.asarray(T_map_src) @ T_src_ci
    T_now = T_map_ci @ T_ci_now
    yaw = math.atan2(T_now[1, 0], T_now[0, 0])
    q = Rotation.from_matrix(T_src_ci[:3, :3]).as_quat()
    return T_map_ci, (float(T_now[0, 3]), float(T_now[1, 3]), yaw), q


def compose_planar(xyyaw, ext):
    """Pose of base_footprint from the pose of the IMU-origin proxy and the planar mount extrinsic (x, y, yaw of the base in the
    levelled body frame). (0, 0, 0) = the proxy itself (PROVISIONAL default: the mount has not been measured)."""
    x, y, w = xyyaw
    ex, ey, ew = ext
    return (x + math.cos(w) * ex - math.sin(w) * ey, y + math.sin(w) * ex + math.cos(w) * ey, w + ew)


def dump(path, obj):
    with open(path, 'w') as f:
        json.dump(obj, f, indent=2, default=float)
        f.write('\n')
