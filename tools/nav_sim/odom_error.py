"""Relative error of an odometry against a reference trajectory, per window of path length d, in the body frame of the window start.
Invariant to the (unknown) rigid transform between the two world frames. Used to calibrate the sim odometry noise from FAST-LIO2 vs Voxel-SLAM.
  python3 -B odom_error.py <bag dir with /Odometry> <alidarState.txt> [d ...]   (needs ROS for the bag)
Planar approximation: heading = yaw of the pose, motion = xy; the tilt of the FAST-LIO2 world frame is ignored (< 1 deg)."""
import math
import os
import sys

import numpy as np


def yaw_of(q):
    x, y, z, w = q.T
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def window_errors(est_xy, est_yaw, ref_xy, ref_yaw, d, max_idx_span=None):
    """Both trajectories sampled at the same instants. Windows end where the REFERENCE has travelled d metres.
    Returns arrays: dist (reference length), along / lateral (m per m, error of the estimate's displacement in the start body frame), frac (norm of both), yaw_per_m (rad/m)."""
    s = np.r_[0.0, np.cumsum(np.hypot(*np.diff(ref_xy, axis=0).T))]
    j = np.searchsorted(s, s + d)
    ok = j < len(s)
    if max_idx_span is not None:
        ok &= (j - np.arange(len(s))) <= max_idx_span
    i = np.nonzero(ok)[0]
    j = j[i]

    def local(xy, yaw):
        c, sn = np.cos(yaw[i]), np.sin(yaw[i])
        dx, dy = xy[j, 0] - xy[i, 0], xy[j, 1] - xy[i, 1]
        return np.c_[c * dx + sn * dy, -sn * dx + c * dy]

    dist = s[j] - s[i]
    e = (local(est_xy, est_yaw) - local(ref_xy, ref_yaw)) / dist[:, None]
    dyaw = np.angle(np.exp(1j * ((est_yaw[j] - est_yaw[i]) - (ref_yaw[j] - ref_yaw[i]))))
    return {"dist": dist, "along": e[:, 0], "lateral": e[:, 1], "frac": np.hypot(e[:, 0], e[:, 1]), "yaw_per_m": dyaw / dist}


def read_odometry(bag_dir, topic="/Odometry"):
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    t, xy, q = [], [], []
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name == topic:
            m = deserialize_message(data, Odometry)
            p, o = m.pose.pose.position, m.pose.pose.orientation
            t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            xy.append((p.x, p.y))
            q.append((o.x, o.y, o.z, o.w))
    return np.array(t), np.array(xy), np.array(q)


def on_times(t_ref, xy_ref, yaw_ref, t):
    k = (t >= t_ref[0]) & (t <= t_ref[-1])
    uy = np.unwrap(yaw_ref)
    return k, np.c_[np.interp(t[k], t_ref, xy_ref[:, 0]), np.interp(t[k], t_ref, xy_ref[:, 1])], np.interp(t[k], t_ref, uy)


def main(bag, ref_path, ds):
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "nav_bench"))
    import pose_eval as pe
    t, xy, q = read_odometry(bag)
    tr, xyz, qr, _ = pe.read_poses(ref_path)
    k, rxy, ryaw = on_times(tr, xyz[:, :2], yaw_of(qr), t)
    t, xy, yaw = t[k], xy[k], np.unwrap(yaw_of(q[k]))
    print(f"{len(t)} odometry samples on the reference, {t[-1] - t[0]:.0f} s, reference path {np.hypot(*np.diff(rxy, axis=0).T).sum():.1f} m")
    print(f"{'d (m)':>6s} {'windows':>8s} {'frac med':>9s} {'frac p95':>9s} {'along med':>10s} {'along rms':>10s} {'lat rms':>8s} {'yaw/m rms':>10s} {'yaw/m p95':>10s}  (error per metre of path; yaw in rad/m)")
    for d in ds:
        r = window_errors(xy, yaw, rxy, ryaw, d, max_idx_span=int(60 * 10))
        if len(r["dist"]) < 20:
            continue
        rms = lambda a: float(np.sqrt(np.mean(a * a)))
        print(f"{d:6.1f} {len(r['dist']):8d} {np.median(r['frac']):9.4f} {np.percentile(r['frac'], 95):9.4f} {np.median(r['along']):10.4f} {rms(r['along']):10.4f} {rms(r['lateral']):8.4f} {rms(r['yaw_per_m']):10.4f} {np.percentile(np.abs(r['yaw_per_m']), 95):10.4f}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], [float(x) for x in sys.argv[3:]] or [0.5, 1.0, 2.0, 5.0, 10.0])
