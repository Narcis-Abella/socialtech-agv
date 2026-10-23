"""Noise-free LaserScan traced on an occupancy map from a pose, and a bag rewriter that replaces the /scan of a recorded bag by such scans along the reference trajectory.
Purpose: separate the scan chain (cloud -> pointcloud_to_laserscan) from the environment/AMCL as the cause of a localisation failure: same odometry and map, ideal scan.
  python3 -B synth_scan.py <in bag dir> <out bag dir> <map.yaml> <alidarState.txt>      (needs ROS)
Sphere tracing on the distance-to-occupied field: a beam hits when it comes within half a cell of an occupied cell centre (accuracy about one cell, 5 cm on the maps used)."""
import math
import os
import sys

import numpy as np

import metrics


def scan(occ, res, origin, pose, n, angle_min, inc, rmax, dist=None, max_steps=400):
    """Ranges (n,) of the beams from pose (x, y, yaw) on the map `occ` (image rows top to bottom, True = occupied); inf where nothing is hit within rmax or the beam leaves the map."""
    dist = metrics.distance_to_occupied(occ, res) if dist is None else dist
    h, w = occ.shape
    a = angle_min + inc * np.arange(n) + pose[2]
    c, s = np.cos(a), np.sin(a)
    t = np.zeros(n)
    out = np.full(n, np.inf)
    live = np.ones(n, bool)
    for _ in range(max_steps):
        x, y = pose[0] + t * c, pose[1] + t * s
        col = np.floor((x - origin[0]) / res).astype(int)
        row = h - 1 - np.floor((y - origin[1]) / res).astype(int)
        inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
        live &= inside & (t <= rmax)
        if not live.any():
            break
        d = np.full(n, np.inf)
        d[live] = dist[row[live], col[live]]
        hit = live & (d <= 0.5 * res)
        out[hit] = t[hit]
        live &= ~hit
        t = t + np.where(live, np.maximum(d - 0.5 * res, 0.25 * res), 0.0)
    out[out > rmax] = np.inf
    return out


def rewrite_bag(in_bag, out_bag, map_yaml, ref_path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import LaserScan
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "nav_bench"))
    import pose_eval as pe
    m = pe.read_map_yaml(map_yaml)
    occ = pe.read_pgm(m["image"]) < 50
    dist = metrics.distance_to_occupied(occ, m["resolution"])
    t_ref, ref = pe.reference_map(ref_path, m["T_map_world"])
    ref = ref.copy()
    ref[:, 2] = np.unwrap(ref[:, 2])
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=in_bag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=out_bag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    for tm in reader.get_all_topics_and_types():
        writer.create_topic(tm)
    n_scan = n_skipped = 0
    while reader.has_next():
        name, data, stamp = reader.read_next()
        if name == "/scan":
            msg = deserialize_message(data, LaserScan)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if t_ref[0] <= t <= t_ref[-1]:
                pose = [np.interp(t, t_ref, ref[:, k]) for k in range(3)]
                msg.ranges = scan(occ, m["resolution"], m["origin"], pose, len(msg.ranges), msg.angle_min, msg.angle_increment, msg.range_max, dist).tolist()
                msg.intensities = []
                data = serialize_message(msg)
                n_scan += 1
            else:
                n_skipped += 1
                continue                     # no reference pose: drop the scan rather than keep a real one in a synthetic bag
        writer.write(name, data, stamp)
    print(f"{n_scan} scans replaced, {n_skipped} dropped (outside the reference)")


if __name__ == "__main__":
    rewrite_bag(*sys.argv[1:5])
