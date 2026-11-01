"""Raw Livox frames from a bag, CustomMsg (livox_ros_driver2, decoded by hand) or PointCloud2 (the driver's xfer_format 0). frames(bag, topic, t0, t1) -> (rt, stamp_s, d) with d: t (absolute seconds of every point, float64),
x, y, z (float32), line, tag (uint8), n_slots (points of the message). rt = bag time from the start (s)."""
import sys

import numpy as np

sys.path.insert(0, "/home/narcis/Documents/GitHub/SocialTech/notes/nav_real_scripts")
import livox_custom as lc

PC2_TYPES = {1: "i1", 2: "u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}


def frames(bag, topic, t0=0.0, t1=1e9):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    s0 = float(open(bag + "/metadata.yaml").read().split("nanoseconds_since_epoch:")[1].split()[0]) / 1e9
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    ty = {k.name: k.type for k in r.get_all_topics_and_types()}; kind = ty[topic]; r.set_filter(rosbag2_py.StorageFilter(topics=[topic])); r.seek(int((s0 + t0) * 1e9))
    msg_type = get_message(kind) if kind == "sensor_msgs/msg/PointCloud2" else None
    while r.has_next():
        _, raw, tn = r.read_next(); rt = tn / 1e9 - s0
        if rt > t1: break
        if msg_type is None:
            stamp, tb, p = lc.decode(raw)
            yield rt, stamp - s0, dict(t=stamp + p["t"].astype(np.float64) * 1e-9, x=p["x"], y=p["y"], z=p["z"], line=p["line"], tag=p["tag"], n_slots=len(p))
        else:
            m = deserialize_message(raw, msg_type); dt = np.dtype({"names": [f.name for f in m.fields], "formats": [PC2_TYPES[f.datatype] for f in m.fields], "offsets": [f.offset for f in m.fields], "itemsize": m.point_step})
            p = np.frombuffer(bytes(m.data), dtype=dt, count=m.width * m.height); stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            tt = p["timestamp"].astype(np.float64) if "timestamp" in p.dtype.names else np.full(len(p), stamp)
            # the driver writes the point time in ns: absolute (~1e18) or relative to the frame (<~1e9)
            tt = tt * 1e-9 if np.nanmax(tt) > 1e12 else (tt + stamp if np.nanmax(tt) < 1e3 and np.nanmax(tt) > 1e-3 and False else tt)
            yield rt, stamp - s0, dict(t=tt, x=p["x"], y=p["y"], z=p["z"], line=p["line"] if "line" in p.dtype.names else np.zeros(len(p), "u1"), tag=p["tag"] if "tag" in p.dtype.names else np.zeros(len(p), "u1"), n_slots=len(p))
