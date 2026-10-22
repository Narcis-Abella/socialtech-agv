"""Drop Livox points by `tag` confidence from a Mid-360 PointCloud2 bag (fields x y z intensity tag line timestamp).
Tag (Livox protocol): bits 0-1 glue between adjacent objects, 2-3 rain/fog/dust, 4-5 other; each 0 high, 1 medium, 2 low confidence.
Bits 6-7 are reserved (set in 1-3 % of our points), so they are ignored.
  mode low : drop points where any field is 2 (low confidence)
  mode high: keep only points with all three fields 0
usage: tag_filter.py <in_bag> <lidar_topic> <imu_topic> <low|high> <out_dir>"""
import sys
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore

src, lidar, imu, mode, dst = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], Path(sys.argv[5])
assert mode in ("low", "high")
ts = get_typestore(Stores.ROS2_JAZZY)
IN = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<f4"), ("tag", "u1"), ("line", "u1"), ("ts", "<f8")])
assert IN.itemsize == 26

def keep(tag):
    fields = (tag & 3, (tag >> 2) & 3, (tag >> 4) & 3)
    if mode == "high":
        return (fields[0] == 0) & (fields[1] == 0) & (fields[2] == 0)
    return (fields[0] != 2) & (fields[1] != 2) & (fields[2] != 2)

kept = total = 0
with AnyReader([Path(src)], default_typestore=ts) as r, Writer(dst, version=8, storage_plugin=StoragePlugin.MCAP) as w:
    cl = w.add_connection(lidar, "sensor_msgs/msg/PointCloud2", typestore=ts)
    ci = w.add_connection(imu, "sensor_msgs/msg/Imu", typestore=ts)
    for c, t, raw in r.messages(connections=[x for x in r.connections if x.topic in (lidar, imu)]):
        if c.topic == imu:
            w.write(ci, t, raw)
            continue
        m = r.deserialize(raw, c.msgtype)
        a = np.frombuffer(m.data, IN, m.width * m.height)
        b = a[keep(a["tag"])]
        kept, total = kept + len(b), total + len(a)
        m.data = np.frombuffer(b.tobytes(), np.uint8)
        m.width, m.height, m.row_step = len(b), 1, 26 * len(b)
        w.write(cl, t, ts.serialize_cdr(m, "sensor_msgs/msg/PointCloud2"))
print(f"kept {kept}/{total} points ({kept / total * 100:.1f} %)")
