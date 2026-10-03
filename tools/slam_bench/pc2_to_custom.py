"""Mid-360 PointCloud2 bag (x y z intensity tag line timestamp[f64 abs ns]) -> ROS2 mcap with livox_ros_driver2/CustomMsg + IMU.
CDR is written by hand (a python object per point is too slow); the first message is round-tripped through rosbags as a check.
usage: pc2_to_custom.py <in_bag> <lidar_topic> <imu_topic> <out_dir>"""
import struct, sys
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

MSG = {"livox_ros_driver2/msg/CustomPoint": "uint32 offset_time\nfloat32 x\nfloat32 y\nfloat32 z\nuint8 reflectivity\nuint8 tag\nuint8 line\n",
       "livox_ros_driver2/msg/CustomMsg": "std_msgs/Header header\nuint64 timebase\nuint32 point_num\nuint8 lidar_id\nuint8[3] rsvd\nCustomPoint[] points\n"}
ts = get_typestore(Stores.ROS2_JAZZY)
for name, text in MSG.items():
    ts.register(get_types_from_msg(text, name))
PT = np.dtype([("t", "<u4"), ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("r", "u1"), ("tag", "u1"), ("line", "u1"), ("pad", "u1")])
IN = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<f4"), ("tag", "u1"), ("line", "u1"), ("ts", "<f8")])
FRAME = b"livox_frame\x00"
assert PT.itemsize == 20 and IN.itemsize == 26  # CDR pads each 15-byte CustomPoint to 20

def custom_cdr(m):
    a = np.frombuffer(m.data, IN, m.width * m.height)
    timebase = int(a["ts"].min())
    p = np.zeros(len(a), PT)
    p["t"] = (a["ts"] - timebase).astype(np.uint32)
    p["x"], p["y"], p["z"], p["r"] = a["x"], a["y"], a["z"], np.clip(a["i"], 0, 255).astype(np.uint8)
    p["tag"], p["line"] = a["tag"], a["line"]
    head = b"\x00\x01\x00\x00" + struct.pack("<iiI", m.header.stamp.sec, m.header.stamp.nanosec, len(FRAME)) + FRAME  # 4 + 12 + 12 = 28
    head += struct.pack("<QIB3sI", timebase, len(a), 0, b"\0\0\0", len(a))  # timebase@24, point_num@32, lidar_id@36, rsvd@37, seq len@40
    return head + p.tobytes(), timebase

src, lidar, imu, dst = sys.argv[1], sys.argv[2], sys.argv[3], Path(sys.argv[4])
with AnyReader([Path(src)], default_typestore=ts) as r, Writer(dst, version=8, storage_plugin=StoragePlugin.MCAP) as w:
    cl = w.add_connection(lidar, "livox_ros_driver2/msg/CustomMsg", typestore=ts)
    ci = w.add_connection(imu, "sensor_msgs/msg/Imu", typestore=ts)
    first = True
    for c, t, raw in r.messages(connections=[x for x in r.connections if x.topic in (lidar, imu)]):
        if c.topic == lidar:
            m = r.deserialize(raw, c.msgtype)
            data, tb = custom_cdr(m)
            if first:
                chk = ts.deserialize_cdr(data, "livox_ros_driver2/msg/CustomMsg")
                assert chk.timebase == tb and chk.point_num == m.width and len(chk.points) == m.width, "CDR round-trip failed"
                assert abs(chk.points[-1].offset_time * 1e-6 - (np.frombuffer(m.data, IN)["ts"].max() - tb) * 1e-6) < 1e-3
                first = False
            w.write(cl, t, data)
        else:
            w.write(ci, t, raw)  # IMU: same CDR as input (both ROS 2 bags)
