"""M3DGR ROS1 bag -> ROS2 mcap with Mid-360 as PointCloud2 (x y z intensity tag line timestamp[f64 abs ns]) + IMU.
usage: m3dgr_convert.py <in.bag> <out_dir>"""
import struct, sys
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore

LIDAR, IMU = "/livox/mid360/lidar", "/livox/mid360/imu"
ts = get_typestore(Stores.ROS2_JAZZY)
T = ts.types
PT = np.dtype([("t", "<u4"), ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "u1"), ("tag", "u1"), ("line", "u1")])
OUT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"), ("tag", "u1"), ("line", "u1"), ("timestamp", "<f8")])
assert PT.itemsize == 19 and OUT.itemsize == 26
FIELDS = [T["sensor_msgs/msg/PointField"](name=n, offset=OUT.fields[n][1], datatype=d, count=1)
          for n, d in (("x", 7), ("y", 7), ("z", 7), ("intensity", 7), ("tag", 2), ("line", 2), ("timestamp", 8))]

def stamp(sec, nsec):
    return T["builtin_interfaces/msg/Time"](sec=sec, nanosec=nsec)

def cloud(raw):  # ROS1 CustomMsg bytes -> PointCloud2
    sec, nsec = struct.unpack_from("<II", raw, 4)
    fl = struct.unpack_from("<I", raw, 12)[0]
    o = 16 + fl
    timebase, n = struct.unpack_from("<QI", raw, o)
    o += 8 + 4 + 1 + 3  # timebase, point_num, lidar_id, rsvd[3]
    assert struct.unpack_from("<I", raw, o)[0] == n and len(raw) - o - 4 == n * 19, "unexpected CustomMsg layout"
    p = np.frombuffer(raw, PT, n, o + 4)
    out = np.empty(n, OUT)
    for k, s in (("x", "x"), ("y", "y"), ("z", "z"), ("intensity", "i"), ("tag", "tag"), ("line", "line")):
        out[k] = p[s]
    out["timestamp"] = timebase + p["t"].astype(np.float64)  # ns; f64 keeps ~0.25 us at 1.7e18, same as the driver
    hdr = T["std_msgs/msg/Header"](stamp=stamp(sec, nsec), frame_id="livox_frame")
    return T["sensor_msgs/msg/PointCloud2"](header=hdr, height=1, width=n, fields=FIELDS, is_bigendian=False,
        point_step=26, row_step=26 * n, data=np.frombuffer(out.tobytes(), np.uint8), is_dense=True)

def imu(m):
    v3 = lambda a: T["geometry_msgs/msg/Vector3"](x=a.x, y=a.y, z=a.z)
    q = m.orientation
    hdr = T["std_msgs/msg/Header"](stamp=stamp(m.header.stamp.sec, m.header.stamp.nanosec), frame_id="livox_frame")
    return T["sensor_msgs/msg/Imu"](header=hdr,
        orientation=T["geometry_msgs/msg/Quaternion"](x=q.x, y=q.y, z=q.z, w=q.w),
        orientation_covariance=np.asarray(m.orientation_covariance, np.float64),
        angular_velocity=v3(m.angular_velocity), angular_velocity_covariance=np.asarray(m.angular_velocity_covariance, np.float64),
        linear_acceleration=v3(m.linear_acceleration), linear_acceleration_covariance=np.asarray(m.linear_acceleration_covariance, np.float64))

src, dst = Path(sys.argv[1]), Path(sys.argv[2])
with AnyReader([src]) as r, Writer(dst, version=8, storage_plugin=StoragePlugin.MCAP) as w:
    cl = w.add_connection(LIDAR, "sensor_msgs/msg/PointCloud2", typestore=ts)
    ci = w.add_connection(IMU, "sensor_msgs/msg/Imu", typestore=ts)
    for c, t, raw in r.messages(connections=[x for x in r.connections if x.topic in (LIDAR, IMU)]):
        if c.topic == LIDAR:
            w.write(cl, t, ts.serialize_cdr(cloud(raw), "sensor_msgs/msg/PointCloud2"))
        else:
            w.write(ci, t, ts.serialize_cdr(imu(r.deserialize(raw, c.msgtype)), "sensor_msgs/msg/Imu"))
