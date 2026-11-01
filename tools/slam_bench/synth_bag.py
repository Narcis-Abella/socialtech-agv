"""Test-only: write a synthetic CustomMsg / PointCloud2 bag with rosbags (None when rosbags is not installed)."""
from pathlib import Path

import numpy as np

try:
    from rosbags.rosbag2 import StoragePlugin, Writer
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
except ImportError:
    Writer = None

LIVOX = {"livox_ros_driver2/msg/CustomPoint": "uint32 offset_time\nfloat32 x\nfloat32 y\nfloat32 z\nuint8 reflectivity\nuint8 tag\nuint8 line\n",
         "livox_ros_driver2/msg/CustomMsg": "std_msgs/Header header\nuint64 timebase\nuint32 point_num\nuint8 lidar_id\nuint8[3] rsvd\nCustomPoint[] points\n"}


def livox_typestore():
    ts = get_typestore(Stores.ROS2_JAZZY)
    for name, text in LIVOX.items():
        ts.register(get_types_from_msg(text, name))
    return ts


def custom_msg(ts, xyz, stamp=(12, 340_000_000), frame="livox_frame"):
    T, H = ts.types["builtin_interfaces/msg/Time"], ts.types["std_msgs/msg/Header"]
    P = ts.types["livox_ros_driver2/msg/CustomPoint"]
    pts = [P(offset_time=i * 1000, x=float(x), y=float(y), z=float(z), reflectivity=10, tag=0, line=1) for i, (x, y, z) in enumerate(xyz)]
    return ts.types["livox_ros_driver2/msg/CustomMsg"](header=H(stamp=T(sec=stamp[0], nanosec=stamp[1]), frame_id=frame), timebase=7, point_num=len(pts), lidar_id=0, rsvd=np.zeros(3, np.uint8), points=pts)


def point_cloud2(ts, xyz, stamp=(12, 340_000_000)):
    T, H, F = ts.types["builtin_interfaces/msg/Time"], ts.types["std_msgs/msg/Header"], ts.types["sensor_msgs/msg/PointField"]
    data = np.asarray(xyz, "<f4").tobytes()
    return ts.types["sensor_msgs/msg/PointCloud2"](header=H(stamp=T(sec=stamp[0], nanosec=stamp[1]), frame_id="livox_frame"), height=1, width=len(xyz),
                                                   fields=[F(name=n, offset=4 * i, datatype=7, count=1) for i, n in enumerate("xyz")], is_bigendian=False, point_step=12,
                                                   row_step=12 * len(xyz), data=np.frombuffer(data, np.uint8), is_dense=True)


def write_bag(bag, scans_xyz, kind, t0=1.0, dt=0.1, topic="/livox/lidar"):
    """scans_xyz: list of (n, 3) sensor-frame clouds, written at t0 + k * dt as `kind` = "custom" (CustomMsg) or "pc2" (PointCloud2)."""
    ts = livox_typestore()
    build, msgtype = (custom_msg, "livox_ros_driver2/msg/CustomMsg") if kind == "custom" else (point_cloud2, "sensor_msgs/msg/PointCloud2")
    with Writer(Path(bag), version=8, storage_plugin=StoragePlugin.MCAP) as w:
        conn = w.add_connection(topic, msgtype, typestore=ts)
        for k, xyz in enumerate(scans_xyz):
            sec, nsec = divmod(round((t0 + k * dt) * 1e9), 10**9)
            w.write(conn, round((t0 + k * dt) * 1e9), ts.serialize_cdr(build(ts, np.asarray(xyz, np.float32), stamp=(int(sec), int(nsec))), msgtype))
