"""Reader for livox_ros_driver2/msg/CustomMsg in a bag without the Livox message package (decodes the CDR by hand). usage: livox_custom.py BAG_DIR [T0 T1] | livox_custom.py --selftest
frames(bag, t0, t1) -> generator of (t_s from the start of the bag, header stamp s, structured array of the points: t (offset_time ns), x, y, z (m), r, tag, line).
Layout (CDR little endian, alignment relative to the byte after the 4-byte encapsulation header): Header(stamp int32+uint32, frame_id string), uint64 timebase, uint32 point_num, uint8 lidar_id,
uint8[3] rsvd, then the sequence: uint32 length and elements {uint32 offset_time, float x, y, z, uint8 reflectivity, tag, line} of 20 bytes each (19 + 1 of padding before the next uint32)."""
import struct
import sys

import numpy as np

PT = np.dtype([("t", "<u4"), ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("r", "u1"), ("tag", "u1"), ("line", "u1"), ("pad", "u1")])


def decode(raw):
    """-> (stamp s, timebase ns, points) or raises ValueError when the layout does not hold"""
    b = memoryview(raw)[4:]
    if raw[1] != 1: raise ValueError("not little endian CDR")
    sec, nsec, slen = struct.unpack_from("<iII", b, 0); p = 12 + slen; p = (p + 7) & ~7
    timebase, npts = struct.unpack_from("<QI", b, p); p += 12 + 4   # + lidar_id and rsvd[3]
    p = (p + 3) & ~3; (n,) = struct.unpack_from("<I", b, p); p += 4
    if n != npts or p + n * PT.itemsize > len(b) + 1: raise ValueError(f"layout mismatch: point_num {npts}, sequence {n}, bytes {len(b)}")
    return sec + nsec * 1e-9, timebase, np.frombuffer(b, dtype=PT, count=n, offset=p).copy()


def frames(bag, t0=0.0, t1=1e9):
    import rosbag2_py
    s0 = float(open(bag + "/metadata.yaml").read().split("nanoseconds_since_epoch:")[1].split()[0]) / 1e9
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("", "")); r.set_filter(rosbag2_py.StorageFilter(topics=["/livox/lidar"])); r.seek(int((s0 + t0) * 1e9))
    while r.has_next():
        _, raw, tn = r.read_next(); rt = tn / 1e9 - s0
        if rt > t1: break
        stamp, tb, pts = decode(raw); yield rt, stamp - s0, pts


def selftest():
    hdr = b"\x00\x01\x00\x00"; fid = b"livox_frame\x00"; body = struct.pack("<iII", 5, 6, len(fid)) + fid; body += b"\x00" * (((len(body) + 7) & ~7) - len(body)); body += struct.pack("<QI", 123, 2) + b"\x01\x00\x00\x00" + struct.pack("<I", 2)
    for i in range(2): body += struct.pack("<IfffBBB", 1000 * i, 1.0 + i, 2.0, 3.0, 9, 1, i) + b"\x00"
    stamp, tb, pts = decode(hdr + body); assert abs(stamp - 5.000000006) < 1e-9 and tb == 123 and len(pts) == 2 and pts["x"][1] == 2.0 and pts["line"][1] == 1 and pts["t"][1] == 1000, (stamp, tb, pts)
    print("selftest ok")


if __name__ == "__main__":
    if sys.argv[1] == "--selftest": selftest(); sys.exit()
    n = 0
    for rt, st, pts in frames(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 0, float(sys.argv[3]) if len(sys.argv) > 3 else 2):
        r = np.hypot(pts["x"], pts["y"]); print(f"t={rt:8.3f} stamp={st:8.3f} points {len(pts):6d}  z {pts['z'].min():+.2f}..{pts['z'].max():+.2f}  r {np.hypot(r, pts['z']).min():.2f}..{np.hypot(r, pts['z']).max():.1f}  offset_time max {pts['t'].max()/1e6:.1f} ms  tag set {np.unique(pts['tag'])[:4].tolist()}"); n += 1
        if n >= 6: break
