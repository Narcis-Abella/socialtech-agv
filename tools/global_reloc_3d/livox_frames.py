"""Raw Livox frames (livox_ros_driver2/CustomMsg, xfer_format 1) into the cloud the relocalizer exposes, without deserializing the message.

rclpy takes ~80 ms to deserialize one 20 000-point frame (AGX), a core at 10 Hz. The message is plain CDR (little endian): 4-byte encapsulation header, stamp (i32, u32),
frame_id (u32 length incl. NUL + bytes), timebase u64 (8-aligned from the stream start), point_num u32, lidar_id u8, rsvd u8[3], points (u32 count, then 20-byte elements:
offset_time u32, x y z f32, reflectivity u8, tag u8, line u8, 1 pad byte; the last element has the pad byte or not, by serializer). Checked against rclpy's own serializer and a frame the C++ simulator published (test_livox_frames.py, in the image).

The filters are FAST-LIO2's preprocessing for lidar_type 1 (preprocess.cpp avia_handler): a point counts when its line < 4 and its tag says a normal return ((tag & 0x30) is 0x00 or 0x10), it is kept
every `decimate`-th (point_filter_num) and farther than `blind` from the sensor. The cloud is moved to FAST-LIO2's body frame (the IMU) by the lidar -> IMU extrinsic (extrinsic_T, extrinsic_R = I)."""
import struct

import numpy as np

POINT = np.dtype([('t', '<u4'), ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('refl', 'u1'), ('tag', 'u1'), ('line', 'u1'), ('pad', 'u1')])


def parse(raw, blind=.1, decimate=2, extrinsic=(-.011, -.02329, .04412)):
    """(stamp [s], valid points, cloud (m, 3) float64 in the body frame). `valid` counts the returns that pass the tag/line/blind test before decimating: what says the lidar sees."""
    b = bytes(raw)
    if b[:2] != b'\x00\x01':
        raise ValueError('not a little-endian CDR message')
    sec, nsec = struct.unpack_from('<iI', b, 4)
    L, = struct.unpack_from('<I', b, 12)
    off = 16 + L
    off += -(off - 4) % 8
    off += 8                                        # timebase
    off += 4 + 4                                    # point_num, lidar_id + rsvd
    n, = struct.unpack_from('<I', b, off)
    off += 4
    if len(b) - off - 20 * n not in (-1, 0):        # the last element's padding byte is there when the C++ serializer wrote the message (the driver, the simulator), not when rclpy's did
        raise ValueError(f'frame layout mismatch: {n} points from offset {off} do not end at byte {len(b)}')
    p = np.frombuffer(b + b'\x00', POINT, n, off)
    xyz = np.stack([p['x'], p['y'], p['z']], axis=1).astype(np.float64)
    tag = p['tag'] & 0x30
    ok = (p['line'] < 4) & ((tag == 0x00) | (tag == 0x10)) & (np.einsum('ij,ij->i', xyz, xyz) > blind * blind)
    xyz = xyz[ok]
    return sec + nsec * 1e-9, int(ok.sum()), xyz[::max(decimate, 1)] + np.asarray(extrinsic)


def imu_fields(b):
    """(stamp [s], gyro (3), acc (3)) of a sensor_msgs/Imu CDR message without deserializing it: after the frame_id the message is float64s (orientation 4 + covariance 9, angular_velocity 3 +
    covariance 9, linear_acceleration 3 + covariance 9), 8-aligned from the stream start."""
    if b[:2] != b'\x00\x01':
        raise ValueError('not a little-endian CDR message')
    sec, nsec, L = struct.unpack_from('<iII', b, 4)
    off = 16 + L
    off += -(off - 4) % 8
    if len(b) < off + 8 * (4 + 9 + 3 + 9 + 3 + 9):
        raise ValueError('short Imu message')
    return sec + nsec * 1e-9, struct.unpack_from('<3d', b, off + 104), struct.unpack_from('<3d', b, off + 200)
