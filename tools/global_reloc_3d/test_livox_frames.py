import struct
import unittest

import numpy as np

import livox_frames as lf


def cdr(points, frame_id=b'livox_frame', sec=1793574000, nsec=123456789, pad_last=False):
    """A CustomMsg as the CDR the middleware gives: built the way the layout is documented in livox_frames.py."""
    b = bytearray(b'\x00\x01\x00\x00') + struct.pack('<iI', sec, nsec) + struct.pack('<I', len(frame_id) + 1) + frame_id + b'\x00'
    b += b'\x00' * (-(len(b) - 4) % 8)
    b += struct.pack('<QIB3x', sec * 10**9 + nsec, len(points), 0) + struct.pack('<I', len(points))
    for t, x, y, z, refl, tag, line in points:
        b += struct.pack('<I3fBBB', t, x, y, z, refl, tag, line) + b'\x00'
    return bytes(b if pad_last else b[:-1])                # rclpy's serializer leaves no padding after the last element, the C++ one (driver, simulator) does


class ParseTest(unittest.TestCase):
    def test_stamp_count_and_body_frame(self):
        pts = [(i * 5000, float(i + 1), 0., 1., 100, 0, i % 4) for i in range(10)]
        stamp, valid, xyz = lf.parse(cdr(pts), decimate=1, extrinsic=(1., 2., 3.))
        self.assertAlmostEqual(stamp, 1793574000.123456789, 3)
        self.assertEqual(valid, 10)
        np.testing.assert_allclose(xyz[0], [2., 2., 4.])

    def test_fast_lio_filters(self):
        pts = [(0, 5., 0., 0., 1, 0, 0),         # kept
               (0, 0., 0., 0., 0, 0, 0),         # no return (zero): inside the blind distance
               (0, .05, 0., 0., 1, 0, 0),        # nearer than blind
               (0, 6., 0., 0., 1, 0x20, 0),      # tag: noise
               (0, 7., 0., 0., 1, 0x10, 1),      # tag 0x10: kept
               (0, 8., 0., 0., 1, 0x30, 2),      # tag 0x30: dropped
               (0, 9., 0., 0., 1, 0, 4)]         # line >= 4: dropped
        _, valid, xyz = lf.parse(cdr(pts), blind=.1, decimate=1, extrinsic=(0., 0., 0.))
        self.assertEqual(valid, 2)
        self.assertEqual(xyz[:, 0].tolist(), [5., 7.])

    def test_decimation_keeps_every_nth_valid_point(self):
        pts = [(0, float(i + 1), 0., 0., 1, 0, 0) for i in range(10)]
        _, valid, xyz = lf.parse(cdr(pts), decimate=2, extrinsic=(0., 0., 0.))
        self.assertEqual(valid, 10)
        self.assertEqual(xyz[:, 0].tolist(), [1., 3., 5., 7., 9.])

    def test_the_padding_byte_after_the_last_element_is_optional(self):
        pts = [(0, 3. + i, 0., 0., 1, 0, 0) for i in range(7)]
        for pad in (False, True):
            self.assertEqual(lf.parse(cdr(pts, pad_last=pad), decimate=1)[1], 7, pad)

    def test_a_frame_id_of_any_length_keeps_the_alignment(self):
        pts = [(0, 3., 0., 0., 1, 0, 0)]
        for fid in (b'a', b'abcd', b'livox_frame', b'a_longer_frame_id_x'):
            self.assertEqual(lf.parse(cdr(pts, fid), decimate=1)[1], 1, fid)

    def test_a_truncated_or_foreign_message_is_refused(self):
        raw = cdr([(0, 3., 0., 0., 1, 0, 0)] * 5)
        with self.assertRaises(ValueError):
            lf.parse(raw[:-7])
        with self.assertRaises(ValueError):
            lf.parse(b'\x00\x00' + raw[2:])

    def test_fast_enough_for_the_node(self):
        import time
        raw = cdr([(i, 1. + i % 7, 0.5, 1., 100, 0, i % 4) for i in range(19968)])
        t = time.monotonic()
        for _ in range(10):
            lf.parse(raw)
        self.assertLess((time.monotonic() - t) / 10, .02)   # the message deserializer takes 80 ms on the AGX; this is the laptop, 20 ms is generous




class RclpyTest(unittest.TestCase):
    """The layout against rclpy's own serializer (inside the robot image only)."""
    def test_same_points_as_the_message(self):
        try:
            from livox_ros_driver2.msg import CustomMsg, CustomPoint
            from rclpy.serialization import serialize_message
        except ImportError:
            self.skipTest('needs the robot image (rclpy, livox_ros_driver2)')
        rng = np.random.default_rng(1)
        for fid, n in (('livox_frame', 1000), ('x', 3), ('livox_mid360_link', 777)):
            m = CustomMsg(); m.header.frame_id = fid; m.header.stamp.sec = 1793574000; m.header.stamp.nanosec = 5; m.point_num = n
            for i in range(n):
                q = CustomPoint(); q.offset_time = i * 5000; q.x, q.y, q.z = (float(v) for v in rng.normal(0, 4, 3)); q.reflectivity = int(rng.integers(0, 255)); q.tag = int(rng.choice([0, 16, 32, 48])); q.line = i % 4
                m.points.append(q)
            stamp, valid, xyz = lf.parse(serialize_message(m), blind=.1, decimate=1, extrinsic=(0., 0., 0.))
            keep = [q for q in m.points if q.line < 4 and (q.tag & 0x30) in (0, 0x10) and q.x * q.x + q.y * q.y + q.z * q.z > .01]
            self.assertAlmostEqual(stamp, 1793574000 + 5e-9, 6)
            self.assertEqual(valid, len(keep))
            np.testing.assert_allclose(xyz, np.array([[q.x, q.y, q.z] for q in keep]), atol=1e-6)


if __name__ == '__main__':
    unittest.main()
