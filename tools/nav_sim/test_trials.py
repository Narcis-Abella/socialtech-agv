"""Run (with ROS sourced): python3 -B test_trials.py"""
import math

import trials as t


def test_yaw_quaternion_roundtrip():
    for yaw in (-3.0, -1.5708, 0.0, 0.607, 2.828, 3.1):
        z, w = t.yaw_to_quat(yaw)
        assert abs(t.quat_to_yaw(z, w) - yaw) < 1e-9
        assert abs(math.hypot(z, w) - 1.0) < 1e-12


if __name__ == "__main__":
    test_yaw_quaternion_roundtrip()
    print("ok")
