"""live_paths.spin: a pose that arrives while the context is shutting down must not kill the node with exit code 1 (seen on the Nano at every stop), a real error must still raise.
Needs ROS 2 (run inside the robot image): python3 test_live_paths.py"""
import os, sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "scripts"))
import rclpy
import live_paths


def boom(_node):
    raise RuntimeError("Failed to publish: publisher's context is invalid")


def test_an_error_after_shutdown_is_not_an_error():
    rclpy.init()
    rclpy.shutdown()                      # the launch stopped: the context is gone
    real_spin, rclpy.spin = rclpy.spin, boom
    try:
        live_paths.spin(object())         # must return
    finally:
        rclpy.spin = real_spin


def test_an_error_with_a_live_context_still_raises():
    rclpy.init()
    real_spin, rclpy.spin = rclpy.spin, boom
    try:
        try:
            live_paths.spin(object())
        except RuntimeError:
            return
        raise AssertionError("a real error was swallowed")
    finally:
        rclpy.spin = real_spin
        rclpy.shutdown()


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
