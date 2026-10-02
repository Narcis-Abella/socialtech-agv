"""Checks the C++ planar_odom node (docker/ros/planar_odom) against planar_odom.footprint, the Python reference. Needs ROS 2 and the built package
(run inside the robot image: python3 test_planar_odom_node.py); skips without rclpy. Run the unit tests of the reference with test_planar_odom.py."""
import subprocess
import sys
import time

import numpy as np

import planar_odom as po

H = 0.773
LEVEL = list(np.array([0.2, -0.3, 0.1, 0.92]) / np.linalg.norm([0.2, -0.3, 0.1, 0.92]))


def test_node_matches_the_python_reference_and_publishes_the_level_transform():
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from tf2_msgs.msg import TFMessage

    node_proc = subprocess.Popen(["ros2", "run", "planar_odom", "planar_odom", "--ros-args", "-p", f"sensor_height:={H}", "-p", f"level:={LEVEL}"],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        rclpy.init()
        n = Node("planar_odom_test")
        got, static = {}, []
        n.create_subscription(TFMessage, "/tf", lambda m: [got.__setitem__(t.header.stamp.nanosec, t) for t in m.transforms if t.child_frame_id == "base_footprint"], 1000)
        n.create_subscription(TFMessage, "/tf_static", lambda m: static.extend(m.transforms), QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        pub = n.create_publisher(Odometry, "/Odometry", 100)
        t_end = time.time() + 6  # node start-up
        while time.time() < t_end:
            rclpy.spin_once(n, timeout_sec=0.1)
        rng, sent = np.random.default_rng(0), []
        for i in range(1, 301):
            m = Odometry()
            m.header.stamp.nanosec = i
            p, q = rng.normal(size=3) * 5, rng.normal(size=4)
            q /= np.linalg.norm(q)
            m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = p
            m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w = q
            pub.publish(m)
            sent.append((i, p, q))
            rclpy.spin_once(n, timeout_sec=0.005)
        for _ in range(50):
            rclpy.spin_once(n, timeout_sec=0.05)
        assert all(i in got for i, _, _ in sent), f"{sum(i not in got for i, _, _ in sent)} of 300 TF missing"
        for i, p, q in sent:
            x, y, z, sz, cw = po.footprint(p, q, LEVEL, H)
            r = got[i].transform
            assert max(abs(r.translation.x - x), abs(r.translation.y - y), abs(r.translation.z - z), abs(r.rotation.z - sz), abs(r.rotation.w - cw)) < 1e-9
            assert r.rotation.x == 0 and r.rotation.y == 0  # planar: yaw only
        s = [t for t in static if (t.header.frame_id, t.child_frame_id) == ("odom", "camera_init")]
        assert s, "no static odom -> camera_init"
        r = s[0].transform.rotation
        assert max(abs(r.x - LEVEL[0]), abs(r.y - LEVEL[1]), abs(r.z - LEVEL[2]), abs(r.w - LEVEL[3])) < 1e-12
    finally:
        node_proc.terminate()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        import rclpy  # noqa: F401
    except ImportError:
        print("skip: no rclpy (run inside the robot image)")
        sys.exit(0)
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
