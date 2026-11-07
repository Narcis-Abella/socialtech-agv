"""Checks the C++ planar_odom node (docker/ros/planar_odom) against planar_odom.footprint, the Python reference. Needs ROS 2 and the built package
(run inside the robot image: python3 test_planar_odom_node.py); skips without rclpy. Run the unit tests of the reference with test_planar_odom.py."""
import subprocess
import sys
import time

import numpy as np

import planar_odom as po

H = 0.773
LEVEL = list(np.array([0.2, -0.3, 0.1, 0.92]) / np.linalg.norm([0.2, -0.3, 0.1, 0.92]))
LIDAR = [-0.011, -0.02329, 0.04412]


def binary():
    """The executable itself: `ros2 run` is a wrapper, a signal to it leaves the node running (and the next test would hear two publishers)."""
    import os
    from ament_index_python.packages import get_package_prefix
    return os.path.join(get_package_prefix("planar_odom"), "lib", "planar_odom", "planar_odom")


def test_node_matches_the_python_reference_and_publishes_the_level_transform():
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from tf2_msgs.msg import TFMessage

    node_proc = subprocess.Popen([binary(), "--ros-args", "-p", f"sensor_height:={H}", "-p", f"level:={LEVEL}", "-p", f"lidar_in_body:={LIDAR}"],
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
            x, y, z, sz, cw = po.footprint(p, q, LEVEL, H, LIDAR[:2])
            r = got[i].transform
            assert max(abs(r.translation.x - x), abs(r.translation.y - y), abs(r.translation.z - z), abs(r.rotation.z - sz), abs(r.rotation.w - cw)) < 1e-9
            assert r.rotation.x == 0 and r.rotation.y == 0  # planar: yaw only
        s = [t for t in static if (t.header.frame_id, t.child_frame_id) == ("odom", "camera_init")]
        assert s, "no static odom -> camera_init"
        r = s[0].transform.rotation
        assert max(abs(r.x - LEVEL[0]), abs(r.y - LEVEL[1]), abs(r.z - LEVEL[2]), abs(r.w - LEVEL[3])) < 1e-12
        s = [t for t in static if (t.header.frame_id, t.child_frame_id) == ("base_footprint", "lidar_level")]
        assert s, "no static base_footprint -> lidar_level"
        v, r = s[0].transform.translation, s[0].transform.rotation
        assert (v.x, v.y) == (0, 0) and abs(v.z - (H + LIDAR[2])) < 1e-12 and (r.x, r.y, r.z, r.w) == (0, 0, 0, 1)  # levelled, straight above the base
    finally:
        node_proc.terminate()
        rclpy.shutdown()


def test_with_a_correction_topic_camera_init_is_dynamic_and_both_transforms_use_c_times_the_level():
    import rclpy
    from geometry_msgs.msg import QuaternionStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from tf2_msgs.msg import TFMessage

    tilt = np.radians(0.7)
    c = np.array([np.sin(tilt / 2), 0.0, 0.0, np.cos(tilt / 2)])  # a roll of 0.7 deg
    corrected = po.corrected_level(LEVEL, c)
    node_proc = subprocess.Popen([binary(), "--ros-args", "-p", f"sensor_height:={H}", "-p", f"level:={LEVEL}", "-p", f"lidar_in_body:={LIDAR}", "-p", "correction_topic:=/level_corrector/correction"],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        rclpy.init()
        n = Node("planar_odom_test_c")
        foot, cam, static = {}, {}, []
        def on_tf(m):
            for t in m.transforms:
                if t.child_frame_id == "base_footprint":
                    foot[t.header.stamp.nanosec] = t
                elif t.child_frame_id == "camera_init":
                    cam[t.header.stamp.nanosec] = t
        n.create_subscription(TFMessage, "/tf", on_tf, 1000)
        n.create_subscription(TFMessage, "/tf_static", lambda m: static.extend(m.transforms), QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        pub_c = n.create_publisher(QuaternionStamped, "/level_corrector/correction", 10)
        pub = n.create_publisher(Odometry, "/Odometry", 100)
        t_end = time.time() + 6  # node start-up; C is repeated like the corrector does every cycle
        while time.time() < t_end:
            q = QuaternionStamped(); q.header.frame_id = "odom"
            q.quaternion.x, q.quaternion.y, q.quaternion.z, q.quaternion.w = c
            pub_c.publish(q)
            rclpy.spin_once(n, timeout_sec=0.1)
        rng, sent = np.random.default_rng(1), []
        for i in range(1, 101):
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
        assert all(i in foot and i in cam for i, _, _ in sent), f"TF missing: {len(foot)} base_footprint, {len(cam)} camera_init of {len(sent)}"
        assert not [t for t in static if t.child_frame_id == "camera_init"], "camera_init must not be static when it is dynamic (two publishers of one child)"
        for i, p, q in sent:
            x, y, z, sz, cw = po.footprint(p, q, corrected, H, LIDAR[:2])
            r = foot[i].transform
            assert max(abs(r.translation.x - x), abs(r.translation.y - y), abs(r.translation.z - z), abs(r.rotation.z - sz), abs(r.rotation.w - cw)) < 1e-9
            k = cam[i].transform.rotation
            got = np.array([k.x, k.y, k.z, k.w])
            assert abs(abs(got @ corrected) - 1) < 1e-9 and cam[i].header.frame_id == "odom"
        # a wild correction (30 deg, a broken corrector) is ignored: the last good one stays
        bad = QuaternionStamped(); bad.quaternion.x, bad.quaternion.w = float(np.sin(np.radians(15))), float(np.cos(np.radians(15)))
        for _ in range(5):
            pub_c.publish(bad)
            rclpy.spin_once(n, timeout_sec=0.05)
        m = Odometry(); m.header.stamp.nanosec = 500; m.pose.pose.orientation.w = 1.0
        pub.publish(m)
        for _ in range(20):
            rclpy.spin_once(n, timeout_sec=0.05)
        k = cam[500].transform.rotation
        assert abs(abs(np.array([k.x, k.y, k.z, k.w]) @ corrected) - 1) < 1e-9, "a 30 deg correction was applied"
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
