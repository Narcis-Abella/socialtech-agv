"""The real level_corrector node on a synthetic room: the body is physically tilted (roll 1.0 deg, pitch -0.7 deg) while /Odometry says it is level; the node must publish the
roll/pitch correction that puts the floor normal back on z, with a status that says ok. Run where the package is built: python3 test_level_corrector_node.py (own domain, localhost only)."""
import math
import os
import struct
import subprocess
import sys
import tempfile
import time

os.environ["ROS_DOMAIN_ID"] = "91"   # fake TF, /Odometry and clouds: never on the domain of a running robot
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
import numpy as np

rng = np.random.default_rng(3)


def room(density):
    def plane(o, a, b):
        o, a, b = map(np.array, (o, a, b)); n = int(density * np.linalg.norm(a) * np.linalg.norm(b)) or 1
        s, t = rng.random((n, 1)), rng.random((n, 1)); return o + s * a + t * b
    P = [plane([-5, -4, 0], [10, 0, 0], [0, 8, 0]), plane([-5, -4, 0], [10, 0, 0], [0, 0, 2.5]), plane([-5, 4, 0], [10, 0, 0], [0, 0, 2.5]),
         plane([-5, -4, 0], [0, 8, 0], [0, 0, 2.5]), plane([5, -4, 0], [0, 8, 0], [0, 0, 2.5]),
         plane([1, 1, 0], [1, 0, 0], [0, 0, 1.2]), plane([1, 1, 0], [0, 1, 0], [0, 0, 1.2]), plane([-3, -2, 0], [0.8, 0, 0], [0, 0, 0.9]), plane([-3, -2, 0], [0, 0.8, 0], [0, 0, 0.9])]
    return np.concatenate(P)


def rot(roll_deg, pitch_deg, yaw_deg):
    r, p, y = np.radians([roll_deg, pitch_deg, yaw_deg])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    return Rz @ Ry @ Rx


def quat(R):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()   # x y z w


def write_map(d, pts):
    hdr = f"ply\nformat binary_little_endian 1.0\nelement vertex {len(pts)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n"
    with open(d + "/map.ply", "wb") as f:
        f.write(hdr.encode()); f.write(pts.astype("<f4").tobytes())
    with open(d + "/map.yaml", "w") as f:
        f.write("image: map.pgm\n# T_map_world (row-major; p_map = T p_ply):\n# [1.0, 0.0, 0.0, 0.0]\n# [0.0, 1.0, 0.0, 0.0]\n# [0.0, 0.0, 1.0, 0.0]\n# [0.0, 0.0, 0.0, 1.0]\n")


def main():
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import QuaternionStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster

    d = tempfile.mkdtemp(); write_map(d, room(400))
    binary = os.path.join(get_package_prefix("level_corrector"), "lib", "level_corrector", "level_corrector_node")
    proc = subprocess.Popen([binary, "--ros-args", "-p", f"map_dir:={d}", "-p", "rate_hz:=2.0", "-p", "level:=[0.0,0.0,0.0,1.0]"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    rclpy.init(); n = rclpy.create_node("lc_test"); got = {"C": None, "status": None}
    n.create_subscription(QuaternionStamped, "/level_corrector/correction", lambda m: got.__setitem__("C", m), 10)
    n.create_subscription(String, "/level_corrector/status", lambda m: got.__setitem__("status", m.data), 10)
    odom_pub, cloud_pub, br = n.create_publisher(Odometry, "/Odometry", 10), n.create_publisher(PointCloud2, "/cloud_registered_body", 10), TransformBroadcaster(n)

    R_true = rot(1.0, -0.7, 20.0); t_true = np.array([-1.0, 0.5, 0.5])
    R_chain = rot(0, 0, 20.5); t_chain = np.array([-1.02, 0.47, 0.5])   # what FAST-LIO says: level, planar pose a little off
    world = room(250); body = (world - t_true) @ R_true; r = np.linalg.norm(body, axis=1); body = body[(r > 0.8) & (r < 8)].astype(np.float32)
    fields = [PointField(name=k, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, k in enumerate("xyz")]
    deadline = time.time() + 40
    try:
        while (got["C"] is None or got["status"] is None) and time.time() < deadline:   # the status follows the correction by an instant
            now = n.get_clock().now().to_msg()
            tf = TransformStamped(); tf.header.stamp = now; tf.header.frame_id = "map"; tf.child_frame_id = "odom"; tf.transform.rotation.w = 1.0; br.sendTransform(tf)
            o = Odometry(); o.header.stamp = now; o.header.frame_id = "camera_init"; o.child_frame_id = "body"
            o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = t_chain
            q = quat(R_chain); o.pose.pose.orientation.x, o.pose.pose.orientation.y, o.pose.pose.orientation.z, o.pose.pose.orientation.w = q
            odom_pub.publish(o)
            c = PointCloud2(); c.header.stamp = now; c.header.frame_id = "body"; c.height = 1; c.width = len(body); c.fields = fields; c.point_step = 12; c.row_step = 12 * len(body)
            c.is_dense = True; c.data = body.tobytes(); cloud_pub.publish(c)
            rclpy.spin_once(n, timeout_sec=0.1)
    finally:
        proc.terminate()
        out = proc.communicate(timeout=10)[0]
    assert got["C"] is not None, f"no correction published; status={got['status']}\nnode log:\n{out}"
    assert got["status"] and got["status"].startswith("state=ok"), got["status"]
    q = got["C"].quaternion; from scipy.spatial.transform import Rotation
    C = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    ez = np.array([0, 0, 1.0]); up_true_in_body = R_true.T @ ez                      # the map's up in the body, what the registration has to find
    after = C @ R_chain @ up_true_in_body                                             # corrected chain: the floor normal must come back to z
    err = math.degrees(math.acos(min(1.0, float(after @ ez)))); before = math.degrees(math.acos(min(1.0, float(R_chain @ up_true_in_body @ ez))))
    print(f"floor tilt in odom: {before:.3f} deg before, {err:.3f} deg after the correction; {got['status']}")
    assert before > 1.0 and err < 0.2, (before, err)
    st = dict(kv.split("=") for kv in got["status"].split()); u = C @ ez   # the status reports C as pitch/roll of its z axis; a garbled number there once came from a dangling Eigen expression
    assert abs(float(st["C_pitch"]) - math.degrees(math.atan2(u[0], u[2]))) < 0.01 and abs(float(st["C_roll"]) - math.degrees(math.atan2(u[1], u[2]))) < 0.01, (got["status"], u)
    assert proc.returncode in (0, -15), f"node exit code {proc.returncode} (a crash at shutdown, like FAST-LIO's -11)"
    print("OK")


if __name__ == "__main__":
    try:
        import rclpy  # noqa: F401
    except ImportError:
        print("skip: no rclpy"); sys.exit(0)
    main()
