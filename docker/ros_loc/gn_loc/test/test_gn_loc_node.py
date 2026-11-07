"""The real gn_loc node on a synthetic room: map->odom carries the z that puts base_footprint on the floor of the map, gn_loc/pose comes out, and a scan frame straight above the base raises no warning while one
off to the side does. Run where gn_loc is built: python3 test_gn_loc_node.py (on its own domain); skips without rclpy."""
import math
import os
import subprocess
import sys
import time

os.environ["ROS_DOMAIN_ID"] = "90"   # fake /map, /initialpose and TF: never on the domain of a running robot
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
ODOM_Z = -0.64
HALF = 5.0


def run(scan_frame, scan_x):
    """The node's log and the map->odom z and pose it published with /scan in scan_frame, scan_x ahead of and 0.57 m above the base."""
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    from tf2_ros import TransformBroadcaster

    # the binary itself: `ros2 run` is a wrapper, a signal to it leaves the node running
    binary = os.path.join(get_package_prefix("gn_loc"), "lib", "gn_loc", "gn_loc_node")
    proc = subprocess.Popen([binary, "--ros-args", "-p", "bond_heartbeat_period:=0.0"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    n = rclpy.create_node("gn_loc_test")
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    got = {"z": None, "pose": None}
    n.create_subscription(TFMessage, "/tf", lambda m: [got.__setitem__("z", t.transform.translation.z) for t in m.transforms if (t.header.frame_id, t.child_frame_id) == ("map", "odom")], 100)
    n.create_subscription(PoseWithCovarianceStamped, "/gn_loc/pose", lambda m: got.__setitem__("pose", m), latched)
    map_pub, init_pub = n.create_publisher(OccupancyGrid, "/map", latched), n.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    scan_pub, br = n.create_publisher(LaserScan, "/scan", 10), TransformBroadcaster(n)

    res, size = 0.05, int(2 * (HALF + 1) / 0.05)
    grid = OccupancyGrid()
    grid.header.frame_id = "map"; grid.info.resolution = res; grid.info.width = grid.info.height = size
    grid.info.origin.position.x = grid.info.origin.position.y = -(HALF + 1)
    cell = [0] * (size * size)
    for j in range(size):
        for i in range(size):
            x, y = -(HALF + 1) + (i + .5) * res, -(HALF + 1) + (j + .5) * res
            if abs(max(abs(x), abs(y)) - HALF) < res:
                cell[j * size + i] = 100
    grid.data = cell

    def tf(stamp, parent, child, x, z):
        t = TransformStamped(); t.header.stamp = stamp; t.header.frame_id = parent; t.child_frame_id = child
        t.transform.translation.x = x; t.transform.translation.z = z; t.transform.rotation.w = 1.0
        return t

    def tick():
        st = n.get_clock().now().to_msg()
        br.sendTransform([tf(st, "odom", "base_footprint", 0.0, ODOM_Z), tf(st, "base_footprint", scan_frame, scan_x, 0.57)])
        s = LaserScan(); s.header.stamp = st; s.header.frame_id = scan_frame; s.angle_min = -math.pi; s.angle_increment = math.radians(1.0); s.angle_max = math.pi - s.angle_increment; s.range_min = 0.1; s.range_max = 30.0
        s.ranges = [float(HALF / max(abs(math.cos(a)), abs(math.sin(a)))) for a in (s.angle_min + k * s.angle_increment for k in range(360))]
        scan_pub.publish(s)
        t_end = time.time() + .1   # 10 Hz, the rate of the real scan
        while time.time() < t_end:
            rclpy.spin_once(n, timeout_sec=.01)

    def spin(sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            tick()

    try:
        spin(3)
        for transition in ("configure", "activate"):
            subprocess.run(["ros2", "lifecycle", "set", "/gn_loc", transition], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
            spin(1)
        map_pub.publish(grid)
        spin(2)
        init = PoseWithCovarianceStamped(); init.header.frame_id = "map"; init.pose.pose.orientation.w = 1.0
        c = [0.0] * 36; c[0] = c[7] = 0.01; c[35] = 0.01; init.pose.covariance = c
        init_pub.publish(init)
        spin(6)
    finally:
        proc.terminate()
        log = proc.communicate(timeout=20)[0]
        z, pose = got["z"], got["pose"]
        n.destroy_node(); rclpy.shutdown()
    return log, z, pose


def test_map_to_odom_puts_the_base_on_the_floor_and_a_scan_above_the_base_is_accepted():
    log, z, pose = run("lidar_level", 0.0)
    assert z is not None, "no map -> odom\n" + log
    assert abs(z + ODOM_Z) < 1e-6, z
    assert pose is not None, "no gn_loc/pose"
    assert "not straight above" not in log, log


def test_a_scan_off_to_the_side_of_the_base_is_warned_about():
    log, _, _ = run("off_centre", 0.1)
    assert "scan frame 'off_centre' is not straight above" in log, log


if __name__ == "__main__":
    try:
        import rclpy  # noqa: F401
    except ImportError:
        print("skip: no rclpy (run where gn_loc is built)")
        sys.exit(0)
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
