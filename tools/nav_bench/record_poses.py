"""Record poses from a ROS topic to a text file (t x y z qx qy qz qw [cov_xx cov_yy cov_yawyaw]), t = header.stamp; stops when the topic goes quiet.
usage: record_poses.py <topic> <odom|amcl> <out.txt> [--idle 15] [--max-wait 120]
odom = nav_msgs/Odometry (FAST-LIO2 /Odometry); amcl = geometry_msgs/PoseWithCovarianceStamped (/amcl_pose). Exit 1 if nothing arrives within --max-wait."""
import argparse
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("topic")
    ap.add_argument("kind", choices=("odom", "amcl"))
    ap.add_argument("out")
    ap.add_argument("--idle", type=float, default=15.0)
    ap.add_argument("--max-wait", type=float, default=120.0)
    a = ap.parse_args()

    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = rclpy.create_node("record_poses_" + a.kind)
    f = open(a.out, "w")
    last = [None]

    def cb(m):
        p, q, c = m.pose.pose.position, m.pose.pose.orientation, m.pose.covariance
        row = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if a.kind == "amcl":
            row += [c[0], c[7], c[35]]
        f.write(" ".join(f"{v:.9f}" for v in row) + "\n")
        f.flush()
        last[0] = time.monotonic()

    # amcl_pose is transient_local; /Odometry is volatile (a transient_local subscriber would not match it)
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL if a.kind == "amcl" else DurabilityPolicy.VOLATILE)
    node.create_subscription(PoseWithCovarianceStamped if a.kind == "amcl" else Odometry, a.topic, cb, qos)
    t0 = time.monotonic()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)
        now = time.monotonic()
        if last[0] is None and now - t0 > a.max_wait:
            print(f"no message on {a.topic} within {a.max_wait:.0f} s", file=sys.stderr)
            sys.exit(1)
        if last[0] is not None and now - last[0] > a.idle:
            break
    f.close()


if __name__ == "__main__":
    main()
