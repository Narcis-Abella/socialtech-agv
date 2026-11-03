"""FAST-LIO2's odometry against the simulator's ground truth, for the full-LIO simulation (lio_sim.sh).
record:  lio_eval.py record OUT.csv [seconds]     rclpy node: /Odometry (FAST-LIO2) and /odom_gt_raw (Gazebo model pose), one row per odometry message
report:  lio_eval.py report OUT.csv               alignment on the first row (the two frames start unrelated), then the error of the aligned track
The error is position (x, y, z) and yaw of FAST-LIO2 against the truth, after one rigid alignment at the start: the drift the stack really has."""
import csv
import math
import sys

import numpy as np


def yaw(q):
    return math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def se2_align(lio0, gt0):
    """Planar rigid transform (R, t) that takes the LIO frame onto the truth frame so that the first poses coincide."""
    d = wrap(gt0[2] - lio0[2])
    R = np.array([[math.cos(d), -math.sin(d)], [math.sin(d), math.cos(d)]])
    return d, R, np.asarray(gt0[:2]) - R @ np.asarray(lio0[:2])


def errors(rows):
    """rows: [t, lx, ly, lz, lyaw, gx, gy, gz, gyaw] -> dict of the error series after aligning on the first row."""
    a = np.asarray(rows, float)
    d, R, t = se2_align(a[0, 1:5][[0, 1, 3]], a[0, 5:9][[0, 1, 3]])
    xy = a[:, 1:3] @ R.T + t
    exy = np.hypot(*(xy - a[:, 5:7]).T)
    ez = (a[:, 3] - a[0, 3]) - (a[:, 7] - a[0, 7])
    eyaw = np.array([wrap(l + d - g) for l, g in zip(a[:, 4], a[:, 8])])
    path = float(np.sum(np.hypot(*np.diff(a[:, 5:7], axis=0).T)))
    return {'t': a[:, 0], 'xy': exy, 'z': ez, 'yaw': eyaw, 'path_m': path}


def summary(e):
    f = lambda x: {'median': float(np.median(np.abs(x))), 'p95': float(np.percentile(np.abs(x), 95)), 'max': float(np.abs(x).max()), 'final': float(x[-1])}
    return {'n': int(len(e['t'])), 'duration_s': float(e['t'][-1] - e['t'][0]), 'path_m': e['path_m'], 'xy_m': f(e['xy']), 'z_m': f(e['z']), 'yaw_deg': f(np.degrees(e['yaw']))}


def report(path):
    rows = [[float(v) for v in r] for r in list(csv.reader(open(path)))[1:]]
    s = summary(errors(rows))
    print(f"{s['n']} rows, {s['duration_s']:.0f} s, {s['path_m']:.1f} m driven")
    for k, u in (('xy_m', 'm'), ('z_m', 'm'), ('yaw_deg', 'deg')):
        print(f"  {k:8s} median {s[k]['median']:.4f} p95 {s[k]['p95']:.4f} max {s[k]['max']:.4f} final {s[k]['final']:+.4f} {u}")
    return s


def record(path, seconds):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    class Rec(Node):
        def __init__(self):
            super().__init__('lio_eval')
            self.gt, self.f = None, open(path, 'w', newline='')
            self.w = csv.writer(self.f)
            self.w.writerow(['t', 'lx', 'ly', 'lz', 'lyaw', 'gx', 'gy', 'gz', 'gyaw'])
            self.create_subscription(Odometry, '/odom_gt_raw', lambda m: setattr(self, 'gt', m), QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(Odometry, '/Odometry', self.on_lio, 10)

        def on_lio(self, m):
            if self.gt is None:
                return
            p, q, g, gq = m.pose.pose.position, m.pose.pose.orientation, self.gt.pose.pose.position, self.gt.pose.pose.orientation
            self.w.writerow([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z, yaw((q.x, q.y, q.z, q.w)), g.x, g.y, g.z, yaw((gq.x, gq.y, gq.z, gq.w))])

    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    n = Rec()
    t0 = n.get_clock().now()
    while rclpy.ok() and (n.get_clock().now() - t0).nanoseconds < seconds * 1e9:
        rclpy.spin_once(n, timeout_sec=.1)
    n.f.close()


if __name__ == '__main__':
    if len(sys.argv) >= 3 and sys.argv[1] == 'record':
        record(sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 120.)
    elif len(sys.argv) == 3 and sys.argv[1] == 'report':
        report(sys.argv[2])
    else:
        print(__doc__)
