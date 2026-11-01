"""Diagnostic: which kind of Gazebo model can be moved with set_pose AND is seen by the lidar AND stops the robot? Run by dyn_probe.sh inside the sim image, with the simulation running.
For each model: teleport it 3 m ahead of the robot, read the lidar sector ahead; teleport it 2 m ahead, read again; then drive into it at 0.3 m/s and report where the robot ends up."""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan

import dyn

MODELS = sys.argv[1].split(",")


def main():
    rclpy.init(args=sys.argv)
    n = rclpy.create_node("dyn_probe")
    st = {"scan": None, "gt": None}
    n.create_subscription(LaserScan, "/scan", lambda m: st.__setitem__("scan", m), 10)
    n.create_subscription(PoseStamped, "/gt/pose", lambda m: st.__setitem__("gt", (m.pose.position.x, m.pose.position.y)), 10)
    pub = n.create_publisher(Twist, "/cmd_vel", 10)
    mv = dyn.Mover("loyola", lambda: n.get_clock().now().nanoseconds / 1e9)

    def spin(s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.02)

    def front():
        m = st["scan"]
        if m is None:
            return None
        a = [m.ranges[i] for i in range(len(m.ranges)) if abs(m.angle_min + i * m.angle_increment) < math.radians(8) and m.range_min < m.ranges[i] < m.range_max]
        return round(min(a), 2) if a else None

    def robot_to(x):
        from geometry_msgs.msg import PoseStamped as _P  # noqa: F401
        mv.put("tracer_robot", x, 0.0, 0.0, 0.15)
        spin(2.0)

    while st["scan"] is None or st["gt"] is None:
        spin(0.2)
    print(f"PROBE topics scan frame={st['scan'].header.frame_id} n={len(st['scan'].ranges)} range_max={st['scan'].range_max}", flush=True)
    for k, name in enumerate(MODELS + ["(none)"]):
        robot_to(2.0)
        for m in MODELS:
            mv.put(m, 12.0, -30.0 - 2 * MODELS.index(m), 0.0, 0.85)
        base = front()
        res = {"baseline": base}
        if name != "(none)":
            gx = st["gt"][0]
            mv.put(name, gx + 3.0, 0.0, 0.0, 0.85); spin(1.5); res["at3m"] = front()
            mv.put(name, gx + 2.0, 0.0, 0.0, 0.85); spin(1.5); res["at2m"] = front()
            t = Twist(); t.linear.x = 0.3
            end = time.time() + 14
            while time.time() < end:
                pub.publish(t); spin(0.05)
            pub.publish(Twist()); spin(0.5)
            res["robot_x_after_driving_into_it"] = round(st["gt"][0], 2)
            res["obstacle_x"] = round(gx + 2.0, 2)
        print(f"PROBE {name}: {res}", flush=True)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
