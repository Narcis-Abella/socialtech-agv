"""Constant in-place angular commands on /cmd_vel_nav, to measure the base's static response. THE ROBOT TURNS. usage: rot_static_pub.py LOG.csv | rot_static_pub.py --dry
Runs where ROS and the stack are (docker exec into the nav container). For each level: +w for HOLD s, pause, -w for HOLD s, pause (net rotation zero). The command goes through the real chain
(velocity smoother -> Collision Monitor -> base). LOG.csv: wall time (s), commanded w, per segment. Always sends zeros on exit; aborts if /odom shows translation > 0.1 m/s or |w| > 1.3 rad/s.
Analysis: rot_gain.py on the bag of the run."""
import csv
import sys
import time

LEVELS = (0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.50, 0.60)
HOLD, PAUSE, RATE = 4.0, 2.0, 20.0

def plan():
    """[(w, seconds)]: for each level +w, pause, -w, pause"""
    return [seg for w in LEVELS for seg in ((w, HOLD), (0.0, PAUSE), (-w, HOLD), (0.0, PAUSE))]

def main(log_path):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    rclpy.init(); node = rclpy.create_node("rot_static_pub"); pub = node.create_publisher(Twist, "/cmd_vel_nav", 10); odo = {}
    node.create_subscription(Odometry, "/odom", lambda m: odo.update(v=m.twist.twist.linear.x, w=m.twist.twist.angular.z), 10)
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.1)
        if odo: break
    if not odo: raise SystemExit("no /odom: is the base on?")
    if abs(odo["v"]) > 0.02 or abs(odo["w"]) > 0.02: raise SystemExit(f"robot not still: v={odo['v']:.3f} w={odo['w']:.3f}")
    msg = Twist(); log = open(log_path, "w", newline=""); out = csv.writer(log); out.writerow(["t_wall", "w_cmd"])
    try:
        for w, dur in plan():
            msg.angular.z = w; out.writerow([f"{time.time():.3f}", w]); log.flush(); end = time.monotonic() + dur
            print(f"w = {w:+.2f} for {dur:.0f} s", flush=True)
            while time.monotonic() < end:
                pub.publish(msg); rclpy.spin_once(node, timeout_sec=1 / RATE)
                if abs(odo["v"]) > 0.1 or abs(odo["w"]) > 1.3: raise SystemExit(f"abort: v={odo['v']:.2f} w={odo['w']:.2f}")
    finally:
        msg.angular.z = 0.0
        for _ in range(20): pub.publish(msg); time.sleep(0.05)
        out.writerow([f"{time.time():.3f}", 0.0]); log.close(); rclpy.shutdown()

if __name__ == "__main__":
    if sys.argv[1] == "--dry":
        p = plan(); assert abs(sum(w for w, _ in p)) < 1e-9 and all(abs(w) <= 0.6 for w, _ in p); print(f"{len(p)} segments, {sum(d for _, d in p):.0f} s, levels {LEVELS}")
    else: main(sys.argv[1])
