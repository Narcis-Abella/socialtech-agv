"""Per-stage latency of the command chain, from controlled steps. THE ROBOT TURNS in place (+-). usage (inside the nav container): delay_probe.py OUT_PREFIX [chain] | delay_probe.py --dry   ("chain": only the 8 steps through the whole chain)
Angular-velocity steps are published either on /cmd_vel_nav (whole chain: smoother, Collision Monitor, base) or directly on /cmd_vel (the base alone). Every message of the chain is stamped with the
receive time on one clock (wall time, ns): /cmd_vel_nav, /cmd_vel_smoothed, /cmd_vel, /odom (+ its header stamp), /tracer_status rpm, /livox/imu gyro z. Capture the CAN bus at the same time on the host:
  timeout 200 candump -L can_tracer > can.log      (same CLOCK_REALTIME). Writes OUT_PREFIX_raw.csv (t_ns, topic, a, b, stamp_ns) and OUT_PREFIX_steps.csv (t_ns, mode, w). Aborts if |v| > 0.1 m/s.
Analysis: delay_eval.py."""
import csv
import sys
import time

HOLD, REST, RATE = 1.5, 3.0, 20.0
SCHEDULE = [("chain", 0.6)] * 8 + [("direct", 0.6)] * 8 + [("direct", 0.3)] * 4 + [("direct", 0.15)] * 4

def plan(only=None):
    """[(mode, signed w)]: the sense alternates along the sequence (net rotation zero); only = keep that mode"""
    return [(m, w * (1 if i % 2 == 0 else -1)) for i, (m, w) in enumerate(s for s in SCHEDULE if only in (None, s[0]))]

def main(prefix, only=None):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from tracer_msgs.msg import TracerStatus
    rclpy.init(); node = rclpy.create_node("delay_probe"); rows, odo = [], {}
    stamp = lambda h: h.stamp.sec * 10**9 + h.stamp.nanosec
    for name in ("/cmd_vel_nav", "/cmd_vel_smoothed", "/cmd_vel"):
        node.create_subscription(Twist, name, lambda m, n=name: rows.append((time.time_ns(), n, m.linear.x, m.angular.z, 0)), 100)

    def on_odom(m):
        odo.update(v=m.twist.twist.linear.x, w=m.twist.twist.angular.z); rows.append((time.time_ns(), "/odom", odo["v"], odo["w"], stamp(m.header)))
    node.create_subscription(Odometry, "/odom", on_odom, 100)
    node.create_subscription(TracerStatus, "/tracer_status", lambda m: rows.append((time.time_ns(), "/tracer_status", m.actuator_states[0].rpm, m.actuator_states[1].rpm, stamp(m.header))), 100)
    node.create_subscription(Imu, "/livox/imu", lambda m: rows.append((time.time_ns(), "/livox/imu", m.angular_velocity.z, 0.0, stamp(m.header))), qos_profile_sensor_data)
    pubs = {"chain": node.create_publisher(Twist, "/cmd_vel_nav", 10), "direct": node.create_publisher(Twist, "/cmd_vel", 10)}
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.1)
        if odo: break
    if not odo or abs(odo["v"]) > 0.02 or abs(odo["w"]) > 0.02: raise SystemExit(f"robot not still or no /odom: {odo}")
    steps = []; msg = Twist()

    def hold(pub, w, secs):
        msg.angular.z = w; end = time.monotonic() + secs; nxt = time.monotonic()
        while time.monotonic() < end:
            if time.monotonic() >= nxt: pub.publish(msg); nxt += 1 / RATE
            rclpy.spin_once(node, timeout_sec=0.002)
            if abs(odo["v"]) > 0.1: raise SystemExit(f"abort: translating v={odo['v']:.2f}")
    try:
        for mode, w in plan(only):
            pub = pubs[mode]; steps.append((time.time_ns(), mode, w)); print(f"{mode} w = {w:+.2f}", flush=True)
            hold(pub, w, HOLD); hold(pub, 0.0, REST)
    finally:
        msg.angular.z = 0.0
        for pub in pubs.values():
            for _ in range(10): pub.publish(msg); time.sleep(0.02)
        with open(prefix + "_raw.csv", "w", newline="") as f: csv.writer(f).writerows(rows)
        with open(prefix + "_steps.csv", "w", newline="") as f: csv.writer(f).writerows(steps)
        rclpy.shutdown()

if __name__ == "__main__":
    if sys.argv[1] == "--dry":
        p = plan(); c = plan("chain"); assert abs(sum(w for _, w in p)) < 1e-9 and len(p) == len(SCHEDULE) and len(c) == 8 and abs(sum(w for _, w in c)) < 1e-9; print(len(p), "steps,", f"{len(p) * (HOLD + REST):.0f} s;", len(c), "chain steps")
    else: main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
