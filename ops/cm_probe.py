"""Collision Monitor throughput and latency in isolation. Sends a ~stationary command (0.001 rad/s: 0.06 deg/s). usage (inside the nav container): cm_probe.py [SECONDS=10] [RATE=100] | cm_probe.py --selftest
Publishes on the CM input (/cmd_vel_smoothed) at RATE Hz an angular.z = 0.001 + k * 1e-6 (k = sequence number) and looks each value up on its output (/cmd_vel). Prints how many of the inputs came out,
the output rate and the latency (output receive time - input publish time) per matched message; one clock. The smoother must be idle (no Nav2 goal running)."""
import statistics
import sys
import time

def match(sent, got):
    """sent {k: t_pub}, got [(t_rx, w)] -> [(k, latency s)] for the outputs whose value is 0.001 + k * 1e-6"""
    out = []
    for t, w in got:
        k = round((w - 0.001) / 1e-6)
        if k in sent and abs(w - (0.001 + k * 1e-6)) < 2e-7: out.append((k, t - sent[k]))
    return out

def selftest():
    sent = {k: 100.0 + 0.01 * k for k in range(100)}; got = [(100.0 + 0.01 * k + 0.12, 0.001 + k * 1e-6) for k in range(0, 100, 10)] + [(150.0, 0.5)]
    m = match(sent, got); assert len(m) == 10 and all(abs(l - 0.12) < 1e-9 for _, l in m), m
    print("selftest ok")

def main(secs, rate):
    import rclpy
    from geometry_msgs.msg import Twist
    rclpy.init(); node = rclpy.create_node("cm_probe"); pub = node.create_publisher(Twist, "/cmd_vel_smoothed", 10); sent, got = {}, []
    node.create_subscription(Twist, "/cmd_vel", lambda m: got.append((time.time(), m.angular.z)), 100)
    for _ in range(20): rclpy.spin_once(node, timeout_sec=0.1)
    msg = Twist(); k = 0; t0 = time.monotonic(); nxt = t0
    try:
        while time.monotonic() - t0 < secs:
            if time.monotonic() >= nxt:
                msg.angular.z = 0.001 + k * 1e-6; sent[k] = time.time(); pub.publish(msg); k += 1; nxt += 1 / rate
            rclpy.spin_once(node, timeout_sec=0.001)
        for _ in range(100): rclpy.spin_once(node, timeout_sec=0.01)
    finally:
        msg.angular.z = 0.0
        for _ in range(10): pub.publish(msg); time.sleep(0.02)
        rclpy.shutdown()
    m = match(sent, got); lat = sorted(l for _, l in m)
    print(f"sent {len(sent)} in {secs:.0f} s ({len(sent) / secs:.0f} Hz); came out {len(m)} ({100 * len(m) / max(len(sent), 1):.0f} %), output rate {len(m) / secs:.1f} Hz")
    if lat: print(f"latency ms: median {1e3 * statistics.median(lat):.0f}, p5 {1e3 * lat[len(lat) // 20]:.0f}, p95 {1e3 * lat[-len(lat) // 20 - 1]:.0f}, max {1e3 * lat[-1]:.0f}")
    if len(m) > 3: gaps = [b[0] - a[0] for a, b in zip(m, m[1:])]; print(f"input index gap between consecutive outputs: median {statistics.median(gaps):.0f} (1 = every message processed)")

if __name__ == "__main__":
    selftest() if len(sys.argv) > 1 and sys.argv[1] == "--selftest" else main(float(sys.argv[1]) if len(sys.argv) > 1 else 10.0, float(sys.argv[2]) if len(sys.argv) > 2 else 100.0)
