"""A/B of RPP's in-place rotation over max_angular_accel. THE ROBOT TURNS (no translation). usage: rot_ab.py LOG.csv A [A ...] [--delta DEG=120] [--reps N=2] [--seed S=1] | rot_ab.py --dry - A [A ...]
Runs where the stack is (docker exec into the nav container). Per trial: set controller_server FollowPath.max_angular_accel = A, send a FollowPath goal (frame odom) whose path is two poses at the
robot's position, the second rotated by +-delta (each A is run in both senses). The robot is inside the goal's xy tolerance, so RPP rotates in place to the goal heading with its own loop
(rotate_to_heading_angular_vel, clamp around the measured /odom w). Trials run in a shuffled order. LOG.csv: t_wall start, A, delta deg, result, seconds. Cancels the goal and restores A on exit;
abort if /odom shows translation > 0.1 m/s. Analysis: rot_ab_eval.py on the bag."""
import csv
import math
import random
import sys
import time

TIMEOUT, PAUSE = 25.0, 3.0

def plan(accels, reps, delta, seed):
    """[(A, signed delta deg)]: every A `reps` times, the sense alternating per repetition (A gets both senses; net rotation zero for even reps), in shuffled order"""
    trials = [(a, delta * (1 if r % 2 == 0 else -1)) for a in accels for r in range(reps)]; random.Random(seed).shuffle(trials)
    return trials

def main(log_path, accels, reps, delta, seed):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import FollowPath
    from nav_msgs.msg import Odometry, Path
    from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters, SetParameters
    from rclpy.action import ActionClient
    from tf2_ros import Buffer, TransformListener
    rclpy.init(); node = rclpy.create_node("rot_ab"); buf = Buffer(); TransformListener(buf, node); odo = {}
    node.create_subscription(Odometry, "/odom", lambda m: odo.update(v=m.twist.twist.linear.x, w=m.twist.twist.angular.z), 10)
    ac = ActionClient(node, FollowPath, "follow_path"); setc = node.create_client(SetParameters, "/controller_server/set_parameters"); getc = node.create_client(GetParameters, "/controller_server/get_parameters")
    name = "FollowPath.max_angular_accel"

    def call(cli, req):
        f = cli.call_async(req); rclpy.spin_until_future_complete(node, f, timeout_sec=5.0)
        if not f.done(): raise SystemExit("service call timed out")
        return f.result()

    def set_a(a):
        p = ParamMsg(name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(a)))
        if not call(setc, SetParameters.Request(parameters=[p])).results[0].successful: raise SystemExit(f"could not set {name}={a}")

    def pose_odom():
        t = buf.lookup_transform("odom", "base_footprint", rclpy.time.Time()).transform
        q = t.rotation; return t.translation.x, t.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def stamped(x, y, yaw):
        m = PoseStamped(); m.header.frame_id = "odom"; m.pose.position.x = x; m.pose.position.y = y
        m.pose.orientation.z = math.sin(yaw / 2); m.pose.orientation.w = math.cos(yaw / 2); return m

    if not ac.wait_for_server(timeout_sec=10.0): raise SystemExit("no follow_path action server")
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.1)
        if odo: break
    if not odo or abs(odo["v"]) > 0.02 or abs(odo["w"]) > 0.02: raise SystemExit(f"robot not still or no /odom: {odo}")
    for _ in range(100):   # the TF listener fills its buffer only while the node spins
        rclpy.spin_once(node, timeout_sec=0.1)
        if buf.can_transform("odom", "base_footprint", rclpy.time.Time()): break
    else: raise SystemExit("no TF odom -> base_footprint")
    orig = call(getc, GetParameters.Request(names=[name])).values[0].double_value
    log = open(log_path, "w", newline=""); out = csv.writer(log); out.writerow(["t_wall", "a", "delta_deg", "result", "seconds"]); gh = None
    try:
        for a, d in plan(accels, reps, delta, seed):
            set_a(a); x, y, yaw = pose_odom(); path = Path(); path.header.frame_id = "odom"
            path.poses = [stamped(x, y, yaw), stamped(x, y, yaw + math.radians(d))]
            goal = FollowPath.Goal(path=path, controller_id="FollowPath", goal_checker_id="general_goal_checker", progress_checker_id="progress_checker")
            t_wall, t0 = time.time(), time.monotonic(); print(f"a = {a} delta = {d:+.0f} deg", flush=True)
            f = ac.send_goal_async(goal); rclpy.spin_until_future_complete(node, f, timeout_sec=5.0); gh = f.result()
            if gh is None or not gh.accepted: out.writerow([f"{t_wall:.3f}", a, d, "rejected", 0]); log.flush(); gh = None; continue
            rf = gh.get_result_async(); result = "timeout"
            while time.monotonic() - t0 < TIMEOUT:
                rclpy.spin_once(node, timeout_sec=0.05)
                if abs(odo["v"]) > 0.1: raise SystemExit(f"abort: translating v={odo['v']:.2f}")
                if rf.done(): result = {4: "succeeded", 5: "canceled", 6: "aborted"}.get(rf.result().status, str(rf.result().status)); break
            if result == "timeout": rclpy.spin_until_future_complete(node, gh.cancel_goal_async(), timeout_sec=3.0)
            gh = None; secs = time.monotonic() - t0; out.writerow([f"{t_wall:.3f}", a, d, result, f"{secs:.2f}"]); log.flush(); print(f"  -> {result} in {secs:.1f} s", flush=True)
            end = time.monotonic() + PAUSE
            while time.monotonic() < end: rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if gh is not None: gh.cancel_goal_async()
        try: set_a(orig)
        except BaseException: pass
        log.close(); rclpy.shutdown()

if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(); ap.add_argument("log"); ap.add_argument("accels", type=float, nargs="+")
    ap.add_argument("--delta", type=float, default=120.0); ap.add_argument("--reps", type=int, default=2); ap.add_argument("--seed", type=int, default=1); ap.add_argument("--dry", action="store_true")
    o = ap.parse_args()
    if o.dry:
        p = plan(o.accels, o.reps, o.delta, o.seed); assert len(p) == o.reps * len(o.accels) and abs(sum(d for _, d in p)) <= o.delta + 1e-9; print(len(p), "trials:", p)
    else: main(o.log, o.accels, o.reps, o.delta, o.seed)
