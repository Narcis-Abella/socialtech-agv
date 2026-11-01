"""Send one recorded goal to Nav2 and measure how the robot sets off. THE ROBOT MOVES. Runs where ROS and the stack are (docker exec into the nav container).
  goal_test.py [INDEX|auto] [--timeout 60]      INDEX = 0-based position in goals_lab.yaml; auto = the goal that needs the biggest turn from the current AMCL pose (1.2-5 m away)
The goal is oriented along the line robot -> goal, so no final turn adds to the measure. Reported: time to the first translation, and while turning in place before it
(commanded |w| > 0.05 rad/s, |vx| < 0.02): duration, mean and peak commanded |w|, and the angle the base itself reports (integral of /odom w).
Always cancels the goal on exit: Ctrl-C, timeout or error."""
import argparse
import math
import os
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

W_MIN, VX_MAX_ROTATING, VX_MIN_MOVING = 0.05, 0.02, 0.05


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def pick(goals, x, y, yaw):
    """-> index of the goal 1.2-5 m away whose bearing differs most from `yaw`."""
    best = None
    for i, g in enumerate(goals):
        d = math.hypot(g["x"] - x, g["y"] - y)
        turn = abs(wrap(math.atan2(g["y"] - y, g["x"] - x) - yaw))
        if 1.2 <= d <= 5 and (best is None or turn > best[0]):
            best = (turn, i)
    return None if best is None else best[1]


def phases(cmd, odom, t_send):
    """cmd: [(t, vx, w)] of /cmd_vel_nav, odom: [(t, w)]. -> dict of the turning-in-place phase before the first translation."""
    translate = next((t for t, vx, _ in cmd if t >= t_send and vx > VX_MIN_MOVING), None)
    rot = [(t, w) for t, vx, w in cmd if t >= t_send and (translate is None or t < translate) and abs(w) > W_MIN and abs(vx) < VX_MAX_ROTATING]
    out = {"first_translation_s": None if translate is None else translate - t_send}
    if rot:
        a, b = rot[0][0], rot[-1][0]
        base = [(t, w) for t, w in odom if a - 0.05 <= t <= b + 0.05]
        out.update(turn_s=b - a, mean_w=sum(abs(w) for _, w in rot) / len(rot), peak_w=max(abs(w) for _, w in rot),
                   base_turned_deg=math.degrees(abs(sum(w * (t2 - t1) for (t1, w), (t2, _) in zip(base, base[1:])))))
    return out


class Probe(Node):
    def __init__(self):
        super().__init__("goal_test")
        self.cmd, self.odom, self.pose = [], [], None
        self.create_subscription(Twist, "/cmd_vel_nav", lambda m: self.cmd.append((time.time(), m.linear.x, m.angular.z)), 10)
        self.create_subscription(Odometry, "/odom", lambda m: self.odom.append((time.time(), m.twist.twist.angular.z)), 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", lambda m: setattr(self, "pose", m.pose.pose), latched)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("goal", nargs="?", default="auto")
    ap.add_argument("--timeout", type=float, default=60)
    args = ap.parse_args()
    goals = yaml.safe_load(open(os.path.join(os.path.dirname(os.path.realpath(__file__)), "goals_lab.yaml")))

    rclpy.init()
    node = Probe()
    node.spin_for(2)
    if node.pose is None:
        raise SystemExit("no /amcl_pose: give the initial pose in rviz2 first")
    x, y, yaw = node.pose.position.x, node.pose.position.y, yaw_of(node.pose.orientation)
    index = pick(goals, x, y, yaw) if args.goal == "auto" else int(args.goal)
    if index is None:
        raise SystemExit("no goal 1.2-5 m away from here")
    g = goals[index]
    bearing = math.atan2(g["y"] - y, g["x"] - x)
    print(f"robot at ({x:.2f}, {y:.2f}) facing {math.degrees(yaw):.0f} deg; goal {g['name']} ({g['x']}, {g['y']}) at bearing {math.degrees(bearing):.0f} deg: "
          f"initial heading error {abs(math.degrees(wrap(bearing - yaw))):.0f} deg, distance {math.hypot(g['x'] - x, g['y'] - y):.2f} m", flush=True)

    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x, goal.pose.pose.position.y = float(g["x"]), float(g["y"])
    goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(bearing / 2), math.cos(bearing / 2)
    handle = None
    try:
        if not node.client.wait_for_server(timeout_sec=10):
            raise SystemExit("navigate_to_pose is not up (did Nav2 start? it waits for the initial pose)")
        t_send = time.time()
        future = node.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future)
        handle = future.result()
        if not handle.accepted:
            raise SystemExit("goal rejected")
        result = handle.get_result_async()
        while not result.done() and time.time() - t_send < args.timeout and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
        status = result.result().status if result.done() else "TIMEOUT (cancelled)"
        total = time.time() - t_send
    finally:
        if handle is not None and handle.accepted:
            node.spin_for(0.1)
            handle.cancel_goal_async()
            node.spin_for(0.5)
    print(f"result: status {status} after {total:.1f} s   (4 = succeeded)")
    info = phases(node.cmd, node.odom, t_send)
    first = info["first_translation_s"]
    print("first translation: " + ("never" if first is None else f"{first:.1f} s after sending the goal"))
    if "turn_s" in info:
        print(f"turning in place before it: {info['turn_s']:.1f} s, commanded |w| mean {info['mean_w']:.2f} / peak {info['peak_w']:.2f} rad/s, base turned {info['base_turned_deg']:.0f} deg")
    else:
        print("no turn in place before translating")


if __name__ == "__main__":
    main()
