"""Runs the trials of a scenario file against Nav2 in the simulation and records raw series for offline analysis (metrics.py).
usage (inside the sim image, with the simulation and Nav2 running): trials.py <scenario.json> <out dir> [reps] --ros-args -p use_sim_time:=true
Each trial: put the robot at `start` (Gazebo set_pose), reset AMCL and the costmaps, wait until the estimate is within 0.3 m of the ground truth, send the goal, record until the result.
env: INIT_SIGMA_XY (0.1 m), INIT_SIGMA_YAW (0.1 rad) of the /initialpose covariance; INIT_OFFSET_M / INIT_OFFSET_DEG (0, 0): the pose given to AMCL is the true start moved by that much (direction turns 60 deg per trial, yaw sign alternates) to test convergence; SETTLE_TIMEOUT (15 s) how long to wait for the estimate to be within 0.3 m before sending the goal; TRIAL_TIMEOUT (240 s of simulation time); ROBOT_NAME (tracer_robot); WORLD (loyola)."""
import json
import math
import os
import subprocess
import sys

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from initpose import init_offset

HZ = 50.0


def yaw_to_quat(yaw):
    return math.sin(yaw / 2), math.cos(yaw / 2)       # (z, w)


def quat_to_yaw(z, w):
    return 2 * math.atan2(z, w)


def path_xy(msg):
    return [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in msg.poses]


class Runner:
    def __init__(self, node, out_dir):
        self.n, self.out = node, out_dir
        self.client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self.tf = Buffer()
        TransformListener(self.tf, node)
        self.gt = None
        self.cmd = {"cmd_vel_nav": (0.0, 0.0), "cmd_vel_smoothed": (0.0, 0.0), "cmd_vel": (0.0, 0.0)}
        self.recording, self.rows, self.plans, self.k = False, [], [], 0
        node.create_subscription(PoseStamped, "/gt/pose", self.on_gt, 10)
        for name in self.cmd:
            node.create_subscription(Twist, "/" + name, lambda m, k=name: self.cmd.__setitem__(k, (m.linear.x, m.angular.z)), 10)
        node.create_subscription(Path, "/plan", self.on_plan, 10)
        self.init_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.clear = [node.create_client(ClearEntireCostmap, s) for s in ("/local_costmap/clear_entirely_local_costmap", "/global_costmap/clear_entirely_global_costmap")]
        node.create_timer(1.0 / HZ, self.sample)

    def now(self):
        return self.n.get_clock().now().nanoseconds / 1e9

    def on_gt(self, m):
        o = m.pose.orientation
        self.gt = (m.pose.position.x, m.pose.position.y, quat_to_yaw(o.z, o.w))

    def on_plan(self, m):
        if self.recording:
            self.plans.append((self.now(), path_xy(m)))

    def estimate(self):
        try:
            t = self.tf.lookup_transform("map", "base_footprint", Time()).transform
        except TransformException:
            return None
        return (t.translation.x, t.translation.y, quat_to_yaw(t.rotation.z, t.rotation.w))

    def sample(self):
        if not self.recording or self.gt is None:
            return
        est = self.estimate() or (math.nan,) * 3
        self.rows.append([round(self.now(), 4), *[round(v, 4) for v in self.gt], *[round(v, 4) for v in est],
                          *[round(v, 4) for k in self.cmd for v in self.cmd[k]]])

    def spin_for(self, seconds):
        end = self.now() + seconds
        while self.now() < end:
            rclpy.spin_once(self.n, timeout_sec=0.02)

    def reset(self, start, offset=(0.0, 0.0, 0.0)):
        x, y, yaw = start
        z, w = yaw_to_quat(yaw)
        req = f'name: "{os.environ.get("ROBOT_NAME", "tracer_robot")}" position {{x: {x} y: {y} z: 0.15}} orientation {{z: {z} w: {w}}}'
        subprocess.run(["gz", "service", "-s", f"/world/{os.environ.get('WORLD', 'loyola')}/set_pose", "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                        "--timeout", "3000", "--req", req], check=True, capture_output=True)
        self.spin_for(2.0)
        m = PoseWithCovarianceStamped()
        m.header.frame_id, m.header.stamp = "map", self.n.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = x + offset[0], y + offset[1]
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = yaw_to_quat(yaw + offset[2])
        sxy, syaw = float(os.environ.get("INIT_SIGMA_XY", 0.1)), float(os.environ.get("INIT_SIGMA_YAW", 0.1))
        m.pose.covariance[0] = m.pose.covariance[7] = sxy ** 2
        m.pose.covariance[35] = syaw ** 2
        self.init_pub.publish(m)
        self.spin_for(1.0)
        for c in self.clear:
            if c.wait_for_service(timeout_sec=2.0):
                rclpy.spin_until_future_complete(self.n, c.call_async(ClearEntireCostmap.Request()), timeout_sec=3.0)
        t_end, good_since, err = self.now() + float(os.environ.get("SETTLE_TIMEOUT", 15)), None, math.nan
        while self.now() < t_end:
            rclpy.spin_once(self.n, timeout_sec=0.05)
            est = self.estimate()
            if est and self.gt:
                err = math.hypot(est[0] - self.gt[0], est[1] - self.gt[1])
                if err < 0.3:
                    good_since = good_since or self.now()
                    if self.now() - good_since >= 1.0:
                        break
                else:
                    good_since = None
        return err

    def run_trial(self, trial, tag):
        offset = init_offset(self.k, float(os.environ.get("INIT_OFFSET_M", 0)), float(os.environ.get("INIT_OFFSET_DEG", 0)))
        self.k += 1
        init_err = self.reset(trial["start"], offset)
        goal = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id, p.header.stamp = "map", self.n.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = trial["goal"][0], trial["goal"][1]
        p.pose.orientation.z, p.pose.orientation.w = yaw_to_quat(trial["goal"][2])
        goal.pose = p
        feedback = []
        self.rows, self.plans, self.recording = [], [], True
        t0 = self.now()
        sent = self.client.send_goal_async(goal, feedback_callback=lambda f: feedback.append(
            [round(self.now(), 3), f.feedback.number_of_recoveries, round(f.feedback.distance_remaining, 3)]))
        rclpy.spin_until_future_complete(self.n, sent)
        handle = sent.result()
        status, code, msg = "rejected", -1, ""
        if handle.accepted:
            result = handle.get_result_async()
            limit = float(os.environ.get("TRIAL_TIMEOUT", 240))
            while not result.done() and self.now() - t0 < limit:
                rclpy.spin_once(self.n, timeout_sec=0.02)
            if result.done():
                r = result.result()
                status = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_ABORTED: "aborted", GoalStatus.STATUS_CANCELED: "canceled"}.get(r.status, str(r.status))
                code, msg = int(getattr(r.result, "error_code", -1)), str(getattr(r.result, "error_msg", ""))
            else:
                status = "timeout"
                rclpy.spin_until_future_complete(self.n, handle.cancel_goal_async(), timeout_sec=5.0)
        t1 = self.now()
        self.recording = False
        self.spin_for(1.0)
        rec = {"id": trial["id"], "tag": tag, "start": trial["start"], "goal": trial["goal"], "shortest": trial.get("shortest"), "init_err": init_err, "init_offset": [round(v, 4) for v in offset],
               "status": status, "error_code": code, "error_msg": msg, "t0": round(t0, 3), "t1": round(t1, 3), "duration": round(t1 - t0, 3),
               "recoveries": feedback[-1][1] if feedback else 0, "feedback": feedback, "n_plans": len(self.plans),
               "plan_first": self.plans[0][1] if self.plans else [], "plan_last": self.plans[-1][1] if self.plans else [],
               "plan_times": [round(t, 3) for t, _ in self.plans],
               "cols": ["t", "gt_x", "gt_y", "gt_yaw", "est_x", "est_y", "est_yaw", "nav_vx", "nav_wz", "sm_vx", "sm_wz", "cmd_vx", "cmd_wz"], "rows": self.rows}
        json.dump(rec, open(os.path.join(self.out, f"trial_{tag}_{trial['id']}.json"), "w"))
        print(f"{tag} {trial['id']}: {status} t={t1 - t0:.1f}s init_err={init_err:.2f} recoveries={rec['recoveries']} code={code}", flush=True)
        return rec


def main(scenario, out_dir, reps=1):
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("trials")
    os.makedirs(out_dir, exist_ok=True)
    r = Runner(node, out_dir)
    r.client.wait_for_server()
    trials = json.load(open(scenario))["trials"]
    while r.gt is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    for k in range(int(reps)):
        for t in trials:
            r.run_trial(t, f"r{k}")
    rclpy.shutdown()


if __name__ == "__main__":
    main(*(sys.argv[1:sys.argv.index("--ros-args")] if "--ros-args" in sys.argv else sys.argv[1:]))
