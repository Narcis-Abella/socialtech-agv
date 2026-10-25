"""Runs the trials of a scenario file against Nav2 in the simulation and records raw series for offline analysis (metrics.py).
usage (inside the sim image, with the simulation and Nav2 running): trials.py <scenario.json> <out dir> [reps] --ros-args -p use_sim_time:=true
Each trial: put the robot at `start` (Gazebo set_pose), reset AMCL and the costmaps, wait until the estimate is within 0.3 m of the ground truth, send the goal, record until the result.
env: CHAIN (unset): 1 = only the first trial resets the robot and AMCL, the next ones start where the previous ended (long-horizon localisation drift); INIT_SIGMA_XY (0.1 m), INIT_SIGMA_YAW (0.1 rad) of the /initialpose covariance; INIT_OFFSET_M / INIT_OFFSET_DEG (0, 0): the pose given to AMCL is the true start moved by that much (direction turns 60 deg per trial, yaw sign alternates) to test convergence; SETTLE_TIMEOUT (15 s) how long to wait for the estimate to be within 0.3 m before sending the goal; TRIAL_TIMEOUT (240 s of simulation time); ROBOT_NAME (tracer_robot); WORLD (loyola)."""
import json
import math
import os
import subprocess
import sys

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

import dyn
import ghost
import legend
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
        self.through = ActionClient(node, NavigateThroughPoses, "navigate_through_poses")   # trials with a "via" list (make_mission.py)
        self.tf = Buffer()
        TransformListener(self.tf, node)
        self.gt = None
        self.cmd = {"cmd_vel_nav": (0.0, 0.0), "cmd_vel_smoothed": (0.0, 0.0), "cmd_vel": (0.0, 0.0)}
        self.recording, self.rows, self.plans, self.k = False, [], [], 0
        self.mover = None   # dyn.Mover, created by the first trial that has moving obstacles
        self.plan_log = []   # per plan: [t, poses, distance est->plan, distance gt->plan, distance est->first pose, plan stamp]
        node.create_subscription(PoseStamped, "/gt/pose", self.on_gt, 10)
        for name in self.cmd:
            node.create_subscription(Twist, "/" + name, lambda m, k=name: self.cmd.__setitem__(k, (m.linear.x, m.angular.z)), 10)
        node.create_subscription(Path, "/plan", self.on_plan, 10)
        self.odom = (math.nan,) * 3
        node.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.init_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.clear = [node.create_client(ClearEntireCostmap, s) for s in ("/local_costmap/clear_entirely_local_costmap", "/global_costmap/clear_entirely_global_costmap")]
        self.static_map, self.gcm, self.ghost = None, None, [None, None]
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "static_map", m), latched)
        node.create_subscription(OccupancyGrid, "/global_costmap/costmap", self.on_gcm, 1)
        self.ghost_series, self.tail_watch = [], False      # [t, wake cells] at every global costmap publication while a trial (or its tail) is watched
        self.legend_pub = node.create_publisher(Marker, "/legend", latched)   # rviz2 Marker display: what the job runs, and which trial
        self.total = 0
        node.create_timer(1.0 / HZ, self.sample)

    def now(self):
        return self.n.get_clock().now().nanoseconds / 1e9

    def on_gt(self, m):
        o = m.pose.orientation
        self.gt = (m.pose.position.x, m.pose.position.y, quat_to_yaw(o.z, o.w))

    def on_odom(self, m):
        o = m.pose.pose.orientation
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y, quat_to_yaw(o.z, o.w))

    def on_plan(self, m):
        if self.recording:
            xy = path_xy(m)
            self.plans.append((self.now(), xy))
            est, gt = self.estimate(), self.gt
            near = lambda p: min(math.hypot(x - p[0], y - p[1]) for x, y in xy) if p and xy else math.nan
            self.plan_log.append([round(self.now(), 3), len(xy), round(near(est), 3), round(near(gt), 3),
                                  round(math.hypot(xy[0][0] - est[0], xy[0][1] - est[1]), 3) if est and xy else math.nan, round(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, 3)])

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
                          *[round(v, 4) for k in self.cmd for v in self.cmd[k]], *[round(v, 4) for v in self.odom]])

    def walker_xy(self):
        """Current ground-truth position of every moving obstacle (last command sent to the simulator)."""
        if self.mover is None:
            return []
        last = {}
        for e in reversed(self.mover.log[-400:]):
            last.setdefault(e[1], (e[2], e[3]))
        return list(last.values())

    def on_gcm(self, m):
        self.gcm = m
        a = self.static_map
        if (self.recording or self.tail_watch) and a is not None and (a.info.width, a.info.height) == (m.info.width, m.info.height):
            n = ghost.wake_cells(m.data, a.data, m.info.width, m.info.height, (m.info.origin.position.x, m.info.origin.position.y), m.info.resolution, self.walker_xy())
            self.ghost_series.append([round(self.now(), 2), n])
            self.last_wake = n
            self.publish_legend()

    def publish_legend(self, trial_id=None):
        if trial_id is not None:
            self.trial_id, self.last_wake = trial_id, None
        trial_id = getattr(self, "trial_id", None)
        m = self.static_map
        if m is None or trial_id is None:
            return
        k = Marker()
        w, h = m.info.width * m.info.resolution, m.info.height * m.info.resolution
        k.header.frame_id, k.ns, k.id, k.type, k.action = "map", "legend", 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        k.pose.position.x, k.pose.position.y, k.pose.position.z = m.info.origin.position.x + w / 2, m.info.origin.position.y + h - 1.6, 0.5
        k.scale.z = max(0.35, 0.025 * w)
        k.color.r, k.color.g, k.color.b, k.color.a = 1.0, 0.9, 0.1, 1.0
        k.text = "\n".join(legend.legend_lines(os.environ.get("LEGEND"), trial_id, self.k, self.total, self.last_wake))
        self.legend_pub.publish(k)

    def ghost_cells(self):
        """Lethal cells (254) of the global costmap that are free in the static map: obstacles the lidar marked at the wrong place or that nobody cleared. With the moving obstacles parked away,
        anything above a few cells is contamination from the previous trial or from the AMCL offset. None before both grids have arrived or when their sizes differ."""
        a, b = self.static_map, self.gcm
        if a is None or b is None or (a.info.width, a.info.height) != (b.info.width, b.info.height):
            return None
        return int(np.count_nonzero((np.asarray(b.data, np.int16) == 254) & (np.asarray(a.data, np.int16) == 0)))

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
        # Clear again once the AMCL has converged: the clear above ran while the estimate was still offset by up to the initial sigma, and the scans that kept arriving were marked at the wrong
        # place (a ghost wall in the global costmap). The global costmap repaints at its publish rate, so let it do so before the goal is sent.
        before = self.ghost_cells()
        for c in self.clear:
            if c.wait_for_service(timeout_sec=2.0):
                rclpy.spin_until_future_complete(self.n, c.call_async(ClearEntireCostmap.Request()), timeout_sec=3.0)
        self.spin_for(float(os.environ.get("CLEAR_SETTLE", 2.0)))
        self.ghost = [before, self.ghost_cells()]
        return err

    def run_trial(self, trial, tag):
        offset = init_offset(self.k, float(os.environ.get("INIT_OFFSET_M", 0)), float(os.environ.get("INIT_OFFSET_DEG", 0)))
        chained = bool(os.environ.get("CHAIN")) and self.k > 0   # CHAIN=1: no teleport and no AMCL reset after the first trial; the robot continues from where it is
        self.k += 1
        self.publish_legend(trial["id"])
        est = self.estimate()
        init_err = (math.hypot(est[0] - self.gt[0], est[1] - self.gt[1]) if est else math.nan) if chained else self.reset(trial["start"], offset)
        self.publish_legend(trial["id"])      # again: the first call can come before the map has arrived
        walkers = trial.get("dyn", [])
        if walkers:
            self.mover = self.mover or dyn.Mover(os.environ.get("WORLD", "loyola"), self.now)
            self.mover.park(walkers)
            self.mover.log.clear()
        waiting = list(walkers)
        def stamped(g):
            p = PoseStamped()
            p.header.frame_id, p.header.stamp = "map", self.n.get_clock().now().to_msg()
            p.pose.position.x, p.pose.position.y = g[0], g[1]
            p.pose.orientation.z, p.pose.orientation.w = yaw_to_quat(g[2])
            return p
        via = trial.get("via")
        dwell = float(os.environ.get("DWELL", -1))      # DWELL=<s>: visit the stops of a mission one NavigateToPose at a time and wait <s> at each (NavigateThroughPoses only passes near them)
        chain = bool(via) and dwell >= 0
        feedback, stops, status, code, msg, recoveries = [], [], "rejected", -1, "", 0
        self.rows, self.plans, self.plan_log, self.recording = [], [], [], True
        self.ghost_series = []
        t0 = self.now()
        limit = float(os.environ.get("TRIAL_TIMEOUT", 240))
        legs = [*via, trial["goal"]] if chain else [None]
        for i, leg in enumerate(legs):
            if via and not chain:
                goal, client = NavigateThroughPoses.Goal(), self.through
                goal.poses = [stamped(g) for g in [*via, trial["goal"]]]
            else:
                goal, client = NavigateToPose.Goal(), self.client
                goal.pose = stamped(leg or trial["goal"])
            n_fb = len(feedback)
            sent = client.send_goal_async(goal, feedback_callback=lambda f: feedback.append(
                [round(self.now(), 3), f.feedback.number_of_recoveries, round(f.feedback.distance_remaining, 3)]))
            rclpy.spin_until_future_complete(self.n, sent)
            handle = sent.result()
            status, code, msg = "rejected", -1, ""
            if handle.accepted:
                result = handle.get_result_async()
                while not result.done() and self.now() - t0 < limit:
                    rclpy.spin_once(self.n, timeout_sec=0.02)
                    for d in [d for d in waiting if dyn.triggered(self.gt, d["trigger"])]:
                        waiting.remove(d)
                        self.mover.start(d)
                if result.done():
                    r = result.result()
                    status = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_ABORTED: "aborted", GoalStatus.STATUS_CANCELED: "canceled"}.get(r.status, str(r.status))
                    code, msg = int(getattr(r.result, "error_code", -1)), str(getattr(r.result, "error_msg", ""))
                else:
                    status = "timeout"
                    rclpy.spin_until_future_complete(self.n, handle.cancel_goal_async(), timeout_sec=5.0)
            recoveries += feedback[-1][1] if len(feedback) > n_fb else 0
            if chain:
                g = leg if leg else trial["goal"]
                stops.append({"i": i, "status": status, "err_xy": round(math.hypot(self.gt[0] - g[0], self.gt[1] - g[1]), 3), "t": round(self.now() - t0, 3)})
                if status == "succeeded" and i < len(legs) - 1:
                    self.spin_for(dwell)
            if status != "succeeded":
                break
        t1 = self.now()
        self.recording = False
        tail = float(os.environ.get("TAIL_S", 0))      # TAIL_S: keep watching the global costmap that many seconds after the trial ends (does the wake of the obstacle clear?)
        if tail > 0:
            self.tail_watch = True
            self.spin_for(tail)
            self.tail_watch = False
        dyn_log = list(self.mover.log) if walkers else []
        if walkers:
            self.mover.park(walkers)
        self.spin_for(1.0)
        rec = {"dyn": walkers, "dyn_log": dyn_log, "dyn_unstarted": [d["model"] for d in waiting],
"id": trial["id"], "tag": tag, "start": trial["start"], "goal": trial["goal"], "via": via or [], "shortest": trial.get("shortest"), "init_err": init_err, "init_offset": [round(v, 4) for v in offset],
               "status": status, "error_code": code, "error_msg": msg, "t0": round(t0, 3), "t1": round(t1, 3), "duration": round(t1 - t0, 3),
               "recoveries": recoveries, "stops": stops, "ghost_series": self.ghost_series, "ghost_cells": self.ghost if not chained else [None, self.ghost_cells()], "feedback": feedback, "n_plans": len(self.plans),
               "plan_first": self.plans[0][1] if self.plans else [], "plan_last": self.plans[-1][1] if self.plans else [],
               "plan_log": self.plan_log,
               "plan_all": [[round(t, 3), xy[::10]] for t, xy in self.plans],   # every plan, thinned, to see which route the planner chose each time (flip_metrics.py)
               "plan_times": [round(t, 3) for t, _ in self.plans],
               "cols": ["t", "gt_x", "gt_y", "gt_yaw", "est_x", "est_y", "est_yaw", "nav_vx", "nav_wz", "sm_vx", "sm_wz", "cmd_vx", "cmd_wz", "odom_x", "odom_y", "odom_yaw"], "rows": self.rows}
        json.dump(rec, open(os.path.join(self.out, f"trial_{tag}_{trial['id']}.json"), "w"))
        print(f"{tag} {trial['id']}: {status} t={t1 - t0:.1f}s init_err={init_err:.2f} recoveries={rec['recoveries']} code={code}", flush=True)
        return rec


def main(scenario, out_dir, reps=1):
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("trials")
    os.makedirs(out_dir, exist_ok=True)
    r = Runner(node, out_dir)
    trials = json.load(open(scenario))["trials"]
    r.total = len(trials) * int(reps)
    (r.through if any(t.get("via") for t in trials) else r.client).wait_for_server()
    while r.gt is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    for k in range(int(reps)):
        for t in trials:
            r.run_trial(t, f"r{k}")
    if r.mover:
        r.mover.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main(*(sys.argv[1:sys.argv.index("--ros-args")] if "--ros-args" in sys.argv else sys.argv[1:]))
