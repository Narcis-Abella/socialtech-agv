"""Runs the trials of a scenario file against Nav2 in the simulation and records raw series for offline analysis (metrics.py).
usage (inside the sim image, with the simulation and Nav2 running): trials.py <scenario.json> <out dir> [reps] --ros-args -p use_sim_time:=true
Each trial: the first one puts the robot at `start` (Gazebo set_pose), resets the localizer and the costmaps and waits until the estimate is within 0.3 m of the ground truth; the next ones start where the previous ended (FAST-LIO2 does not survive a teleport). Then it sends the goal and records until the result.
env: SETTLE_TIMEOUT (15 s) how long the first trial waits for the estimate to be within 0.3 m before sending the goal; CLEAR_SETTLE (2 s) after clearing the costmaps; TRIAL_TIMEOUT (240 s of simulation time); DWELL (<s>: visit the stops of a mission one NavigateToPose at a time and wait <s> at each); TAIL_S (keep watching the global costmap that many seconds after a trial); REC_GCM (record every global costmap publication, see gcm_codec.py); LEGEND (text of the rviz legend); ROBOT_NAME (tracer_robot); WORLD (loyola)."""
import json
import math
import os
import subprocess
import sys

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

import gcm_codec
import ghost
import legend

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
        node.create_subscription(PoseStamped, "/gt/pose", self.on_gt, 10)
        for name in self.cmd:
            node.create_subscription(Twist, "/" + name, lambda m, k=name: self.cmd.__setitem__(k, (m.linear.x, m.angular.z)), 10)
        node.create_subscription(Path, "/plan", self.on_plan, 10)
        self.odom = (math.nan,) * 3
        node.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.init_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.clear = [node.create_client(ClearEntireCostmap, s) for s in ("/local_costmap/clear_entirely_local_costmap", "/global_costmap/clear_entirely_global_costmap")]
        self.static_map, self.gcm = None, None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "static_map", m), latched)
        node.create_subscription(OccupancyGrid, "/global_costmap/costmap", self.on_gcm, 1)
        self.cm_log, self.cm_last = [], None      # [t, action_type, polygon] each time the Collision Monitor changes its mind (0 nothing, 1 stop, 2 slowdown, 3 approach, 4 limit)
        node.create_subscription(CollisionMonitorState, "/collision_monitor_state", self.on_cm, 10)
        self.gcm_frames, self.gcm_meta = [], None      # REC_GCM=1: every global costmap publication of the trial, compressed (see gcm_codec.py), to replay it in rviz2
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
                          *[round(v, 4) for k in self.cmd for v in self.cmd[k]], *[round(v, 4) for v in self.odom]])

    def on_cm(self, m):
        key = (m.action_type, m.polygon_name)
        if (self.recording or self.tail_watch) and key != self.cm_last:
            self.cm_log.append([round(self.now(), 2), int(m.action_type), m.polygon_name])
        self.cm_last = key

    def on_gcm(self, m):
        self.gcm = m
        a = self.static_map
        if (self.recording or self.tail_watch) and a is not None and (a.info.width, a.info.height) == (m.info.width, m.info.height):
            n = ghost.wake_cells(m.data, a.data, m.info.width, m.info.height, (m.info.origin.position.x, m.info.origin.position.y), m.info.resolution, [])
            self.ghost_series.append([round(self.now(), 2), n])
            if os.environ.get("REC_GCM"):
                self.gcm_meta = {"width": m.info.width, "height": m.info.height, "res": m.info.resolution, "origin": [m.info.origin.position.x, m.info.origin.position.y]}
                self.gcm_frames.append([round(self.now(), 2), gcm_codec.encode(m.data)])
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

    def spin_for(self, seconds):
        end = self.now() + seconds
        while self.now() < end:
            rclpy.spin_once(self.n, timeout_sec=0.02)

    def reset(self, start):
        x, y, yaw = start
        z, w = yaw_to_quat(yaw)
        req = f'name: "{os.environ.get("ROBOT_NAME", "tracer_robot")}" position {{x: {x} y: {y} z: 0.15}} orientation {{z: {z} w: {w}}}'
        subprocess.run(["gz", "service", "-s", f"/world/{os.environ.get('WORLD', 'loyola')}/set_pose", "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                        "--timeout", "3000", "--req", req], check=True, capture_output=True)
        self.spin_for(2.0)
        m = PoseWithCovarianceStamped()
        m.header.frame_id, m.header.stamp = "map", self.n.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = yaw_to_quat(yaw)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.1 ** 2      # sigma 0.1 m / 0.1 rad: the click of a person in rviz
        m.pose.covariance[35] = 0.1 ** 2
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
        # Clear again once the localizer has converged: the clear above ran while the estimate was still offset by up to the initial sigma, and the scans that kept arriving were marked at the wrong
        # place (a ghost wall in the global costmap). The global costmap repaints at its publish rate, so let it do so before the goal is sent.
        for c in self.clear:
            if c.wait_for_service(timeout_sec=2.0):
                rclpy.spin_until_future_complete(self.n, c.call_async(ClearEntireCostmap.Request()), timeout_sec=3.0)
        self.spin_for(float(os.environ.get("CLEAR_SETTLE", 2.0)))
        return err

    def run_trial(self, trial, tag):
        chained = self.k > 0     # no teleport and no localizer reset after the first trial; the robot continues from where it is
        self.k += 1
        self.publish_legend(trial["id"])
        est = self.estimate()
        init_err = (math.hypot(est[0] - self.gt[0], est[1] - self.gt[1]) if est else math.nan) if chained else self.reset(trial["start"])
        self.publish_legend(trial["id"])      # again: the first call can come before the map has arrived
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
        self.rows, self.plans, self.recording = [], [], True
        self.ghost_series, self.gcm_frames, self.cm_log = [], [], []
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
        self.spin_for(1.0)
        rec = {"id": trial["id"], "tag": tag, "start": trial["start"], "goal": trial["goal"], "via": via or [], "shortest": trial.get("shortest"), "init_err": init_err,
               "status": status, "error_code": code, "error_msg": msg, "duration": round(t1 - t0, 3), "recoveries": recoveries, "stops": stops,
               "ghost_series": self.ghost_series, "cm_log": self.cm_log, "gcm_meta": self.gcm_meta, "gcm_frames": self.gcm_frames,
               "plan_first": self.plans[0][1] if self.plans else [],
               "plan_all": [[round(t, 3), xy[::10]] for t, xy in self.plans],   # every plan, thinned, to see which route the planner chose each time
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
    rclpy.shutdown()


if __name__ == "__main__":
    main(*(sys.argv[1:sys.argv.index("--ros-args")] if "--ros-args" in sys.argv else sys.argv[1:]))
