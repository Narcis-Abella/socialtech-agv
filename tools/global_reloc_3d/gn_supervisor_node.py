"""ROS 2 node: supervisor of gn_loc with on-demand global relocalization (logic in gn_supervisor_core.py, relocalizer in reloc_live.py).

In:  gn_loc/status (String, 1 Hz), gn_loc/need_reloc (Bool, latched), amcl_pose (GN's pose), wheel_topic (Tracer /odom; 'none' = no parked checks),
     /livox/imu, /Odometry, /cloud_registered_body (the robot's FAST-LIO2; clouds are deserialized only while relocalizing), /initialpose (operator),
     <nav action>/_action/status of NavigateToPose and NavigateThroughPoses.
Out: /initialpose (only a relocalized pose), gn_supervisor/state (String JSON, latched), /diagnostics.
Stop: CancelGoal with an empty request (= every goal) on both Nav2 actions, again every recancel_s while navigation is held and an action still
reports an active goal (its status topic is latched: a goal already running when this node starts is seen too).
Fail-safe: an exception in the relocalizer counts as a failed relocalization; any other exception in the timer cancels and goes FAILED. A node that
dies holds nothing: run it with respawn (it restarts in CONFIRMING, which holds until GN confirms).
Nav2 then stops the robot itself (controller_server publishes zero on a cancelled goal; the velocity smoother ramps it down). This node never
publishes a velocity. Goals are not re-sent after a relocalization: the operator or the mission layer does it (state 'MONITOR', reason says why).
"""
import json
import math
import os
import time
import traceback

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Bool, String

import gn_supervisor_core as sc
import reloc_live as rl
from lio_submap import cloud_xyz, pose as odom_pose, stamp

BEST = QoSProfile(depth=500, reliability=ReliabilityPolicy.BEST_EFFORT)
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
ACTIONS = ('navigate_to_pose', 'navigate_through_poses')
LEVEL = {'MONITOR': DiagnosticStatus.OK, 'VERIFYING': DiagnosticStatus.OK, 'CONFIRMING': DiagnosticStatus.WARN,
         'RELOCALIZING': DiagnosticStatus.WARN, 'FAILED': DiagnosticStatus.ERROR}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class SupervisorNode(Node):
    def __init__(self):
        super().__init__('gn_supervisor')
        P = self.declare_parameter
        core = {k: P(k, getattr(sc.Params, k)).value for k in sc.Params.__dataclass_fields__}
        self.core = sc.Supervisor(sc.Params(**core))
        self.reloc = rl.LiveReloc(P('map_xyz', '').value, P('register_binary', '').value, P('lib_dir', '/opt').value,
                                  P('work_dir', '/tmp/gn_supervisor').value, P('search_deadline_s', 60.).value,
                                  tuple(P('base_from_body_xyyaw', [0., 0., 0.]).value))
        self.init_std, self.init_yaw_std = P('init_std_m', .15).value, P('init_yaw_std_deg', 5.).value
        binary, lib = self.get_parameter('register_binary').value, self.get_parameter('lib_dir').value
        missing = [f for f, ok in ((self.reloc.map_xyz, os.path.isfile(self.reloc.map_xyz)), (binary, os.access(binary, os.X_OK)),
                                   (f'{lib}/libsmall_gicp.so', os.path.isfile(f'{lib}/libsmall_gicp.so'))) if not ok]
        if missing:                       # refuse to start: a supervisor that cannot relocalize would only find out at the first stop
            raise SystemExit(f'gn_supervisor: missing {missing} (parameters map_xyz, register_binary, lib_dir)')
        self.mono = time.monotonic
        self.own = None                                   # (pose, mono) of the last /initialpose this node published
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.state_pub = self.create_publisher(String, 'gn_supervisor/state', LATCHED)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.cancel = {a: self.create_client(CancelGoal, f'/{a}/_action/cancel_goal') for a in ACTIONS}
        self.create_subscription(String, 'gn_loc/status', self.on_status, 10)
        self.create_subscription(Bool, 'gn_loc/need_reloc', lambda m: self.core.on_need_reloc(self.mono(), m.data), LATCHED)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_gn_pose, LATCHED)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initialpose, 10)
        wheel = P('wheel_topic', '/odom').value
        if wheel and wheel != 'none':
            self.create_subscription(Odometry, wheel, lambda m: self.core.on_wheel(self.mono(), m.twist.twist.linear.x, m.twist.twist.angular.z), 50)
        self.create_subscription(Imu, '/livox/imu', self.on_imu, BEST)
        self.create_subscription(Odometry, '/Odometry', self.on_odom, BEST)
        self.create_subscription(PointCloud2, '/cloud_registered_body', self.on_cloud, BEST, raw=True)
        for a in ACTIONS:            # rclcpp_action publishes its status transient-local (rcl_action_qos_profile_status_default)
            self.create_subscription(GoalStatusArray, f'/{a}/_action/status', lambda m, a=a: self.on_goals(a, m), LATCHED)
        self.create_timer(.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))   # watchdogs on wall time, whatever /clock does
        self.publish_state('CONFIRMING', 'startup', {})

    # ------------------------------------------------------------------ inputs
    def on_status(self, m):
        f = m.data.split()            # "<STATE> inliers <r> tight <c> scans <n>" (gn_loc_node.cpp status())
        try:
            self.core.on_status(self.mono(), f[0], float(f[2]), float(f[4]), int(f[6]))
        except (IndexError, ValueError):
            self.get_logger().warn(f'unparsed gn_loc/status: {m.data!r}', throttle_duration_sec=10.)

    def on_gn_pose(self, m):
        p = m.pose.pose
        self.core.on_gn_pose(self.mono(), (p.position.x, p.position.y, yaw_of(p.orientation)))

    def on_initialpose(self, m):
        p = m.pose.pose; pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.own and self.mono() - self.own[1] < 2. and np.allclose(pose, self.own[0], atol=1e-4):
            return                    # our own publication coming back
        self.core.on_operator_pose(self.mono(), pose)

    def on_goals(self, action, m):
        self.core.on_goal_status(self.mono(), action, any(s.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING) for s in m.status_list))

    def reloc_failed(self, what):
        """An exception inside the relocalizer: it is stopped and the core sees a failed attempt (retry or FAILED, still holding)."""
        self.get_logger().error(f'relocalizer exception in {what}: {traceback.format_exc()}')
        self.reloc.abort()
        self.core.on_reloc_result(self.mono(), False, None, f'exception_in_{what}')

    def on_imu(self, m):
        if self.reloc.active:
            g, a = m.angular_velocity, m.linear_acceleration
            try:
                self.reloc.add_imu(stamp(m), (g.x, g.y, g.z), (a.x, a.y, a.z))
            except Exception:
                self.reloc_failed('imu')

    def on_odom(self, m):
        if self.reloc.active:
            try:
                self.reloc.add_odom(stamp(m), odom_pose(m))
            except Exception:
                self.reloc_failed('odom')

    def on_cloud(self, raw):
        if self.reloc.stage == 'expose':          # ~20k points at 10 Hz: never deserialized while not needed
            try:
                m = deserialize_message(raw, PointCloud2)
                self.reloc.add_cloud(stamp(m), cloud_xyz(m))
            except Exception:
                self.reloc_failed('cloud')

    # ------------------------------------------------------------------ outputs
    def tick(self):
        now = self.mono()
        try:
            r = self.reloc.poll()
        except Exception:
            r = None
            self.reloc_failed('poll')
        try:
            if r is not None:
                self.get_logger().info(f"relocalization: {r['why']} {r['pose']} {r['info']}")
                self.last_reloc = r
                self.core.on_reloc_result(now, r['ok'], r['pose'], r['why'])
            self.core.tick(now)
            for a in self.core.drain():
                self.act(a)
        except Exception:                         # a bug here must not leave navigation unsupervised: hold for good
            self.get_logger().error(f'supervisor exception: {traceback.format_exc()}')
            self.core.drain()
            self.core.go(now, 'FAILED', 'internal_error')
            self.act(('cancel',))
            self.core.drain()
            self.publish_state('FAILED', 'internal_error', {})

    def act(self, a):
        if a[0] == 'cancel':
            for name, c in self.cancel.items():
                if c.service_is_ready():
                    c.call_async(CancelGoal.Request())    # empty goal id and stamp: cancel every goal of the action
                else:
                    self.get_logger().warn(f'{name} cancel service not available', throttle_duration_sec=5.)
        elif a[0] == 'reloc_start':
            try:
                self.reloc.start()
            except Exception:
                self.reloc_failed('start')
        elif a[0] == 'reloc_abort':
            self.reloc.abort()
        elif a[0] == 'initialpose':
            x, y, yaw = a[1]
            m = PoseWithCovarianceStamped(); m.header.frame_id = 'map'; m.header.stamp = self.get_clock().now().to_msg()
            m.pose.pose.position.x, m.pose.pose.position.y = x, y
            m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            c = [0.] * 36; c[0] = c[7] = self.init_std ** 2; c[35] = math.radians(self.init_yaw_std) ** 2; m.pose.covariance = c
            self.own = (a[1], self.mono())
            self.init_pub.publish(m)
        elif a[0] == 'state':
            self.publish_state(a[1], a[2], getattr(self, 'last_reloc', None) or {})

    def publish_state(self, state, reason, reloc):
        s = String(); s.data = json.dumps({'state': state, 'reason': reason, 'hold': self.core.hold, 'last_reloc': reloc}, default=float)
        self.state_pub.publish(s)
        d = DiagnosticStatus(level=LEVEL.get(state, DiagnosticStatus.WARN), name='gn_supervisor', hardware_id='localization',
                             message=f'{state}: {reason}' + (' (navigation held)' if self.core.hold else ''),
                             values=[KeyValue(key='state', value=state), KeyValue(key='reason', value=reason)])
        arr = DiagnosticArray(); arr.header.stamp = self.get_clock().now().to_msg(); arr.status = [d]
        self.diag_pub.publish(arr)
        (self.get_logger().error if state == 'FAILED' else self.get_logger().info)(f'{state}: {reason}')


def main():
    rclpy.init()
    n = SupervisorNode()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.reloc.abort()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
