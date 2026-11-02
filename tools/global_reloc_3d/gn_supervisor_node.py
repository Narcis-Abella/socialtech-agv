"""ROS 2 node: supervisor of gn_loc with on-demand global relocalization (logic in gn_supervisor_core.py, relocalizer in reloc_live.py).

In:  gn_loc/status (String, 1 Hz), gn_loc/scan_fit (String, per scan: confirms a pose), gn_loc/need_reloc (Bool, latched), amcl_pose (GN's pose), wheel_topic (Tracer /odom; 'none' = no parked checks),
     /livox/imu, /livox/lidar (raw frames, parsed by livox_frames.py: the relocalizer's exposure), /Odometry (only: is the restarted FAST-LIO2 publishing again), /initialpose (operator),
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

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String

import gn_supervisor_core as sc
import livox_frames as lf
import reloc_live as rl
from lio_submap import stamp

BEST = QoSProfile(depth=500, reliability=ReliabilityPolicy.BEST_EFFORT)
LIDAR = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)     # 500 raw frames of 400 kB would be 200 MB in a stall; a frame older than 0.3 s is of no use here
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
ACTIONS = ('navigate_to_pose', 'navigate_through_poses')
LEVEL = {'MONITOR': DiagnosticStatus.OK, 'VERIFYING': DiagnosticStatus.OK, 'CONFIRMING': DiagnosticStatus.WARN,
         'RELOCALIZING': DiagnosticStatus.WARN, 'FAILED': DiagnosticStatus.ERROR}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class SupervisorNode(Node):
    @staticmethod
    def z_band(P):
        """[lo, hi] heights of the sensor in the map frame, from sensor_z (the mounting height over the floor of the map) +- sensor_z_margin; no sensor_z: the whole map box."""
        z = P('sensor_z', -1.).value
        m = P('sensor_z_margin', .5).value
        return (z - m, z + m) if z >= 0 else None

    def __init__(self):
        super().__init__('gn_supervisor')
        P = self.declare_parameter
        core = {k: P(k, getattr(sc.Params, k)).value for k in sc.Params.__dataclass_fields__}
        self.core = sc.Supervisor(sc.Params(**core))
        self.lio_process = P('lio_process', 'fastlio_mapping').value       # comm of the robot's FAST-LIO2 (the launch respawns it)
        self.reloc = rl.LiveReloc(P('map_xyz', '').value, P('register_binary', '').value, P('lib_dir', '/opt').value,
                                  P('work_dir', '/tmp/gn_supervisor').value, P('search_deadline_s', 60.).value,
                                  tuple(P('base_from_body_xyyaw', [0., 0., 0.]).value),
                                  restart=self.restart_lio if P('restart_lio', True).value else None, lio_timeout_s=P('lio_timeout_s', 15.).value,
                                  threads=P('search_threads', 2).value, z_band=self.z_band(P), min_points=P('lidar_min_points', 3000).value, flow_s=P('lidar_flow_s', .5).value)
        # the FAST-LIO2 config the raw frames are filtered with (ops/nav_live.sh: blind 0.1, point_filter_num 2; extrinsic_T of the upstream mid360.yaml)
        self.frame_cfg = dict(blind=P('lidar_blind', .1).value, decimate=P('lidar_decimate', 2).value, extrinsic=tuple(P('lidar_to_body_xyz', [-.011, -.02329, .04412]).value))
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
        self.create_subscription(String, 'gn_loc/scan_fit', self.on_scan_fit, 10)
        self.create_subscription(Bool, 'gn_loc/need_reloc', lambda m: self.core.on_need_reloc(self.mono(), m.data), LATCHED)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_gn_pose, LATCHED)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initialpose, 10)
        wheel = P('wheel_topic', '/odom').value
        if wheel and wheel != 'none':
            self.create_subscription(Odometry, wheel, self.on_wheel, 50)
        self.create_subscription(Imu, '/livox/imu', self.on_imu, BEST, raw=True)
        self.create_subscription(Odometry, '/Odometry', self.on_odom, BEST)
        self.create_subscription(CustomMsg, '/livox/lidar', self.on_lidar, LIDAR, raw=True)
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

    def on_scan_fit(self, m):
        f = m.data.split()            # same text as the status
        try:
            self.core.on_scan_fit(self.mono(), f[0], float(f[2]), float(f[4]))
        except (IndexError, ValueError):
            self.get_logger().warn(f'unparsed gn_loc/scan_fit: {m.data!r}', throttle_duration_sec=10.)

    def on_gn_pose(self, m):
        p = m.pose.pose
        self.core.on_gn_pose(self.mono(), (p.position.x, p.position.y, yaw_of(p.orientation)))

    def on_initialpose(self, m):
        p = m.pose.pose; pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.own and self.mono() - self.own[1] < 5. and sc.same_pose(pose, self.own[0]):
            return                    # our own publication coming back (yaw compared across the +-pi wrap; 5 s: the echo can be late under load)
        self.core.on_operator_pose(self.mono(), pose)

    def on_goals(self, action, m):
        self.core.on_goal_status(self.mono(), action, any(s.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING) for s in m.status_list))

    def restart_lio(self):
        """Called by the relocalizer when its exposure begins (the robot still for 2 s, the lidar seeing): FAST-LIO2 is killed by its exact name and the launch respawns it,
        so the filter the robot goes on with took gravity from a still IMU and starts at z = 0 (the old one keeps whatever a blind period did to it)."""
        n = rl.signal_by_comm(self.lio_process)
        self.get_logger().warn(f'restarting FAST-LIO2 ({self.lio_process}): {n} process(es) signalled' if n else f'no {self.lio_process} process to restart')
        return n > 0

    def reloc_failed(self, what):
        """An exception inside the relocalizer: it is stopped and the core sees a failed attempt (retry or FAILED, still holding)."""
        self.get_logger().error(f'relocalizer exception in {what}: {traceback.format_exc()}')
        self.reloc.abort()
        self.core.on_reloc_result(self.mono(), False, None, f'exception_in_{what}')

    def guarded(self, what, f, *a):
        try:
            f(*a)
        except Exception:
            if self.reloc.active:
                self.reloc_failed(what)
            else:      # a feed that fails while nothing is relocalizing must not count as a failed relocalization
                self.get_logger().error(f'{what} feed failed: {traceback.format_exc()}', throttle_duration_sec=10.)

    def on_wheel(self, m):
        v, w = m.twist.twist.linear.x, m.twist.twist.angular.z
        self.core.on_wheel(self.mono(), v, w)
        self.reloc.add_wheel(self.core.wheel_moving)

    def on_imu(self, raw):        # always: the stillness window of the relocalizer is a sliding one and must be warm when a relocalization starts. Raw: 1 us to unpack vs 44 us to deserialize, 200 times a second
        try:
            t, g, a = lf.imu_fields(bytes(raw))
        except ValueError:
            self.get_logger().warn('unparsed /livox/imu message', throttle_duration_sec=10.)
            return
        self.guarded('imu', self.reloc.add_imu, t, g, a)

    def on_odom(self, m):
        if self.reloc.active:
            self.guarded('odom', self.reloc.add_odom, stamp(m))

    def on_lidar(self, raw):      # always: ~2-4 ms a frame, and it keeps "the lidar sees" current
        try:
            t, valid, xyz = lf.parse(raw, **self.frame_cfg)
        except ValueError:
            self.get_logger().warn('unparsed /livox/lidar frame', throttle_duration_sec=10.)
            return
        self.guarded('lidar', self.reloc.add_lidar, t, xyz, valid)

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
            self.core.drain()
            try:
                self.act(('cancel',))
                self.publish_state('FAILED', 'internal_error', {})
            except Exception:                     # the handler itself must not take the node down
                self.get_logger().error(f'supervisor exception in the handler: {traceback.format_exc()}')
            try:
                self.reloc.abort()                # a search worker must not keep burning CPU (up to 60 s) behind a FAILED supervisor
            except Exception:
                self.get_logger().error(f'abort after the internal error failed: {traceback.format_exc()}')

    def act(self, a):
        if a[0] == 'cancel':
            for name, c in self.cancel.items():
                if c.service_is_ready():
                    c.call_async(CancelGoal.Request())    # empty goal id and stamp: cancel every goal of the action
                else:
                    self.get_logger().warn(f'{name} cancel service not available', throttle_duration_sec=5.)
        elif a[0] == 'reloc_start':
            try:
                self.reloc.start(restart=self.core.hold)        # only a relocalization that holds Nav2 restarts the odometry; the parked check must not
            except Exception:
                self.reloc_failed('start')
        elif a[0] == 'reloc_abort':
            if self.reloc.active:                                # where it was tells a robot that kept moving (imu, restart) from a slow search
                self.get_logger().info(f'relocalization aborted in stage {self.reloc.stage}')
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
        if state == 'FAILED':                     # one severity per call site: rclpy raises if the same line logs at another level
            self.get_logger().error(f'{state}: {reason}')
        else:
            self.get_logger().info(f'{state}: {reason}')


def main():
    rclpy.init()
    n = SupervisorNode()
    # the default executor costs ~1 ms of CPU per wakeup and this node wakes ~280 times a second (IMU 200 Hz, wheels 50 Hz, lidar and scan_fit 10 Hz): the events executor
    # takes the same work with a third of the CPU (laptop rclpy 7.1.12). SUP_EXECUTOR=default goes back.
    if os.environ.get('SUP_EXECUTOR', 'events') == 'events':
        from rclpy.experimental import EventsExecutor
        ex = EventsExecutor()
    else:
        ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(n)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.reloc.abort()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
