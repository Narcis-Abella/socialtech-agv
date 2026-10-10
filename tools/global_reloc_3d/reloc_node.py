"""ROS 2 node: boot-time global 3D relocalization -> validated /initialpose for Nav2 AMCL.

IMU gate -> cold FAST-LIO2 (spawned and owned here) -> submap accumulation -> search_v2 worker (FFT search, small_gicp,
frozen gate verify.decide) -> accepted pose -> /initialpose -> AMCL coherence check. No reference pose, no /initialpose input.
The worker is a killable subprocess: the 60 s deadline is enforced by killing it, not by hoping it returns.
Frozen candidate: thresholds live in verify.py/search_v2.py and are not parameters of this node.
"""
import ctypes
import json
import math
import os
import re
import shlex
import signal
import struct
import subprocess
import threading
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'nav_bench'))

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener

import reloc_core as rc
from lio_submap import assemble, cloud_xyz, interpolate, pose as odom_pose, stamp as msg_stamp
from planar_odom import footprint

try:
    from livox_ros_driver2.msg import CustomMsg
except ImportError:      # LiDAR availability is then judged from the body clouds only
    CustomMsg = None

BEST = QoSProfile(depth=500, reliability=ReliabilityPolicy.BEST_EFFORT)


LIO_STALL_S = 3.
LIO_STATES = rc.ACCUMULATE_STATES | rc.SEARCH_STATES | {'LIO_STARTING', 'ACCEPTED', 'HANDOFF', 'LOCALIZED'}
HANDOFF_ODOM_MAX_AGE_S = 1.   # the accepted pose is propagated with FAST-LIO odometry: never publish it from odometry older than this
PROFILE = {}


def profiled(fn):
    """Per-callback count / total / max wall time: shows whether the Python executor is the bottleneck."""
    def wrapper(self, *a, **k):
        t = time.monotonic()
        try:
            return fn(self, *a, **k)
        finally:
            d = time.monotonic() - t
            p = PROFILE.setdefault(fn.__name__, [0, 0., 0.])
            p[0] += 1; p[1] += d; p[2] = max(p[2], d)
    return wrapper


def _die_with_parent():
    ctypes.CDLL('libc.so.6').prctl(1, signal.SIGTERM)   # PR_SET_PDEATHSIG: a killed node never leaves FAST-LIO2 behind


class RelocNode(Node):
    def __init__(self):
        super().__init__('global_reloc')
        P = self.declare_parameter
        self.map_xyz = P('map_xyz', '').value
        self.work = Path(P('work_dir', '/tmp/global_reloc').value)
        self.binary = P('register_binary', str(HERE / 'register_v2')).value
        self.lib_dir = P('lib_dir', '/opt').value
        self.mode = P('mode', 's3').value                          # s3 / s2 = baselines of note 15; s2s3 = stationary S2 first, supervised S3 fallback
        self.s2_exposure = P('s2_exposure_s', rc.S2_EXPOSURE_S).value   # protocol main value 3 s; 1 s and 5 s are predeclared ablations
        # Replay only: recorded bags contain their own motion and no supervisor. On the robot this stays false and a person authorizes.
        self.s3_auto = P('s3_auto_authorize', False).value
        self.sensor_height = P('sensor_height', 0.53).value       # PROVISIONAL, uncalibrated (NV-P); exploratory only
        # Planar mount extrinsic: base_footprint in the levelled body (IMU) frame, [x m, y m, yaw rad]. PROVISIONAL zeros = the IMU
        # origin stands in for the base (nothing measured yet, see 17-calibracion-montaje.md).
        self.base_ext = tuple(float(v) for v in P('base_from_body_xyyaw', [0., 0., 0.]).value)
        self.init_std = P('init_std_m', 0.15).value
        self.init_yaw_std = P('init_yaw_std_deg', 5.).value
        self.handoff_timeout = P('handoff_timeout_s', 30.).value
        self.max_attempts = P('max_attempts', 1).value
        self.publish_pose = P('publish_initialpose', True).value
        self.deadline = P('deadline_s', rc.DEADLINE_S).value      # protocol value; override only for fault-injection tests
        self.acq_limit = P('acquisition_limit_s', rc.ACQUISITION_LIMIT_S).value
        self.fastlio_exe = P('fastlio_exe', '').value               # fault injection only: replaces the FAST-LIO2 executable
        rc.DEADLINE_S = self.deadline
        self.process_start = time.monotonic()
        self.work.mkdir(parents=True, exist_ok=True)
        self.lio = None
        self.search = None
        self.attempt_no = 0
        self.last_odom_stamp = -math.inf
        self.level_q = None
        self.br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)
        self.tfbuf = Buffer(); self.tfl = TransformListener(self.tfbuf, self)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/global_reloc/state', latched)
        self.create_subscription(Imu, '/livox/imu', self.on_imu, BEST)
        if CustomMsg is not None:
            # raw=True: the CustomMsg (~20k points) is never converted to Python objects, only its header stamp is read
            self.create_subscription(CustomMsg, '/livox/lidar', self.on_lidar, BEST, raw=True)
        self.create_subscription(Odometry, '/Odometry', self.on_odom, BEST)
        self.create_subscription(PointCloud2, '/cloud_registered_body', self.on_cloud, BEST)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        self.create_subscription(Bool, '/global_reloc/s3_authorize', self.on_s3_authorize, 10)   # volatile: an old grant is never replayed
        self.create_timer(0.05, self.tick)
        self.new_attempt(min_stamp=-math.inf)

    # ------------------------------------------------------------------ attempts
    def new_attempt(self, min_stamp):
        self.attempt_no += 1
        self.att = rc.Attempt(self.attempt_no)
        self.gate = rc.ImuGate()
        self.acq = rc.Acquisition(self.mode, min_stamp, self.s2_exposure)
        self.stage = 's2' if self.mode == 's2s3' else None      # None = single-stage baseline (file layout and mark names of note 15)
        self.stages = {}
        self.s3_authorized = None
        self.stop_signalled = False
        self.last_odom_mono = None
        self.search = None
        self.streams = {k: rc.StreamStats(k) for k in ('imu', 'lidar', 'odom', 'cloud')}
        self.monitor = rc.HandoffMonitor()
        self.reason = None
        self.details = {}
        self.pose_pub_info = None
        self.search_info = None
        self.latest_odom = None
        self.build = None
        self.last_cloud_mono = None
        self.level_q = None
        self.att.go('WAIT_SENSORS')
        self.announce()

    def announce(self):
        m = String(); m.data = json.dumps({'attempt': self.att.id, 'state': self.att.state, 'reason': self.reason,
                                           'supervision_required': self.att.state == 'S3_SUPERVISION_REQUIRED',
                                           'supervisor_stop_required': self.stop_signalled,
                                           'remaining_s': round(self.att.remaining(), 1)})
        self.state_pub.publish(m)
        self.get_logger().info(f'attempt {self.att.id}: {self.att.state} {self.reason or ""}')

    def go(self, state, reason=None, **info):
        self.reason = reason or self.reason
        self.att.go(state, reason=reason, **info)
        self.announce()
        if state in rc.TERMINAL:
            self.finish()

    # ------------------------------------------------------------------ callbacks
    def mono(self):
        return time.monotonic()

    @profiled
    def on_imu(self, m):
        if self.att.state in rc.TERMINAL:
            return
        t = msg_stamp(m); self.streams['imu'].add(t, self.mono())
        if self.att.t_input is None:
            self.att.input_started(); self.att.mark_once('first_imu')
        a = m.linear_acceleration
        acc = np.array([a.x, a.y, a.z])
        w = m.angular_velocity
        if self.att.state in ('WAIT_SENSORS', 'IMU_CONDITIONING'):
            if self.gate.add(t, [w.x, w.y, w.z], acc) and 'imu_ready' not in self.att.marks:
                self.att.mark_once('imu_ready', header_stamp=t, window_end=self.gate.ready_stamp)
                self.imu_ready_header = t
        elif self.att.state == 'LIO_STARTING' or self.att.state in rc.ACCUMULATE_STATES:
            self.acq.add_imu(t, acc)

    @profiled
    def on_lidar(self, m):
        if self.att.state in rc.TERMINAL:
            return
        if self.att.t_input is None:
            self.att.input_started()
        sec, nsec = struct.unpack_from('<iI' if m[1] == 1 else '>iI', m, 4)   # CDR: 4-byte encapsulation header, then Header.stamp
        self.att.mark_once('first_lidar'); self.streams['lidar'].add(sec + nsec * 1e-9, self.mono())

    @profiled
    def on_odom(self, m):
        t = msg_stamp(m)
        self.last_odom_stamp = max(self.last_odom_stamp, t)
        if self.att.state in LIO_STATES:
            if t > self.acq.min_stamp:
                self.streams['odom'].add(t, self.mono())
            T = odom_pose(m)
            if not np.isfinite(T).all():
                return
            self.acq.add_odom(t, T)           # kept after acceptance too: it propagates the accepted pose to AMCL's stamps
            self.att.mark_once('lio_first_odom')
            self.latest_odom = (t, T)
            self.last_odom_mono = self.mono()
            if self.level_q is not None:
                self.publish_footprint(m)

    @profiled
    def on_cloud(self, m):
        st = self.att.state
        if st != 'LIO_STARTING' and st not in rc.ACCUMULATE_STATES:
            return
        t = msg_stamp(m)
        if t <= self.acq.min_stamp:
            self.acq.refused += 1
            return
        self.streams['cloud'].add(t, self.mono())
        self.last_cloud_mono = self.mono()
        if self.att.t_input is None:
            self.att.input_started()
        if m.header.frame_id != 'body':
            return self.go('ERROR', reason='cloud_frame_not_body')
        xyz = cloud_xyz(m)
        if st == 'LIO_STARTING':
            if len(self.acq.odom) < 2 or not len(xyz):
                return
            self.att.mark('lio_ready', header_stamp=t)
            self.go('S2_ACCUMULATING' if self.mode == 's2s3' else 'ACCUMULATING')
        if not self.acq.clouds:
            self.att.mark_once('first_cloud_kept')
        if not self.acq.add_cloud(t, xyz) and 'acq_stop' not in self.att.marks:
            self.att.mark('acq_stop', reason=self.acq.stop_reason)
            if self.acq.stop_reason in ('radius', 'path'):
                # The node commands nothing. It stops acquiring and tells the external supervisor that the motion envelope is used up.
                self.stop_signalled = True
                self.announce()

    def on_s3_authorize(self, m):
        if m.data and self.att.state == 'S3_SUPERVISION_REQUIRED' and self.s3_authorized is None:
            self.s3_authorized = 'supervisor'

    @profiled
    def on_amcl(self, m):
        if self.att.state != 'HANDOFF' or self.pose_pub_info is None:
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position; q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if t < self.pose_pub_info['stamp']:
            return                                # an AMCL pose from before our /initialpose
        self.att.mark_once('amcl_first_pose_after_init')
        try:
            T_now = self.propagate(t)
        except ValueError:
            return
        c = m.pose.covariance
        self.monitor.add(t, (p.x, p.y, yaw), T_now, cov_trace=c[0] + c[7] + c[35])
        if self.monitor.confirmed:
            self.att.mark('amcl_confirmed')
            self.go('LOCALIZED')

    # ------------------------------------------------------------------ helpers
    def publish_footprint(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = m.header.stamp, 'odom', 'base_footprint'
        tr, ro = t.transform.translation, t.transform.rotation
        x, y, z, qz, qw = footprint((p.x, p.y, p.z), (q.x, q.y, q.z, q.w), self.level_q, self.sensor_height)
        x, y, yaw = rc.compose_planar((x, y, 2 * math.atan2(qz, qw)), self.base_ext)
        tr.x, tr.y, tr.z, ro.z, ro.w = float(x), float(y), float(z), math.sin(yaw / 2), math.cos(yaw / 2)
        self.br.sendTransform(t)

    def publish_level_tf(self):
        s = TransformStamped()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id, s.child_frame_id = 'odom', 'camera_init'
        r = s.transform.rotation
        r.x, r.y, r.z, r.w = [float(v) for v in self.level_q]
        self.static_br.sendTransform(s)

    def propagate(self, t):
        """(x, y, yaw) in map at header stamp t from the accepted pose and FAST-LIO odometry only."""
        T_ci = interpolate(self.acq.odom_for_propagation, t)
        _, xyyaw, _ = rc.handoff_pose(self.T_map_src, self.T_level_local, self.T_ci_first, T_ci)
        return rc.compose_planar(xyyaw, self.base_ext)

    # ------------------------------------------------------------------ processes
    def start_lio(self):
        share = Path(get_package_share_directory('fast_lio'))
        cfg = (share / 'config' / 'mid360.yaml').read_text()
        for key, val in (('pcd_save_en', 'false'), ('map_en', 'false'), ('path_en', 'false'), ('gyr_cov', '1.0'), ('acc_cov', '1.0')):
            cfg = re.sub(rf'^( *){key}:.*$', rf'\g<1>{key}: {val}', cfg, flags=re.M)
        out = self.work / f'attempt{self.att.id}'
        out.mkdir(exist_ok=True)
        (out / 'fastlio.yaml').write_text(cfg)
        cmd = shlex.split(self.fastlio_exe) if self.fastlio_exe not in ('', 'default') else [
            str(Path(get_package_prefix('fast_lio')) / 'lib' / 'fast_lio' / 'fastlio_mapping')]
        sim = ['-p', 'use_sim_time:=true'] if self.get_parameter('use_sim_time').value else []
        self.lio = subprocess.Popen(cmd + ['--ros-args', '--params-file', str(out / 'fastlio.yaml')] + sim,
                                    stdout=open(out / 'fastlio.log', 'w'), stderr=subprocess.STDOUT, start_new_session=True,
                                    preexec_fn=_die_with_parent)
        self.att.mark('lio_spawn', pid=self.lio.pid)

    def stop_lio(self):
        if self.lio and self.lio.poll() is None:
            os.killpg(self.lio.pid, signal.SIGINT)
            try:
                self.lio.wait(5)
            except subprocess.TimeoutExpired:
                os.killpg(self.lio.pid, signal.SIGKILL); self.lio.wait()
        self.lio = None

    def stage_dir(self):
        d = self.work / f'attempt{self.att.id}'
        return d / self.stage if self.stage else d

    def k(self, name):
        """Mark name of the current stage ('s2:acq_end'); unprefixed in the single-stage baselines."""
        return f'{self.stage}:{name}' if self.stage else name

    def start_search(self):
        if self.search is not None and self.search.poll() is None:
            raise RuntimeError('search worker already running')      # never two searches at once
        out = self.stage_dir()
        env = {**os.environ, 'LD_LIBRARY_PATH': f"{self.lib_dir}:{os.environ.get('LD_LIBRARY_PATH', '')}"}
        cmd = [sys.executable, '-B', str(HERE / 'search_v2.py'), '--run', str(out / 'input'), '--out', str(out / 'search'),
               '--binary', self.binary, '--deadline-s', f'{max(1., self.att.remaining() - .5):.3f}']
        self.search = subprocess.Popen(cmd, stdout=open(out / 'search.stdout', 'w'), stderr=subprocess.STDOUT, env=env,
                                       start_new_session=True, preexec_fn=_die_with_parent)
        self.att.mark(self.k('search_spawn'), pid=self.search.pid)

    def kill_search(self):
        if self.search and self.search.poll() is None:
            os.killpg(self.search.pid, signal.SIGKILL); self.search.wait()

    # ------------------------------------------------------------------ the state machine tick
    @profiled
    def tick(self):
        a = self.att
        st = a.state
        if st in rc.TERMINAL:
            return
        try:
            if st in ('HANDOFF',):
                if self.mono() - a.marks['state:HANDOFF'] > self.handoff_timeout:
                    return self.go('HANDOFF_FAILED', reason='amcl_not_confirmed' if self.pose_pub_info is not None else
                                   'odom_stale_at_handoff' if self.odom_age() > HANDOFF_ODOM_MAX_AGE_S else 'amcl_not_available')
                if self.pose_pub_info is not None and (self.lio is None or self.lio.poll() is not None or self.odom_age() > LIO_STALL_S):
                    # pose already handed over, but the odometry AMCL needs is gone: say so now instead of waiting for the timeout
                    return self.go('HANDOFF_FAILED', reason='odometry_lost_after_pose')
                return self.tick_handoff()
            if st == 'ACCEPTED':
                return
            if a.expired():
                self.kill_search()
                return self.go('TIMEOUT', reason=self.timeout_reason())
            if self.mode == 's2s3' and st in rc.ACCUMULATE_STATES and self.acq.stop_reason is None and a.elapsed() >= self.acq_limit:
                self.acq.finish('time_limit')      # protocol: no acquisition past 30 s from first input, whatever the stage (also bounds RAM)
                if self.acq.s2_n is None:
                    self.acq.s2_n = len(self.acq.clouds)
            if st == 'WAIT_SENSORS':
                if self.streams['imu'].n and (CustomMsg is None or 'first_lidar' in a.marks):
                    self.go('IMU_CONDITIONING')
            elif st == 'IMU_CONDITIONING':
                if 'imu_ready' in a.marks:
                    self.start_lio(); self.go('LIO_STARTING')
            elif st == 'LIO_STARTING':
                if self.lio and self.lio.poll() is not None:
                    return self.go('ERROR', reason=f'fastlio_exited_{self.lio.returncode}')
                if self.mono() - a.marks['lio_spawn'] > rc.LIO_READY_MAX_S:
                    return self.go('ERROR', reason='lio_not_ready_5s')
            elif st == 'ACCUMULATING':
                if self.lio.poll() is not None:
                    return self.go('ERROR', reason=f'fastlio_exited_{self.lio.returncode}')
                if self.acq.stop_reason is None and self.last_cloud_mono is not None and self.mono() - self.last_cloud_mono > LIO_STALL_S:
                    return self.go('ERROR', reason='lio_stalled')       # FAST-LIO2 stopped producing clouds mid-acquisition
                if self.acq.stop_reason is None and a.elapsed() >= self.acq_limit:
                    self.acq.finish('time_limit')
                if self.acq.stop_reason is not None:
                    self.build_and_search()
            elif st == 'S2_ACCUMULATING':
                if self.lio_fault():
                    return
                if self.acq.s2_n is None and self.acq.stop_reason is None and a.elapsed() >= self.acq_limit:
                    self.acq.finish('time_limit'); self.acq.s2_n = len(self.acq.clouds)
                if self.acq.s2_n is not None:
                    self.build_and_search()
            elif st == 'S3_SUPERVISION_REQUIRED':
                if self.lio_fault():
                    return
                if not rc.s3_viable(a.elapsed(), self.deadline):
                    return self.go('TIMEOUT', reason='s3_not_authorized_in_budget')
                if self.s3_authorized is None and self.s3_auto:
                    self.s3_authorized = 'auto_replay'
                if self.s3_authorized is not None:
                    a.mark('s3_authorized', source=self.s3_authorized)
                    self.stage = 's3'; self.build = None
                    self.go('S3_ACCUMULATING')
            elif st == 'S3_ACCUMULATING':
                if self.build is None and self.lio_fault():
                    return
                if self.acq.stop_reason is None and a.elapsed() >= self.acq_limit:
                    self.acq.finish('time_limit')
                if self.acq.stop_reason is not None:
                    self.build_and_search()
            elif st in rc.SEARCH_STATES:
                if self.lio_fault():       # without live odometry an accepted pose could not be propagated to the present
                    return
                rcode = self.search.poll()
                if rcode is not None:
                    self.collect_search(rcode)
        except Exception as e:                   # never leave a half-state: a failing step ends the attempt, publishes nothing
            self.get_logger().error(f'{type(e).__name__}: {e}')
            if self.att.state not in rc.TERMINAL:
                self.go('ERROR', reason=f'{type(e).__name__}: {e}'[:200])

    def odom_age(self):
        return math.inf if self.last_odom_mono is None else self.mono() - self.last_odom_mono

    def lio_fault(self):
        """FAST-LIO2 died or went silent while its odometry is still needed (further accumulation, or propagating an accepted pose)."""
        if self.lio is None or self.lio.poll() is not None:
            self.kill_search()
            self.go('ERROR', reason=f'fastlio_exited_{self.lio.returncode if self.lio else None}')
            return True
        if self.odom_age() > LIO_STALL_S:
            self.kill_search()
            self.go('ERROR', reason='lio_stalled')
            return True
        return False

    def timeout_reason(self):
        s = self.att.state
        return {'WAIT_SENSORS': 'sensors_unavailable', 'IMU_CONDITIONING': 'imu_not_stationary',
                'LIO_STARTING': 'lio_not_ready', 'ACCUMULATING': 'acquisition', 'SEARCHING': 'search_deadline',
                'S2_ACCUMULATING': 's2_acquisition', 'S2_SEARCHING': 's2_search_deadline', 'S3_ACCUMULATING': 's3_acquisition',
                'S3_SEARCHING': 's3_search_deadline', 'S3_SUPERVISION_REQUIRED': 's3_not_authorized'}.get(s, s)

    def build_and_search(self):
        """Assemble the submap in a worker thread (callbacks keep draining sensors); polled from tick()."""
        a = self.att
        s2 = self.stage == 's2'
        if self.build is None:
            a.mark(self.k('acq_end'), reason='s2_exposure' if s2 else self.acq.stop_reason)
            clouds = list(self.acq.clouds[:self.acq.s2_n]) if s2 else list(self.acq.clouds)   # S3 = every cloud of this attempt, S2 ones included
            if len(clouds) < 3:
                return self.no_pose('insufficient_submap')
            self.build = {'done': False, 'error': None, 'man': None, 't0': self.mono()}
            args = (list(self.acq.odom), clouds, list(self.acq.imu), clouds[0][0], clouds[-1][0] - clouds[0][0] + .1,
                    self.stage_dir() / 'input')
            self.stage_dir().mkdir(parents=True, exist_ok=True)
            limit = rc.S3_MAX_RADIUS_M if self.mode == 's3' or self.stage == 's3' else None

            def work(b=self.build):
                try:
                    b['man'] = assemble(args[0], args[1], args[2], self.map_xyz, args[3], args[4], args[5],
                                        max_radius=limit, max_path=rc.S3_MAX_PATH_M, source='online')
                except Exception as e:
                    b['error'] = e
                b['done'] = True
            threading.Thread(target=work, daemon=True).start()
            return
        if not self.build['done']:
            return
        if self.build['error'] is not None:
            e = self.build['error']
            return self.no_pose(f'submap_{e}'[:80]) if isinstance(e, ValueError) else self.go('ERROR', reason=f'submap_{type(e).__name__}')
        man = self.build['man']
        a.mark(self.k('submap_built'), build_s=self.mono() - self.build['t0'])
        self.T_ci_first = interpolate(self.acq.odom, man['stamps_s'][0])
        self.T_level_local = np.asarray(man['T_level_local'])
        self.acq.odom_for_propagation = self.acq.odom
        _, _, q = rc.handoff_pose(np.eye(4), self.T_level_local, self.T_ci_first, np.eye(4))
        self.level_q = q
        self.publish_level_tf()
        self.details = {'manifest': {k: man[k] for k in ('start_header_s', 'duration_s', 'last_scan_end_s', 'fit_points', 'check_points',
                                                         's3_stop', 'levelling')}, 'acq_stop_reason': self.acq.stop_reason}
        self.build = None if self.stage else self.build
        self.start_search(); self.go({'s2': 'S2_SEARCHING', 's3': 'S3_SEARCHING', None: 'SEARCHING'}[self.stage])

    def no_pose(self, decision):
        """The current stage ended without an accepted pose. S2: ask for supervised S3 if the same 60 s still allow it. Otherwise REJECTED."""
        a = self.att
        if self.stage != 's2':
            return self.go('REJECTED', reason=decision)
        self.stages['s2'] = {'decision': decision, 'search': self.search_info, 'details': self.details,
                             'odom_radius_m': getattr(self.acq, 's2_radius', None), 'clouds': self.acq.s2_n}
        self.search_info = None
        self.go('S2_INSUFFICIENT', reason='s2_' + decision)
        if not rc.s3_viable(a.elapsed(), self.deadline):
            return self.go('TIMEOUT', reason='s3_budget_insufficient')      # abstain: S3 could not finish inside the same deadline
        self.go('S3_SUPERVISION_REQUIRED')

    def collect_search(self, rcode):
        a = self.att
        a.mark(self.k('search_end'), returncode=rcode)
        path = self.stage_dir() / 'search' / 'search.json'
        if rcode != 0 or not path.exists():
            return self.go('ERROR', reason=f'search_worker_exit_{rcode}')
        s = json.loads(path.read_text())
        self.search_info = {k: s.get(k) for k in ('decision', 'accepted', 'primary_id', 'support_margin', 'confirmation', 'distinct_top',
                                                  'field_seconds', 'correlative_seconds', 'stage_seconds', 'elapsed_search_s',
                                                  'child_cpu_seconds', 'self_cpu_seconds', 'child_maxrss_kib', 'orchestrator_maxrss_kib',
                                                  'valid_candidates', 'truth_used')}
        if s['decision'] == 'timeout':
            return self.go('TIMEOUT', reason='search_deadline')
        if not s['accepted']:
            return self.no_pose(s['decision'])
        if a.expired():
            return self.go('TIMEOUT', reason='accepted_after_deadline')   # an acceptance after the deadline is not a result
        res = json.loads((path.parent / s['primary_id'] / 'result.json').read_text())
        self.T_map_src = np.asarray(res['T_map_lidar_refined'])
        a.mark('accepted', stage=self.stage)
        if self.stage:
            self.reason = None             # the S2 reason stays in metrics['stages']['s2']; a localized S3 result is not "rejected"
        self.go('ACCEPTED')
        self.go('HANDOFF')

    def tick_handoff(self):
        a = self.att
        if self.pose_pub_info is not None or not self.publish_pose:
            return
        if self.init_pub.get_subscription_count() == 0:
            return                                     # AMCL not up yet: wait, bounded by handoff_timeout
        if self.latest_odom is None or self.odom_age() > HANDOFF_ODOM_MAX_AGE_S:
            return                                     # stale odometry: the pose "now" is unknown, publish nothing (bounded wait)
        t, T = self.latest_odom
        _, xyyaw, _ = rc.handoff_pose(self.T_map_src, self.T_level_local, self.T_ci_first, T)
        x, y, yaw = rc.compose_planar(xyyaw, self.base_ext)
        # The only place in this file that publishes a pose, reachable only from HANDOFF (itself only after ACCEPTED).
        assert a.state == 'HANDOFF'
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp.sec = int(t); m.header.stamp.nanosec = int((t - int(t)) * 1e9)
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        c = [0.] * 36
        c[0] = c[7] = self.init_std ** 2
        c[35] = math.radians(self.init_yaw_std) ** 2
        m.pose.covariance = c
        self.init_pub.publish(m)
        self.pose_pub_info = {'stamp': t, 'x': x, 'y': y, 'yaw': yaw}
        a.mark('initialpose_published', **self.pose_pub_info)

    # ------------------------------------------------------------------ outcome
    def finish(self):
        a = self.att
        self.kill_search()
        if a.state not in ('LOCALIZED', 'HANDOFF_FAILED'):
            self.stop_lio()               # nothing to track: a failed attempt leaves no FAST-LIO behind (success keeps it for AMCL)
        (self.work / f'attempt{a.id}').mkdir(parents=True, exist_ok=True)   # early failures happen before any worker dir exists
        try:
            out = self.build_metrics()
            rc.dump(self.work / f'attempt{a.id}' / 'metrics.json', out)
        except Exception:
            import traceback
            (self.work / f'attempt{a.id}' / 'metrics_error.txt').write_text(traceback.format_exc())
            raise
        self.get_logger().info(f'attempt {a.id} ended {a.state} ({self.reason}); metrics written')
        if a.state in ('REJECTED', 'TIMEOUT', 'ERROR', 'HANDOFF_FAILED') and a.id < self.max_attempts:
            self.stop_lio()
            self.kill_search()
            self.new_attempt(min_stamp=self.last_odom_stamp)

    def build_metrics(self):
        a = self.att
        now = self.mono()
        marks = a.marks
        ti = a.t_input if a.t_input is not None else a.t_start
        rel = lambda k: (marks[k] - ti) if k in marks else None
        dur = lambda x, y: (marks[y] - marks[x]) if x in marks and y in marks else None
        K = self.k
        ss = (self.search_info or {}).get('stage_seconds') or {}
        conf = (self.search_info or {}).get('confirmation') or {}
        sb = a.events and next((e for e in a.events if e['name'] == K('submap_built')), {})
        out = {
            'attempt': a.id, 'state': a.state, 'reason': self.reason, 'accepted': a.state in ('ACCEPTED', 'HANDOFF', 'LOCALIZED', 'HANDOFF_FAILED'),
            'origin_note': 'protocol clock origin = first permitted sensor input (first IMU/LiDAR message of the attempt); '
                           'B = localization startup counts from node process start; A (cold boot) is not measured here',
            'mode': self.mode, 'deadline_s': self.deadline, 'truth_used': False,
            'mount': {'sensor_height_m': self.sensor_height, 'base_from_body_xyyaw': self.base_ext, 'init_std_m': self.init_std,
                      'init_yaw_std_deg': self.init_yaw_std, 'status': 'PROVISIONAL_not_measured'},
            'stage_s': {
                'startup_to_first_input_s': marks.get('first_input', a.t_start) - a.t_start,
                'imu_time_s': rel('imu_ready'),                                  # input origin -> stationary window passed
                'lio_time_s': dur('lio_spawn', 'lio_ready'),                     # spawn -> usable odometry + cloud
                'submap_exposure_s': dur('lio_ready', K('acq_end')),                # accumulation, at replay 1x == sensor time
                'submap_build_s': sb.get('build_s') if sb else None,
                'search_worker_wall_s': dur(K('search_spawn'), K('search_end')),
                'map_field_s': (self.search_info or {}).get('field_seconds'),
                'fft_primary_s': (self.search_info or {}).get('correlative_seconds'),
                'fft_confirm_s': conf.get('search_seconds'),
                'gicp_s': ss.get('gicp'), 'score_s': ss.get('score'), 'confirm_total_s': ss.get('confirm_total'),
                'handoff_publish_s': dur('accepted', 'initialpose_published'),
                'handoff_confirm_s': dur('initialpose_published', 'amcl_confirmed'),
            },
            'total_s': {
                'protocol_accept_from_first_input': rel('accepted'),
                'accept_from_lio_ready': (marks['accepted'] - marks['lio_ready']) if 'accepted' in marks and 'lio_ready' in marks else None,
                'accept_from_acq_end': dur(K('acq_end'), 'accepted'),
                'accept_from_node_start': (marks['accepted'] - a.t_start) if 'accepted' in marks else None,
                'localized_from_first_input': rel('amcl_confirmed'),
                'attempt_end_from_first_input': now - ti,
            },
            'marks_rel_first_input_s': {k: v - ti for k, v in marks.items()},
            'streams': {k: s.report() for k, s in self.streams.items()},
            'acquisition': {'clouds': len(self.acq.clouds), 'refused_stale': self.acq.refused, 'stop_reason': self.acq.stop_reason,
                            'imu_samples': len(self.acq.imu), 'span_s': self.acq.span_s(), 's2_clouds': self.acq.s2_n,
                            's2_odom_radius_m': getattr(self.acq, 's2_radius', None),   # own-odometry motion during the S2 exposure
                            'odom_max_radius_m': self.acq.max_radius, 'odom_path_m': self.acq.path_m()},
            # mode s2s3 only. stage_s/search above describe the LAST stage run; the earlier S2 stage is kept here.
            'accepted_stage': self.stage if a.state in ('ACCEPTED', 'HANDOFF', 'LOCALIZED', 'HANDOFF_FAILED') else None,
            'last_stage': self.stage, 'stages': self.stages, 's2_exposure_s': self.s2_exposure,
            's3': {'authorized_by': self.s3_authorized, 'supervisor_stop_signalled': self.stop_signalled,
                   'requested_at_s': rel('state:S3_SUPERVISION_REQUIRED'), 'authorized_at_s': rel('s3_authorized')},
            's2_stage_s': {'exposure_s': dur('lio_ready', 's2:acq_end'), 'build_s': next((e.get('build_s') for e in a.events if e['name'] == 's2:submap_built'), None),
                           'search_worker_wall_s': dur('s2:search_spawn', 's2:search_end'), 'decided_at_s': rel('s2:search_end')},
            'imu_gate': {'evaluations': self.gate.evaluations, 'failed': self.gate.failed_evaluations, 'ready_stamp': self.gate.ready_stamp},
            'search': self.search_info, 'details': self.details, 'initialpose': self.pose_pub_info,
            'handoff_samples': self.monitor.samples[-40:], 'callback_profile': {k: {'n': v[0], 'total_s': v[1], 'max_s': v[2]} for k, v in PROFILE.items()}, 'process_start_to_end_s': now - self.process_start,
        }
        return out

    def shutdown(self):
        if self.att.state not in rc.TERMINAL:      # interrupted attempt: keep what was measured, flagged as such
            try:
                (self.work / f'attempt{self.att.id}').mkdir(parents=True, exist_ok=True)
                out = self.build_metrics(); out['interrupted'] = True
                rc.dump(self.work / f'attempt{self.att.id}' / 'metrics_interrupted.json', out)
            except Exception:
                import traceback; traceback.print_exc()
        self.kill_search()
        if self.att.state not in ('LOCALIZED',):
            self.stop_lio()


def main():
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = RelocNode()
    stop = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.append(1))
    try:
        while not stop and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        print(f'main loop ended: stop={bool(stop)} ok={rclpy.ok()} state={node.att.state}', flush=True)
        node.shutdown()              # writes metrics_interrupted.json if mid-attempt; kills FAST-LIO2 unless it serves a localized AMCL
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
