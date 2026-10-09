"""ROS-free core of the online global relocalizer: state machine, instrumentation, IMU gate, acquisition, handoff maths.

The node (reloc_node.py) only moves messages in and out; everything that decides lives here so it can be tested without ROS.
Monotonic clock for every duration; message header stamps only order data. No reference pose exists in this module.
"""
import collections
import json
import math
import time

import numpy as np
from scipy.spatial.transform import Rotation

import imu_ready
from lio_submap import interpolate, s3_limit

DEADLINE_S = 60.          # 02-protocolo-evaluacion.md: monotonic wall time from first permitted input
ACQUISITION_LIMIT_S = 30. # S3 acquisition limit, same origin, includes IMU/LIO readiness
LIO_READY_MAX_S = 5.      # protocol: usable odometry + cloud within 5 s of the LIO start
S3_MAX_RADIUS_M = 2.8
S3_MAX_PATH_M = 6.
S2_EXPOSURE_S = 3.

TRANSITIONS = {
    'IDLE': {'WAIT_SENSORS', 'ERROR'},
    'WAIT_SENSORS': {'IMU_CONDITIONING', 'TIMEOUT', 'ERROR'},
    'IMU_CONDITIONING': {'LIO_STARTING', 'TIMEOUT', 'ERROR'},
    'LIO_STARTING': {'ACCUMULATING', 'TIMEOUT', 'ERROR'},
    'ACCUMULATING': {'SEARCHING', 'TIMEOUT', 'ERROR'},
    'SEARCHING': {'ACCEPTED', 'REJECTED', 'TIMEOUT', 'ERROR'},
    'ACCEPTED': {'HANDOFF', 'ERROR'},
    'HANDOFF': {'LOCALIZED', 'HANDOFF_FAILED', 'ERROR'},
    'LOCALIZED': set(), 'HANDOFF_FAILED': set(), 'REJECTED': set(), 'TIMEOUT': set(), 'ERROR': set(),
}
TERMINAL = {s for s, n in TRANSITIONS.items() if not n}


class IllegalTransition(Exception):
    pass


class Attempt:
    """One localization attempt: its own clock origin, state, event log and stage durations. Nothing is shared between attempts."""

    def __init__(self, attempt_id, clock=time.monotonic):
        self.id = attempt_id
        self.clock = clock
        self.state = 'IDLE'
        self.t_start = clock()          # process/attempt start (B: localization startup origin)
        self.t_input = None             # first permitted sensor input (protocol origin of the 60 s)
        self.marks = {}
        self.events = []
        self.error = None

    def mark(self, name, **info):
        t = self.clock()
        self.marks.setdefault(name, t)
        self.events.append({'t': t, 'name': name, **info})
        return t

    def mark_once(self, name, **info):
        if name not in self.marks:
            self.mark(name, **info)

    def go(self, state, **info):
        if state not in TRANSITIONS[self.state]:
            raise IllegalTransition(f'{self.state} -> {state}')
        self.state = state
        self.mark('state:' + state, **info)

    def input_started(self):
        if self.t_input is None:
            self.t_input = self.mark('first_input')

    def elapsed(self):
        """Seconds since the protocol origin (first permitted input); 0 before any input."""
        return 0. if self.t_input is None else self.clock() - self.t_input

    def remaining(self):
        return DEADLINE_S - self.elapsed()

    def expired(self):
        return self.t_input is not None and self.elapsed() >= DEADLINE_S

    def since(self, a, b):
        return self.marks[b] - self.marks[a] if a in self.marks and b in self.marks else None


class StreamStats:
    """Messages received / expected / lost from header stamps (nominal period = running median of the gaps)."""

    def __init__(self, name):
        self.name = name
        self.n = 0
        self.lost = 0
        self.gaps = collections.deque(maxlen=200)
        self.last = None
        self.first_mono = None
        self.last_mono = None
        self.nonmonotonic = 0
        self.max_gap_s = 0.
        self.loss_events = []      # (seconds since first message, missing count): when losses happened

    def add(self, stamp, mono):
        self.n += 1
        self.first_mono = self.first_mono or mono
        self.last_mono = mono
        if self.last is not None:
            gap = stamp - self.last
            if gap <= 0:
                self.nonmonotonic += 1
            else:
                self.max_gap_s = max(self.max_gap_s, gap)
                if len(self.gaps) >= 10:
                    period = float(np.median(self.gaps))
                    missing = int(round(gap / period)) - 1
                    if missing > 0:
                        self.lost += missing
                        if len(self.loss_events) < 300:
                            self.loss_events.append((round(mono - self.first_mono, 2), missing))
                self.gaps.append(gap)
        self.last = stamp

    def report(self):
        span = (self.last_mono - self.first_mono) if self.first_mono else 0.
        return {'received': self.n, 'lost_estimate': self.lost, 'nonmonotonic': self.nonmonotonic, 'max_gap_s': self.max_gap_s,
                'rate_hz_wall': (self.n - 1) / span if span > 0 else None,
                'nominal_period_s': float(np.median(self.gaps)) if self.gaps else None,
                'loss_events_first30': self.loss_events[:30], 'loss_events_n': len(self.loss_events)}


class ImuGate:
    """Streaming version of imu_ready.py: ready at the first 0.1 s grid time whose preceding 2 s window passes the unchanged test."""

    def __init__(self, step=.1):
        self.t = collections.deque(); self.g = collections.deque(); self.a = collections.deque()
        self.step = step
        self.next_eval = None
        self.ready_stamp = None
        self.evaluations = 0
        self.failed_evaluations = 0

    def add(self, stamp, gyro, acc_g):
        if self.ready_stamp is not None:
            return True
        if self.t and stamp <= self.t[-1]:
            return False                 # non-monotonic sample: ignored, counted by StreamStats
        self.t.append(stamp); self.g.append(gyro); self.a.append(acc_g)
        while self.t and self.t[0] < stamp - imu_ready.WINDOW_S - 1.:
            self.t.popleft(); self.g.popleft(); self.a.popleft()
        if self.next_eval is None:
            self.next_eval = self.t[0] + imu_ready.WINDOW_S
        while stamp >= self.next_eval:
            T = self.next_eval
            self.next_eval += self.step
            t = np.asarray(self.t)
            i, j = np.searchsorted(t, [T - imu_ready.WINDOW_S, T])
            if j - i < 3:
                continue
            cadence = float(np.median(np.diff(t[i:j])))
            expected = int(np.ceil(imu_ready.WINDOW_S / cadence))
            self.evaluations += 1
            if imu_ready.window_ok(np.asarray(self.g)[i:j], np.asarray(self.a)[i:j], expected):
                self.ready_stamp = T
                return True
            self.failed_evaluations += 1
        return False


class Acquisition:
    """FAST-LIO2 odometry, body clouds and IMU of ONE attempt (dropped wholesale on restart).

    Messages older than min_stamp (the last stamp the previous FAST-LIO process produced) are refused, so two attempts never mix.
    """

    def __init__(self, mode='s3', min_stamp=-math.inf):
        self.mode = mode
        self.min_stamp = min_stamp
        self.odom, self.clouds, self.imu = [], [], []
        self.stop_reason = None
        self.refused = 0
        self._first = None

    def add_odom(self, t, T):
        if t <= self.min_stamp:
            self.refused += 1
            return
        self.odom.append((t, T))

    def add_imu(self, t, acc):
        if t > self.min_stamp:
            self.imu.append((t, acc))

    def add_cloud(self, t, xyz):
        """Returns True while accumulating, False once a stop condition holds (the cloud that breaks S3 is not kept)."""
        if self.stop_reason:
            return False
        if t <= self.min_stamp:
            self.refused += 1
            return True
        self.clouds.append((t, xyz))
        if self.mode == 's2' and t - self.clouds[0][0] + .1 >= S2_EXPOSURE_S:
            self.stop_reason = 's2_exposure'
            return False
        if self.mode == 's3':
            # Incremental form of lio_submap.s3_limit (same radius/path definitions, first violation excluded), O(log n) per cloud.
            try:
                if self._first is None:
                    self._first_inv = np.linalg.inv(interpolate(self.odom, t)); self._first = True
                    self._xy0 = np.zeros(2); self._last_xy = self._xy0; self._path = 0.
                    return True
                rel = self._first_inv @ interpolate(self.odom, t)
            except ValueError:
                return True               # odometry bracket not yet available for the newest cloud
            xy = rel[:2, 3]
            path = self._path + float(np.linalg.norm(xy - self._last_xy))
            radius = float(np.linalg.norm(xy - self._xy0))
            if radius > S3_MAX_RADIUS_M or path > S3_MAX_PATH_M:
                self.stop_reason = 'radius' if radius > S3_MAX_RADIUS_M else 'path'
                self.clouds = self.clouds[:-1]
                return False
            self._path, self._last_xy = path, xy
        return True

    def start_stamp(self):
        return self.clouds[0][0] if self.clouds else None

    def span_s(self):
        return self.clouds[-1][0] - self.clouds[0][0] + .1 if self.clouds else 0.

    def finish(self, reason):
        self.stop_reason = self.stop_reason or reason


def handoff_pose(T_map_src, T_level_local, T_ci_first, T_ci_now):
    """Accepted map<-source pose -> (x, y, yaw) of the body proxy in map NOW, and the odom levelling quaternion.

    Same algebra as handoff_pose.py (validated on replay): T_map_ci = T_map_src @ T_level_local @ inv(T_ci_first);
    the footprint proxy is the IMU origin (sensor->base extrinsic uncalibrated: NV-P).
    """
    T_src_ci = np.asarray(T_level_local) @ np.linalg.inv(T_ci_first)
    T_map_ci = np.asarray(T_map_src) @ T_src_ci
    T_now = T_map_ci @ T_ci_now
    yaw = math.atan2(T_now[1, 0], T_now[0, 0])
    q = Rotation.from_matrix(T_src_ci[:3, :3]).as_quat()
    return T_map_ci, (float(T_now[0, 3]), float(T_now[1, 3]), yaw), q


def yaw_diff_deg(a, b):
    d = a - b
    return abs(math.degrees(math.atan2(math.sin(d), math.cos(d))))


class HandoffMonitor:
    """AMCL coherence against the relocalizer's own propagated pose (FAST-LIO odometry), truth-free.

    Confirmed when `need` consecutive /amcl_pose updates stay within xy_m / yaw_deg of the propagated pose and span >= hold_s.
    """

    def __init__(self, xy_m=.30, yaw_deg=8., hold_s=5., need=3):
        self.xy_m, self.yaw_deg, self.hold_s, self.need = xy_m, yaw_deg, hold_s, need
        self.run = []
        self.samples = []
        self.confirmed = False

    def add(self, stamp, amcl_xyyaw, own_xyyaw, cov_trace=None):
        xy = math.hypot(amcl_xyyaw[0] - own_xyyaw[0], amcl_xyyaw[1] - own_xyyaw[1])
        dy = yaw_diff_deg(amcl_xyyaw[2], own_xyyaw[2])
        ok = xy <= self.xy_m and dy <= self.yaw_deg
        self.samples.append({'stamp': stamp, 'xy_m': xy, 'yaw_deg': dy, 'ok': ok, 'cov_trace': cov_trace})
        self.run = self.run + [stamp] if ok else []
        if len(self.run) >= self.need and self.run[-1] - self.run[0] >= self.hold_s:
            self.confirmed = True
        return self.confirmed


def dump(path, obj):
    with open(path, 'w') as f:
        json.dump(obj, f, indent=2, default=float)
        f.write('\n')
