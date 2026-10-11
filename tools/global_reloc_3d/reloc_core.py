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
# Planning estimate only (never an acceptance threshold): worst submap build (5.7 s) + worst search worker (15.3 s) observed in
# note 15 (development data) + 1 s. S3 is not started when less than this remains of the single 60 s budget.
S3_COMPUTE_RESERVE_S = 22.
# Phase 2 (robustness). None of these touches the 60 s clock, the search or the acceptance gate.
SENSOR_WAIT_S = 30.       # O1: no IMU and no LiDAR message at all within this time of the attempt start -> TIMEOUT (clock B, not the 60 s)
ODOM_MAX_SPEED_MPS = 3.   # plausibility bound on FAST-LIO2 odometry, about twice the top speed of the base (AgileX figure, not verified here)
ODOM_BAD_STEPS = 3        # consecutive implausible steps (0.3 s at 10 Hz) before the odometry is declared diverged
LOSS_XY_M = .50           # O3: AMCL against the node's own propagated pose after LOCALIZED; 0.5 m / 3 s is the loss rule of pose_eval.py
LOSS_YAW_DEG = 15.        # no earlier definition exists for yaw; provisional
LOSS_HOLD_S = 3.
AMCL_TF_STALL_S = 3.      # AMCL republishes map -> odom on every scan (nav2_amcl laserReceived); silent for this long = AMCL gone
SUPERVISE_GAP_S = 2.      # the supervision tick (20 Hz) itself went unserved this long: the node could not vouch meanwhile
ODOM_KEEP = 1200          # odometry samples kept after acceptance (2 min at 10 Hz): only stamps near "now" are ever propagated
INPUT_JUMP_S = 1.        # a sensor header stamp may not advance this much more than the wall time between two messages
# Map check (map_check.py): the pose that is about to go out, or that is being vouched for, tested against the map with the newest scans.
MAP_CHECK_WINDOW_S = S2_EXPOSURE_S   # the exposure the acceptance rule was validated on
MAP_CHECK_TIMEOUT_S = 8.  # worker limit; a check that does not answer is a failed check
MAP_CHECK_MAX_AGE_S = 3.  # /initialpose carries the pose AT the newest scan of a passed check and goes out at most this long after it
MAP_CHECK_PERIOD_S = 3.   # after LOCALIZED: one check every this long (LOSS_HOLD_S); a failed one is repeated at once
MAP_CHECK_FAILS = 2       # consecutive failed checks before LOCALIZED is revoked (one scan window can be blocked by a person or a door)
PERMIT_PERIOD_S = .1      # navigation permit heartbeat
PERMIT_TTL_S = .5         # the gate closes when the newest permit is older than this (5 missed heartbeats)
PERMIT_NEED = 3           # consecutive fresh `true` heartbeats before the gate opens

TRANSITIONS = {
    'IDLE': {'WAIT_SENSORS', 'ERROR'},
    'WAIT_SENSORS': {'IMU_CONDITIONING', 'TIMEOUT', 'ERROR'},
    'IMU_CONDITIONING': {'LIO_STARTING', 'TIMEOUT', 'ERROR'},
    'LIO_STARTING': {'ACCUMULATING', 'S2_ACCUMULATING', 'TIMEOUT', 'ERROR'},
    # mode s2s3: stationary S2 first, supervised S3 only if S2 gives no accepted pose; one attempt, one clock (protocol 02, line 141)
    'S2_ACCUMULATING': {'S2_SEARCHING', 'S2_INSUFFICIENT', 'TIMEOUT', 'ERROR'},
    'S2_SEARCHING': {'ACCEPTED', 'S2_INSUFFICIENT', 'TIMEOUT', 'ERROR'},
    'S2_INSUFFICIENT': {'S3_SUPERVISION_REQUIRED', 'TIMEOUT', 'ERROR'},
    'S3_SUPERVISION_REQUIRED': {'S3_ACCUMULATING', 'TIMEOUT', 'ERROR'},
    'S3_ACCUMULATING': {'S3_SEARCHING', 'REJECTED', 'TIMEOUT', 'ERROR'},
    'S3_SEARCHING': {'ACCEPTED', 'REJECTED', 'TIMEOUT', 'ERROR'},
    'ACCUMULATING': {'SEARCHING', 'REJECTED', 'TIMEOUT', 'ERROR'},
    'SEARCHING': {'ACCEPTED', 'REJECTED', 'TIMEOUT', 'ERROR'},
    'ACCEPTED': {'HANDOFF', 'ERROR'},
    'HANDOFF': {'LOCALIZED', 'HANDOFF_FAILED', 'ERROR'},
    # LOCALIZED ends the attempt (metrics are written there, as in F1) but is supervised afterwards; the only way out is a
    # revocation. There is no automatic way back: a new localization needs a new attempt started by a person or a restart.
    'LOCALIZED': {'LOCALIZATION_LOST'}, 'LOCALIZATION_LOST': set(),
    'HANDOFF_FAILED': set(), 'REJECTED': set(), 'TIMEOUT': set(), 'ERROR': set(),
}
TERMINAL = {s for s, n in TRANSITIONS.items() if not n} | {'LOCALIZED'}
SEARCH_STATES = {'SEARCHING', 'S2_SEARCHING', 'S3_SEARCHING'}
ACCUMULATE_STATES = {'ACCUMULATING', 'S2_ACCUMULATING', 'S2_SEARCHING', 'S2_INSUFFICIENT', 'S3_SUPERVISION_REQUIRED', 'S3_ACCUMULATING'}


def s3_viable(elapsed, deadline=None, reserve=S3_COMPUTE_RESERVE_S):
    """Can a supervised S3 stage still build, search, verify and decide inside the SAME deadline? The clock is never restarted."""
    return (DEADLINE_S if deadline is None else deadline) - elapsed >= reserve


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

    def no_input(self, limit=None):
        """O1: not one sensor message since the attempt started. The 60 s clock has no origin yet, so this is a separate bound."""
        return self.t_input is None and self.clock() - self.t_start >= (SENSOR_WAIT_S if limit is None else limit)

    def since(self, a, b):
        return self.marks[b] - self.marks[a] if a in self.marks and b in self.marks else None


class StreamStats:
    """Messages received / expected / lost from header stamps (nominal period = running median of the gaps).

    lost_estimate is only valid for evenly stamped streams. The Mid-360 IMU stamps alternate around 4.25 / 5.75 ms (median 4.79 ms,
    mean 5.00 ms), so every long step counts as a lost sample and a complete 200 Hz stream reads as 7-8 % lost (note 16).
    For completeness use rate_hz_stamp (mean rate over header time) against the sensor's specified rate."""

    def __init__(self, name):
        self.name = name
        self.n = 0
        self.lost = 0
        self.gaps = collections.deque(maxlen=200)
        self.last = None
        self.first_stamp = None
        self.first_mono = None
        self.last_mono = None
        self.nonmonotonic = 0
        self.max_gap_s = 0.
        self.loss_events = []      # (seconds since first message, missing count): when losses happened

    def add(self, stamp, mono):
        self.n += 1
        if self.first_stamp is None:
            self.first_stamp = stamp
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
                'rate_hz_stamp': (self.n - 1) / (self.last - self.first_stamp) if self.n > 1 and self.last > self.first_stamp else None,
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
        if self.next_eval < self.t[0]:
            # Stamp jump (driver time sync): every grid time up to the oldest kept sample has an empty window and would be skipped
            # one by one below, blocking the executor (and the deadline) for 55 us per second of jump. Skip them in one step.
            self.next_eval += math.ceil((self.t[0] - self.next_eval) / self.step) * self.step
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

    def __init__(self, mode='s3', min_stamp=-math.inf, s2_exposure=S2_EXPOSURE_S):
        self.mode = mode                # s3 | s2 (baselines) | s2s3 (S2 cut, then keep accumulating under the S3 limits)
        self.min_stamp = min_stamp
        self.s2_exposure = s2_exposure
        self.s2_n = None                # number of leading clouds that form the S2 submap
        self.max_radius = 0.            # own-odometry XY radius from the first cloud (s3/s2s3), recorded, never from a reference
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
        if self.mode == 's2':
            if t - self.clouds[0][0] + .1 >= self.s2_exposure:
                self.s2_n = len(self.clouds)
                self.stop_reason = 's2_exposure'
                return False
            return True
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
            if self.mode == 's2s3' and self.s2_n is None:
                self.s2_n = len(self.clouds)
            return False
        self._path, self._last_xy = path, xy
        self.max_radius = max(self.max_radius, radius)
        if self.mode == 's2s3' and self.s2_n is None and t - self.clouds[0][0] + .1 >= self.s2_exposure:
            self.s2_n = len(self.clouds)
            self.s2_radius = self.max_radius      # own-odometry motion during the S2 exposure (diagnostic: was it really stationary?)
        return True

    def path_m(self):
        return getattr(self, '_path', 0.)

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


def compose_planar(xyyaw, ext):
    """Pose of base_footprint from the pose of the IMU-origin proxy and the planar mount extrinsic (x, y, yaw of the base in the
    levelled body frame). (0, 0, 0) = the proxy itself (PROVISIONAL default: the mount has not been measured)."""
    x, y, w = xyyaw
    ex, ey, ew = ext
    return (x + math.cos(w) * ex - math.sin(w) * ey, y + math.sin(w) * ex + math.cos(w) * ey, w + ew)


def planar_delta(a, b):
    """4x4 rigid motion of the map plane that takes planar pose b (x, y, yaw) onto pose a."""
    def se2(p):
        T = np.eye(4); c, s_ = math.cos(p[2]), math.sin(p[2])
        T[:2, :2] = [[c, -s_], [s_, c]]; T[:2, 3] = p[:2]
        return T
    return se2(a) @ np.linalg.inv(se2(b))


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


class InputIntegrity:
    """Header stamps of one sensor stream while FAST-LIO2 consumes it. Latches on the first stamp that does not increase, or that
    advances more than INPUT_JUMP_S beyond the wall time since the previous message (a gap is not a jump: wall time advances too).

    FAST-LIO2 does not refuse such input: with 20 % of the LiDAR frames swapped it kept publishing odometry and the accepted pose
    went out 14 deg wrong (case h15, note 21). Events are counted always; they only latch while `armed`."""

    def __init__(self, name, jump_s=None):
        self.name = name
        self.jump_s = INPUT_JUMP_S if jump_s is None else jump_s
        self.last = None
        self.backward = 0
        self.jumps = 0
        self.fault = None

    def add(self, stamp, mono, armed):
        if self.last is not None:
            why = None
            if stamp <= self.last[0]:
                self.backward += 1; why = 'backward'
            elif (stamp - self.last[0]) - (mono - self.last[1]) > self.jump_s:
                self.jumps += 1; why = 'jump'
            if why and armed and self.fault is None:
                self.fault = f'input_stamp_{why}_{self.name}'
        self.last = (stamp, mono)

    def report(self):
        return {'backward': self.backward, 'jumps': self.jumps, 'fault': self.fault}


class OdomPlausibility:
    """FAST-LIO2 alive but diverged: implied speed between consecutive odometry stamps above a physical bound, several times in a row.

    Latches. Samples with a non-increasing stamp are ignored here (StreamStats counts them)."""

    def __init__(self, max_speed=None, need=None):
        self.max_speed = ODOM_MAX_SPEED_MPS if max_speed is None else max_speed
        self.need = ODOM_BAD_STEPS if need is None else need
        self.last = None
        self.bad = 0
        self.diverged = False
        self.max_seen = 0.

    def add(self, stamp, xyz):
        xyz = np.asarray(xyz, float)
        if self.last is not None and stamp > self.last[0]:
            v = float(np.linalg.norm(xyz - self.last[1]) / (stamp - self.last[0]))
            self.max_seen = max(self.max_seen, v)
            self.bad = self.bad + 1 if v > self.max_speed else 0
            if self.bad >= self.need:
                self.diverged = True
        if self.last is None or stamp > self.last[0]:
            self.last = (stamp, xyz)
        return self.diverged


class LocalizationMonitor:
    """After LOCALIZED: is AMCL still there and still coherent with the node's own propagated pose? Truth-free, monotonic time.

    It can only revoke. Either estimator may be the wrong one; the answer is the same (stop trusting the pose).
    The propagated pose drifts with FAST-LIO2, so over a long mission this monitor will eventually revoke a healthy AMCL."""

    def __init__(self, t0, xy_m=None, yaw_deg=None, hold_s=None, tf_stall_s=None):
        self.xy_m = LOSS_XY_M if xy_m is None else xy_m
        self.yaw_deg = LOSS_YAW_DEG if yaw_deg is None else yaw_deg
        self.hold_s = LOSS_HOLD_S if hold_s is None else hold_s
        self.tf_stall_s = AMCL_TF_STALL_S if tf_stall_s is None else tf_stall_s
        self.t0 = t0
        self.last_tf = t0
        self.bad_since = None
        self.bad_n = 0
        self.n = 0
        self.max_xy = self.max_yaw = self.max_tf_gap = 0.
        self.trace = collections.deque(maxlen=720)     # one sample per 5 s: an hour of discrepancy history, bounded
        self._trace_t = -math.inf

    def add(self, mono, xy, yaw_deg):
        """One map -> odom transform of AMCL, as its distance at the robot from the node's own pose."""
        self.max_tf_gap = max(self.max_tf_gap, mono - self.last_tf)
        self.last_tf = mono
        self.n += 1
        self.max_xy, self.max_yaw = max(self.max_xy, xy), max(self.max_yaw, yaw_deg)
        if mono - self._trace_t >= 5.:
            self._trace_t = mono
            self.trace.append((round(mono - self.t0, 1), round(xy, 3), round(yaw_deg, 2)))
        if xy > self.xy_m or yaw_deg > self.yaw_deg:
            self.bad_since = mono if self.bad_since is None else self.bad_since
            self.bad_n += 1
        else:
            self.bad_since, self.bad_n = None, 0

    @property
    def doubt(self):
        """AMCL is outside the limits right now. Not yet a loss (that needs hold_s), but no reason to keep the permit meanwhile."""
        return self.bad_since is not None

    def lost(self, mono):
        """Reason for revoking, or None."""
        if mono - self.last_tf > self.tf_stall_s:
            return 'amcl_silent'
        if self.bad_since is not None and self.bad_n >= 3 and mono - self.bad_since >= self.hold_s:
            return 'amcl_incoherent'
        return None

    def report(self):
        return {'tf_received': self.n, 'max_xy_m': self.max_xy, 'max_yaw_deg': self.max_yaw, 'max_tf_gap_s': self.max_tf_gap,
                'limits': {'xy_m': self.xy_m, 'yaw_deg': self.yaw_deg, 'hold_s': self.hold_s, 'tf_stall_s': self.tf_stall_s},
                'trace_t_xy_yaw': list(self.trace)}


class PermitGate:
    """Consumer side of the navigation permit. Fail-closed: open only on PERMIT_NEED consecutive fresh `true` heartbeats from
    exactly one publisher. Any `false`, a gap over the TTL, a dead or duplicated publisher closes it."""

    def __init__(self, ttl=None, need=None):
        self.ttl = PERMIT_TTL_S if ttl is None else ttl
        self.need = PERMIT_NEED if need is None else need
        self.streak = 0
        self.last = None

    def on_permit(self, value, mono):
        if not value or (self.last is not None and mono - self.last > self.ttl):
            self.streak = 0
        if value:
            self.streak += 1
        self.last = mono

    def state(self, mono, publishers=1):
        """(open, reason)"""
        if publishers != 1:
            self.streak = 0
            return False, f'publishers_{publishers}'
        if self.last is None:
            return False, 'no_permit'
        if mono - self.last > self.ttl:
            self.streak = 0
            return False, 'permit_stale'
        if self.streak < self.need:
            return False, 'permit_false' if self.streak == 0 else 'permit_warming'
        return True, 'open'


def trim(rows, keep=None):
    """Bound a growing list in place (amortised): once it holds twice `keep` rows, drop all but the newest `keep`."""
    keep = ODOM_KEEP if keep is None else keep
    if len(rows) >= 2 * keep:
        del rows[:-keep]


def dump(path, obj):
    with open(path, 'w') as f:
        json.dump(obj, f, indent=2, default=float)
        f.write('\n')
