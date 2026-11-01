"""Supervisor of the gn_loc localizer, pure logic (no ROS): when GN cannot be trusted, hold navigation, relocalize with the frozen global search of this
directory and resume only once GN confirms a pose the relocalizer agrees with. It never commands motion: its only lever is cancelling Nav2 goals.

Inputs (t = monotonic s): GN status (gn_loc/status, 1 Hz, with its scan counter), GN pose (/amcl_pose), wheel speeds (Tracer /odom |v|, |w|),
relocalizer results, an operator /initialpose, whether each Nav2 action has an active goal. Actions (drain()): ('cancel',) all Nav2 goals; ('reloc_start', reason); ('reloc_abort',);
('initialpose', (x, y, yaw)); ('state', state, reason) on every change.

States: CONFIRMING (start-up, or after a pose: hold until GN fits the map) -> MONITOR -> RELOCALIZING (hold) -> CONFIRMING -> MONITOR; VERIFYING = a
relocalization while parked that does not hold; FAILED holds until an operator pose is confirmed. Thresholds and their evidence: notes of SocialTech,
notes/gn_reloc_supervisor_2026-10-31.md.
"""
import math
from dataclasses import dataclass


@dataclass
class Params:
    lost_stop_s: float = 0.          # GN LOST this long -> stop. 0: at the first LOST status (GN then dead-reckons on the odometry)
    status_stale_s: float = 3.       # no gn_loc/status (1 Hz) this long -> GN is gone: stop, FAILED
    accept_tight: float = .60        # GN's own acceptance of a recovery candidate (gn_core.hpp Params::accept_tight)
    min_inliers: float = .25         # GN's min_inliers
    min_tight: float = .50           # GN's min_tight: below it (or below min_inliers) GN counts the scan as bad
    bad_stop_s: float = 3.           # TRACKING statuses bad this long -> stop. GN itself goes LOST after 3 s of bad scans, except a scan with < 20 valid
                                     # beams (covered lidar, rmin 0.5 m), which stays TRACKING at 0/0 forever. Clean runs: longest bad span 2.2 s (28 runs).
                                     # 3 s by Narcis (2026-11-01): ~0.6 m less blind travel than 4 s at 0.6 m/s; margin over the clean spans is only 0.8 s, so
                                     # log the real distribution on the robot first (H-plan) and raise it if it stops without cause
    scan_stall_n: int = 2            # consecutive statuses with GN's scan counter unchanged -> stop (no /scan reaches GN: its status repeats old values)
    recancel_s: float = 1.           # while held, cancel again this often as long as a goal is still active (a cancel request can be lost)
    confirm_n: int = 3               # consecutive good statuses (1 Hz) after a pose
    confirm_lag_s: float = .5        # statuses this soon after the pose may describe a scan before it
    confirm_timeout_s: float = 15.
    agree_xy: float = .30            # relocalizer vs GN: the wrong-acceptance bounds of the relocalizer's protocol (02-protocolo-evaluacion.md)
    agree_yaw_deg: float = 8.
    reloc_timeout_s: float = 75.     # IMU 2 s + exposure 3 s + search: 7 s (small map) - 23 s (mapa_20261028_01) on the AGX, Nano slower and unmeasured
    max_attempts: int = 2
    stop_check_s: float = 3.         # wheels still turning this long after the cancel -> reported (the supervisor cannot do more)
    wheel_v_still: float = .002      # m/s, rad/s: the stillness test of the L2 parked windows
    wheel_w_still: float = .003
    park_arm_s: float = 3.           # wheels still this long arms the parked-drift check (anchor = GN pose then)
    park_drift_xy: float = .25       # GN pose moving this far from the anchor while parked -> stop and relocalize
    park_drift_yaw_deg: float = 5.
    park_drift_hold_s: float = 1.    # ... for this long with the wheels still (the scan sees a turn ~0.1-1 s before the wheel speed does: 140131)
    wheel_stale_s: float = .5        # no wheel message this long: stillness unknown, the parked check disarms (131206: 7 s gap in /odom)
    verify_period_s: float = 0.      # > 0: also relocalize (without holding) when parked and the last check is this old


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def agree(a, b, xy, yaw_deg):
    return math.hypot(a[0] - b[0], a[1] - b[1]) <= xy and abs(math.degrees(wrap(a[2] - b[2]))) <= yaw_deg


class Supervisor:
    def __init__(self, p=None):
        self.p = p or Params()
        self.actions = []
        self.state, self.reason = 'CONFIRMING', 'startup'
        self.target = None              # the pose GN must settle on (relocalizer's); None = any pose that fits (start-up, operator)
        self.t_enter = None
        self.t_pose = -math.inf         # when the pose under confirmation was given
        self.good = 0
        self.gn_state = None
        self.gn_good = False            # last status: TRACKING with the acceptance consistency
        self.t_status = None
        self.status_new = False
        self.t_lost = None
        self.pose = None                # newest GN pose and when
        self.t_gn_pose = -math.inf
        self.attempts = 0
        self.t_wheels_still = None      # wheels still since; None = moving or no wheel source
        self.wheel_moving = False
        self.anchor = None
        self.t_drift = None             # drift beyond the bounds seen since
        self.t_wheel = None
        self.t_verify = -math.inf
        self.t_cancel = None            # first cancel of the current stop (for the not-stopped report)
        self.t_sent = None              # newest cancel sent
        self.goals = {}                 # Nav2 action -> has an active goal
        self.t_bad = None               # TRACKING but bad since
        self.scans, self.stall = None, 0
        self.timeout = None

    # ------------------------------------------------------------------ outputs
    @property
    def hold(self):
        return self.state not in ('MONITOR', 'VERIFYING')

    def drain(self):
        a, self.actions = self.actions, []
        return a

    def go(self, t, state, reason):
        self.state, self.reason, self.t_enter = state, reason, t
        self.actions.append(('state', state, reason))

    def cancel(self, t):
        self.actions.append(('cancel',))
        self.t_cancel = self.t_sent = t

    # ------------------------------------------------------------------ inputs
    def on_status(self, t, state, inliers, tight, scans=None):
        self.t_status, self.gn_state = t, state
        fresh = scans is None or scans != self.scans          # None: no counter (tests, replays)
        self.stall = 0 if fresh else self.stall + 1
        self.scans = scans
        self.gn_good = state == 'TRACKING' and inliers >= self.p.min_inliers and tight >= self.p.accept_tight and fresh
        if state == 'TRACKING' and (inliers < self.p.min_inliers or tight < self.p.min_tight):
            self.t_bad = self.t_bad if self.t_bad is not None else t
        else:
            self.t_bad = None
        if state == 'LOST':
            self.t_lost = self.t_lost if self.t_lost is not None else t
        else:
            self.t_lost = None
        self.status_new = True          # counted in tick(), after the pose of the same cycle (status and pose arrive independently)
        if not self.gn_good:
            self.anchor, self.t_drift = None, None

    def on_need_reloc(self, t, flag):
        if flag and self.state == 'MONITOR':
            self.stop(t, 'gn_need_reloc')

    def on_gn_pose(self, t, pose):
        self.pose, self.t_gn_pose = tuple(pose), t

    def on_wheel(self, t, v, w):
        still = abs(v) <= self.p.wheel_v_still and abs(w) <= self.p.wheel_w_still
        self.wheel_moving, self.t_wheel = not still, t
        if not still:
            self.t_wheels_still, self.anchor, self.t_drift = None, None, None
        elif self.t_wheels_still is None:
            self.t_wheels_still = t

    def on_goal_status(self, t, action, active):
        self.goals[action] = active
        if active and self.hold:
            self.actions.append(('cancel',)); self.t_sent = t

    def on_operator_pose(self, t, pose):
        if self.state in ('RELOCALIZING', 'VERIFYING'):
            self.actions.append(('reloc_abort',))
        self.confirm(t, None, 'operator_pose')

    def on_reloc_result(self, t, ok, pose, why):
        if self.state not in ('RELOCALIZING', 'VERIFYING'):
            return
        verifying = self.state == 'VERIFYING'
        if verifying:
            self.t_verify = t
        if not ok:
            if verifying:                 # a parked check that found no pose says nothing against GN
                return self.go(t, 'MONITOR', f'verify_no_pose_{why}')
            if self.attempts < self.p.max_attempts:
                return self.start_reloc(t, self.reason)
            return self.go(t, 'FAILED', f'reloc_failed_{why}')
        if self.gn_good and self.pose is not None and agree(self.pose, pose, self.p.agree_xy, self.p.agree_yaw_deg):
            return self.go(t, 'MONITOR', 'verified_gn_agrees')
        if verifying:
            self.cancel(t)                # the independent check contradicts a GN that looked fine: a false lock
        self.actions.append(('initialpose', tuple(pose)))
        self.confirm(t, tuple(pose), 'relocalized_confirming')

    # ------------------------------------------------------------------ transitions
    def stop(self, t, why):
        self.cancel(t)
        self.attempts = 0
        self.start_reloc(t, why)

    def start_reloc(self, t, why, state='RELOCALIZING'):
        self.attempts += 1
        self.timeout = t + self.p.reloc_timeout_s
        self.actions.append(('reloc_start', why))
        self.go(t, state, why)

    def confirm(self, t, target, why):
        self.target, self.t_pose, self.good = target, t, 0
        self.timeout = t + self.p.confirm_timeout_s
        self.go(t, 'CONFIRMING', why)

    def alarm(self, t):
        """Why GN cannot be trusted now, or None."""
        if self.t_lost is not None and t - self.t_lost >= self.p.lost_stop_s:
            return 'gn_lost'
        if self.t_bad is not None and t - self.t_bad >= self.p.bad_stop_s:
            return 'gn_bad_scans'
        if self.stall >= self.p.scan_stall_n and self.gn_state in ('TRACKING', 'LOST'):
            return 'gn_no_scans'
        return None

    def tick(self, t):
        p = self.p
        if self.hold and any(self.goals.values()) and (self.t_sent is None or t - self.t_sent >= p.recancel_s):
            self.actions.append(('cancel',)); self.t_sent = t
        if self.state != 'CONFIRMING':
            self.status_new = False
        if self.t_status is not None and t - self.t_status > p.status_stale_s and self.state != 'FAILED':
            if self.state in ('RELOCALIZING', 'VERIFYING'):
                self.actions.append(('reloc_abort',))
            self.cancel(t)
            return self.go(t, 'FAILED', 'gn_status_stale')
        if self.state == 'CONFIRMING':
            if self.status_new and self.t_status >= self.t_pose + p.confirm_lag_s:
                close = self.target is None or (self.pose is not None and self.t_gn_pose >= self.t_pose + p.confirm_lag_s
                                                and agree(self.pose, self.target, p.agree_xy, p.agree_yaw_deg))
                self.good = self.good + 1 if self.gn_good and close else 0
            self.status_new = False
            if self.good >= p.confirm_n:
                self.target = None
                return self.go(t, 'MONITOR', 'confirmed')
            if self.reason != 'startup' and t > self.timeout:
                return self.go(t, 'FAILED', 'gn_not_confirmed')
            return
        if self.state in ('RELOCALIZING', 'VERIFYING') and t > self.timeout:
            self.actions.append(('reloc_abort',))
            if self.state == 'VERIFYING':
                self.t_verify = t
                return self.go(t, 'MONITOR', 'verify_timeout')
            if self.attempts < p.max_attempts:
                return self.start_reloc(t, self.reason)
            return self.go(t, 'FAILED', 'reloc_timeout')
        if self.state == 'RELOCALIZING' and self.wheel_moving and self.t_cancel is not None and t - self.t_cancel > p.stop_check_s \
                and 'not_stopped' not in self.reason:
            self.reason += '+not_stopped'
            self.actions.append(('state', self.state, self.reason))
        if self.t_wheel is not None and t - self.t_wheel > p.wheel_stale_s:
            self.t_wheels_still, self.anchor, self.t_drift = None, None, None
        why = self.alarm(t)
        if self.state == 'VERIFYING' and why:                          # the parked check does not hold: an alarm meanwhile is a stop
            self.cancel(t)
            self.attempts = 1                                            # the running relocalization counts as the first attempt
            self.timeout = t + p.reloc_timeout_s
            return self.go(t, 'RELOCALIZING', why)
        if self.state != 'MONITOR':
            return
        if why:
            return self.stop(t, why)
        parked = self.t_wheels_still is not None and t - self.t_wheels_still >= p.park_arm_s
        if parked and self.gn_good and self.pose is not None:
            if self.anchor is None:
                self.anchor = self.pose
            elif agree(self.pose, self.anchor, p.park_drift_xy, p.park_drift_yaw_deg):
                self.t_drift = None
            else:
                self.t_drift = t if self.t_drift is None else self.t_drift
                if t - self.t_drift >= p.park_drift_hold_s:
                    return self.stop(t, 'parked_drift')
        if parked and p.verify_period_s > 0 and t - self.t_verify >= p.verify_period_s:
            self.attempts = 0
            self.start_reloc(t, 'parked_verify', 'VERIFYING')
