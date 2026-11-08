"""On-demand global relocalization from the LIVE sensors (for gn_supervisor_node.py), with the frozen pieces of this directory unchanged:
sliding IMU stillness (imu_ready.window_ok over 2 s) and a flowing lidar -> S2 exposure of rc.S2_EXPOSURE_S of the RAW Mid-360 frames (livox_frames.py: FAST-LIO2's own preprocessing,
body frame) -> lio_submap.assemble (IMU-levelled fit/check submap) -> search_v2.py worker (FFT search, small_gicp, verify.decide) -> base pose in map.

The robot is held still, so the exposure needs no odometry: a still body is its own odometry (identity), and the robot's FAST-LIO2 is restarted in PARALLEL with the exposure and
the search (a long blind period leaves it with metres of z drift and degrees of attitude error that nothing corrects; a fresh one takes gravity from the still IMU). Its odometry
is not an input: only whether the new filter publishes again, because gn_loc needs its odom -> base_footprint when it receives the pose. A robot that moves between the start of the
exposure and the result is refused (the IMU in the exposure, the wheels after it): nothing carries a pose along a push when the odometry has just been restarted.

Why not the previous online node (reloc_node.py, removed 2026-11-08): it starts its own cold FAST-LIO2, which publishes /Odometry and /cloud_registered_body next to the robot's, and broadcasts odom -> base_footprint; on the live
stack that would corrupt the odometry chain. Here only the sensors are read and nothing is published. Difference from the evidence of PR #14 (cold FAST-LIO2 body clouds started at rest):
the raw frames carry no undistortion (nothing moves), the density is FAST-LIO2's (point_filter_num) and assemble() levels the submap from the IMU of the exposure, as before. NV on the robot.
"""
import collections
import ctypes
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import imu_ready  # noqa: E402
import reloc_core as rc  # noqa: E402
from lio_submap import assemble  # noqa: E402


def _die_with_parent():
    ctypes.CDLL('libc.so.6').prctl(1, signal.SIGTERM)


def signal_by_comm(name, sig=signal.SIGTERM, proc='/proc', kill=os.kill):
    """Signals every process whose kernel name (comm, 15 characters at most) is exactly `name`; returns how many. Never this process. Exact name and not a
    command-line pattern: `pkill -f` matches the caller's own command line and kills its shell."""
    n = 0
    for d in Path(proc).iterdir():
        if not d.name.isdigit() or int(d.name) == os.getpid():
            continue
        try:
            if (d / 'comm').read_text().strip() != name:
                continue
            kill(int(d.name), sig)
            n += 1
        except OSError:                                                      # gone meanwhile, or not ours to read
            continue
    return n


class Worker:
    """search_v2.py as a killable process group."""
    def __init__(self, binary, lib_dir, run_dir, out_dir, deadline_s, threads=2, z_band=None):
        env = {**os.environ, 'LD_LIBRARY_PATH': f"{lib_dir}:{os.environ.get('LD_LIBRARY_PATH', '')}", 'CORR_SEARCH_LIB': f'{lib_dir}/libcorr_search.so'}      # the library the node checked (lib_dir), not a stray one
        cmd = [sys.executable, '-B', str(HERE / 'search_v2.py'), '--run', str(run_dir), '--out', str(out_dir), '--binary', binary,
               '--deadline-s', f'{deadline_s:.3f}', '--threads', str(threads)]
        if z_band:
            cmd += ['--z-band', str(z_band[0]), str(z_band[1])]
        with open(Path(run_dir).parent / 'search.stdout', 'w') as log:      # the child keeps its own copy of the descriptor
            self.p = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True, preexec_fn=_die_with_parent)

    def poll(self):
        return self.p.poll()

    def kill(self):
        try:
            os.killpg(self.p.pid, signal.SIGKILL)      # also with search_v2 dead (the OOM killer): a register_v2 it forked may live on in the group, beside the next attempt's
        except (ProcessLookupError, PermissionError):  # nobody left in the group, or its number already belongs to another user's
            pass
        try:
            self.p.wait(timeout=2.0)       # SIGKILLed, so normally immediate; in uninterruptible disk wait (the Orin NVMe stalls of 30 s) it would block the supervisor's only thread
        except subprocess.TimeoutExpired:
            pass                           # the attempt is over either way; the zombie is reaped by subprocess at the next Popen (callers drop the worker right after kill)


class StillWindow:
    """Stillness of the last 2 s window, re-evaluated every 0.1 s with imu_ready.window_ok; it does not latch: it follows the robot,
    so a start waits for the stillness of *now*, not of some moment in the past."""
    def __init__(self, step=.1):
        self.t, self.g, self.a, self.step, self.next_eval, self.ok, self.calm, self.stale = collections.deque(), collections.deque(), collections.deque(), step, None, False, False, False

    def add(self, stamp, gyro, acc_g, need=True):
        """need=False keeps the window and its grid but skips the test; the next needed call runs it once, at the newest grid time: the answer the eager window had."""
        if self.t and stamp <= self.t[-1]:
            if stamp >= self.t[-1] - .5:
                return self.ok                 # a duplicate or a slightly late sample: ignored
            self.t.clear(); self.g.clear(); self.a.clear()         # the clock stepped back (driver re-sync, replayed or sim clock): what is kept belongs to another timeline and its answer
            self.next_eval, self.ok, self.calm, self.stale = None, False, False, False   # must not freeze as "still"
        self.t.append(stamp); self.g.append(gyro); self.a.append(acc_g)
        while self.t[0] < stamp - imu_ready.WINDOW_S - 1.:
            self.t.popleft(); self.g.popleft(); self.a.popleft()
        if self.next_eval is None or self.next_eval < self.t[0]:
            self.next_eval = self.t[0] + imu_ready.WINDOW_S if self.next_eval is None else self.next_eval + math.ceil((self.t[0] - self.next_eval) / self.step) * self.step
        while stamp >= self.next_eval:
            self.T, self.next_eval, self.stale = self.next_eval, self.next_eval + self.step, True
        if need and self.stale:                # only the newest grid time counts: the eager loop overwrote ok at each one
            self.stale, T = False, self.T
            t = np.asarray(self.t)
            i, j = np.searchsorted(t, [T - imu_ready.WINDOW_S, T])
            if j - i >= 3:
                g, a, n = np.asarray(self.g)[i:j], np.asarray(self.a)[i:j], imu_ready.expected_samples(t[i:j], imu_ready.WINDOW_S)
                self.ok, self.calm = imu_ready.window_ok(g, a, n), imu_ready.window_ok(g, a, n, imu_ready.MOTION_RESIDUAL_SCALE)   # ok: quiet enough to START; calm: not moved (noise allowed)
            else:
                self.ok = self.calm = False
        return self.ok


class LiveReloc:
    """restart: callable that restarts the robot's FAST-LIO2 and returns whether it did. start(restart=True) calls it once, when the exposure begins.
    min_points: valid returns a frame needs for the lidar to count as seeing (a covered Mid-360 returns none; 20 000 slots, ~13 000 valid in a room). flow_s: how long it must have seen
    before the start (a half-covered window must not start a filter or an exposure). lidar_to_body, blind, decimate: the FAST-LIO2 config the frames are filtered with."""
    ODOM_NEW_AFTER_S = .5      # odometry stamped this soon after the restart can still be the old filter's, in flight

    def __init__(self, map_xyz, binary, lib_dir, work_dir, deadline_s=60., base_ext=(0., 0., 0.), exposure_s=rc.S2_EXPOSURE_S, spawn=None,
                 restart=None, lio_timeout_s=15., threads=2, min_points=3000, flow_s=.5, z_band=None):
        self.map_xyz, self.work, self.deadline_s, self.base_ext, self.exposure_s = map_xyz, Path(work_dir), deadline_s, tuple(base_ext), exposure_s
        self.spawn = spawn or (lambda run, out, dl: Worker(binary, lib_dir, run, out, dl, threads, z_band))
        self.restart_fn, self.lio_timeout_s, self.min_points, self.flow_s = restart, lio_timeout_s, min_points, flow_s
        self.stage, self.n, self.proc = None, 0, None
        self.still = StillWindow()               # the node feeds and polls from boot, before the first start()
        self.flow_since, self.t_lidar, self.t_imu = None, None, None
        self.imu, self.motion, self.clouds, self.pending, self.t_ready, self.info = [], [], [], None, None, {}   # per-run state, set by start(); defined here because the node feeds from boot
        self.want_restart, self.restart_done, self.t_restart, self.odom_new, self.moved_in, self.t_first_imu = False, False, None, False, None, None

    # ------------------------------------------------------------------ control
    def start(self, restart=False):
        self.abort()
        self.n += 1
        self.dir = self.work / f'reloc{self.n:03d}_{time.strftime("%H%M%S")}'
        self.imu, self.motion, self.clouds, self.pending = [], [], [], None
        self.t_ready, self.result, self.info = None, None, {'exposure_restarts': 0}
        self.want_restart, self.restart_done, self.t_restart, self.odom_new = bool(restart and self.restart_fn), False, None, False
        self.moved_in, self.t_first_imu = None, None
        self.t_start = time.monotonic()
        self.stage = 'imu'

    def abort(self):
        if self.proc is not None:
            self.proc.kill()
        self.proc, self.stage = None, None

    @property
    def active(self):
        return self.stage in ('imu', 'expose', 'search')

    # ------------------------------------------------------------------ inputs (the sliding windows run from boot; the rest is only kept while active)
    def seeing(self, t):
        """The lidar has returned at least min_points per frame for flow_s, and a frame came in the last 0.3 s."""
        return self.flow_since is not None and t - self.flow_since >= self.flow_s and self.t_lidar is not None and t - self.t_lidar <= .3

    def add_imu(self, t, gyro, acc_g):
        self.t_imu = t
        still = self.still.add(t, gyro, acc_g, need=self.stage in ('imu', 'search'))   # read while waiting to start and, as the IMU witness of a push, while the search runs
        if self.stage == 'imu':
            self.t_first_imu = t if self.t_first_imu is None else self.t_first_imu
            if still and self.seeing(t):
                self.begin(t)
        elif self.stage == 'expose':
            self.imu.append((t, np.asarray(acc_g, float))); self.motion.append((t, gyro, acc_g))
        elif self.stage == 'search' and not self.still.calm and self.moved_in is None:
            self.moved_in = 'search'                       # latched like the wheels: a shove that ended before the worker returned must still refuse the pose

    def begin(self, t):
        """The robot is still and the lidar sees: the filter restarts now and the exposure starts, side by side."""
        self.t_ready, self.stage, self.imu, self.motion, self.clouds = t, 'expose', [], [], []
        self.info['imu_wait_s'] = round(t - self.t_first_imu, 3)
        if self.want_restart and not self.restart_done:
            self.do_restart(t)

    def do_restart(self, t):
        self.restart_done = True
        try:
            ok = bool(self.restart_fn())
        except Exception as e:
            ok = False
            self.info['lio_restart_error'] = repr(e)
        self.info['lio_restart'] = ok
        if ok:                                                               # not ok: go on with the filter it has, worse than a fresh one but better than no answer
            self.t_restart = t

    def add_lidar(self, t, xyz, valid):
        """One raw frame in the body frame (livox_frames.parse); valid = its count of returns before decimating."""
        self.t_lidar = t
        if valid >= self.min_points:
            self.flow_since = t if self.flow_since is None else self.flow_since
        else:
            self.flow_since = None
            if self.stage == 'expose':                                       # covered meanwhile: the frames so far describe another scene (the hand)
                self.info['exposure_restarts'] += 1
                self.stage, self.clouds, self.imu, self.motion, self.moved_in = 'imu', [], [], [], None      # what the aborted exposure saw (a nudge) is not about the next one
                if self.want_restart and self.restart_done:                  # the filter restarted at that begin() initialised with the lidar covered: it is restarted again at the next one
                    self.restart_done, self.t_restart, self.odom_new = False, None, False
            return
        if self.stage == 'expose' and t >= self.t_ready and xyz is not None and len(xyz):
            self.clouds.append((t, np.asarray(xyz, float)))

    def add_wheel(self, moving):
        if moving and self.stage in ('expose', 'search') and self.moved_in is None:
            self.moved_in = self.stage

    def add_odom(self, t):
        """The robot's FAST-LIO2 is publishing: after a restart, the new one is when the stamp is not the old one's in flight."""
        if self.t_restart is not None and t >= self.t_restart + self.ODOM_NEW_AFTER_S:
            self.odom_new = True

    # ------------------------------------------------------------------ progress (called from the node's timer)
    def poll(self):
        """None while running (or idle); else {'ok', 'pose' (x, y, yaw) or None, 'stamp', 'why', 'info'} once."""
        if self.active and self.t_restart is not None and not self.odom_new and self.t_imu is not None and self.t_imu - self.t_restart > self.lio_timeout_s:
            return self.end(False, 'lio_restart_timeout')                    # the new filter never published: do not wait for the deadline of the node
        if self.stage == 'expose' and self.clouds and self.clouds[-1][0] - self.clouds[0][0] >= self.exposure_s:
            return self.build()
        if self.stage == 'search':
            if self.pending is None:
                rcode = self.proc.poll()
                return self.collect(rcode) if rcode is not None else None
            return self.release()
        return None

    def end(self, ok, why, pose=None, stamp=None):
        if self.proc is not None:
            self.proc.kill()                 # lio_restart_timeout ends a run while the worker still searches: the next start() would not find it, and two searches would share the board
        self.stage, self.proc = 'done', None
        self.info['wall_s'] = round(time.monotonic() - self.t_start, 3)
        self.result = {'ok': ok, 'pose': pose, 'stamp': stamp, 'why': why, 'info': self.info}
        self.dir.mkdir(parents=True, exist_ok=True)
        rc.dump(self.dir / 'result.json', self.result)
        return self.result

    def build(self):
        a, b = self.clouds[0][0], self.clouds[-1][0]
        self.info.update(exposure_s=round(b - a, 3), clouds=len(self.clouds))
        t, g, acc = (np.array(x) for x in zip(*[(t, gy, ac) for t, gy, ac in self.motion])) if self.motion else (np.array([]),) * 3
        if self.moved_in == 'expose' or len(t) < 3 or not imu_ready.window_ok(g, acc, imu_ready.expected_samples(t), imu_ready.MOTION_RESIDUAL_SCALE):
            return self.end(False, 'moved_during_exposure')
        odom = [(a - .1 + .1 * k, np.eye(4)) for k in range(int(round(b - a + .2, 1) * 10) + 2)]   # a still body is its own odometry
        t0 = time.monotonic()
        try:
            self.man = assemble(odom, list(self.clouds), list(self.imu), self.map_xyz, a, b - a + .1, self.dir / 'input', source='raw_livox')
        except ValueError as e:
            return self.end(False, f'submap_{e}'[:80])
        self.info['assemble_s'] = round(time.monotonic() - t0, 3)
        self.clouds, self.imu, self.motion = [], [], []
        self.proc, self.stage = self.spawn(self.dir / 'input', self.dir / 'search', self.deadline_s), 'search'
        return None

    def collect(self, rcode):
        path = self.dir / 'search' / 'search.json'
        if rcode != 0 or not path.exists():
            return self.end(False, f'search_worker_exit_{rcode}')
        s = json.loads(path.read_text())
        self.info.update({k: s.get(k) for k in ('decision', 'support_margin', 'elapsed_search_s', 'child_cpu_seconds', 'self_cpu_seconds',
                                                'child_maxrss_kib', 'orchestrator_maxrss_kib', 'valid_candidates')})
        if not s.get('accepted'):
            return self.end(False, s.get('decision') or 'not_accepted')
        T_src = np.asarray(json.loads((path.parent / s['primary_id'] / 'result.json').read_text())['T_map_lidar_refined'])
        _, xyyaw, _ = rc.handoff_pose(T_src, np.asarray(self.man['T_level_local']), np.eye(4), np.eye(4))
        self.pending = tuple(float(v) for v in rc.compose_planar(xyyaw, self.base_ext))
        return self.release()

    def release(self):
        """The accepted pose goes out once the robot is known not to have moved and, after a restart, the new filter is publishing (gn_loc needs odom -> base_footprint)."""
        if self.moved_in is not None or not self.still.calm:                    # the wheels may be silent (base off, slow creep under their threshold): the IMU window is the other witness
            return self.end(False, 'moved_since_exposure')
        if self.t_restart is not None and not self.odom_new:
            return None
        return self.end(True, 'accepted', self.pending, self.t_imu)
