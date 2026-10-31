"""On-demand global relocalization from the LIVE FAST-LIO2 stream (for gn_supervisor_node.py), with the frozen pieces of this directory unchanged:
rc.ImuGate (2 s stationarity, note 06) -> S2 exposure of rc.S2_EXPOSURE_S of /cloud_registered_body joined to /Odometry -> lio_submap.assemble
(IMU-levelled fit/check submap) -> search_v2.py worker (FFT search, small_gicp, verify.decide) -> base pose in map at the newest odometry stamp.

Why not reloc_node.py: it starts its own cold FAST-LIO2, which publishes /Odometry and /cloud_registered_body next to the robot's, and broadcasts
odom -> base_footprint; on the live stack that would corrupt the odometry chain. Here the robot's own (warm) FAST-LIO2 is the input, so nothing is published.
Difference from the evidence of PR #14 (cold FAST-LIO2 started at rest): the warm odometry's frame is not levelled at start; assemble() levels the
submap from the IMU of the exposure itself, as before. NV on the robot.
"""
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

import reloc_core as rc  # noqa: E402
from lio_submap import assemble, interpolate  # noqa: E402

STILL_M = .05          # odometry motion allowed during the exposure (the robot is held; more means it is pushed or the odometry is not sane)
STILL_YAW_DEG = 2.
ODOM_KEEP_S = 30.


def _die_with_parent():
    ctypes.CDLL('libc.so.6').prctl(1, signal.SIGTERM)


class Worker:
    """search_v2.py as a killable process group."""
    def __init__(self, binary, lib_dir, run_dir, out_dir, deadline_s):
        env = {**os.environ, 'LD_LIBRARY_PATH': f"{lib_dir}:{os.environ.get('LD_LIBRARY_PATH', '')}"}
        cmd = [sys.executable, '-B', str(HERE / 'search_v2.py'), '--run', str(run_dir), '--out', str(out_dir), '--binary', binary,
               '--deadline-s', f'{deadline_s:.3f}']
        self.p = subprocess.Popen(cmd, stdout=open(Path(run_dir).parent / 'search.stdout', 'w'), stderr=subprocess.STDOUT, env=env,
                                  start_new_session=True, preexec_fn=_die_with_parent)

    def poll(self):
        return self.p.poll()

    def kill(self):
        if self.p.poll() is None:
            os.killpg(self.p.pid, signal.SIGKILL); self.p.wait()


class LiveReloc:
    def __init__(self, map_xyz, binary, lib_dir, work_dir, deadline_s=60., base_ext=(0., 0., 0.), exposure_s=rc.S2_EXPOSURE_S, spawn=None):
        self.map_xyz, self.work, self.deadline_s, self.base_ext, self.exposure_s = map_xyz, Path(work_dir), deadline_s, tuple(base_ext), exposure_s
        self.spawn = spawn or (lambda run, out, dl: Worker(binary, lib_dir, run, out, dl))
        self.stage, self.n, self.proc = None, 0, None

    # ------------------------------------------------------------------ control
    def start(self):
        self.abort()
        self.n += 1
        self.dir = self.work / f'reloc{self.n:03d}_{time.strftime("%H%M%S")}'
        self.gate, self.imu, self.odom, self.clouds = rc.ImuGate(), [], [], []
        self.t_ready, self.result, self.info = None, None, {}
        self.t_start = time.monotonic()
        self.stage = 'imu'

    def abort(self):
        if self.proc is not None:
            self.proc.kill()
        self.proc, self.stage = None, None

    @property
    def active(self):
        return self.stage in ('imu', 'expose', 'search')

    # ------------------------------------------------------------------ inputs (only kept while active)
    def add_imu(self, t, gyro, acc_g):
        if self.stage == 'imu' and self.gate.add(t, gyro, acc_g):
            self.t_ready, self.stage = self.gate.ready_stamp, 'expose'
            self.info['imu_wait_s'] = round(time.monotonic() - self.t_start, 3)
        if self.stage == 'expose':
            self.imu.append((t, np.asarray(acc_g, float)))

    def add_odom(self, t, T):
        if self.active and np.isfinite(T).all():
            self.odom.append((t, np.asarray(T, float)))
            while self.odom and self.odom[-1][0] - self.odom[0][0] > ODOM_KEEP_S:
                self.odom.pop(0)

    def add_cloud(self, t, xyz):
        if self.stage == 'expose' and t >= self.t_ready and len(xyz):
            self.clouds.append((t, np.asarray(xyz, float)))

    # ------------------------------------------------------------------ progress (called from the node's timer)
    def poll(self):
        """None while running (or idle); else {'ok', 'pose' (x, y, yaw) or None, 'stamp', 'why', 'info'} once."""
        if self.stage == 'expose' and self.odom:
            done = [c for c in self.clouds if c[0] <= self.odom[-1][0]]     # every cloud of the submap needs an odometry bracket
            if done and done[-1][0] - done[0][0] >= self.exposure_s:
                self.clouds = done
                return self.build()
        if self.stage == 'search':
            rcode = self.proc.poll()
            if rcode is not None:
                return self.collect(rcode)
        return None

    def end(self, ok, why, pose=None, stamp=None):
        self.stage, self.proc = 'done', None
        self.info['wall_s'] = round(time.monotonic() - self.t_start, 3)
        self.result = {'ok': ok, 'pose': pose, 'stamp': stamp, 'why': why, 'info': self.info}
        self.dir.mkdir(parents=True, exist_ok=True)
        rc.dump(self.dir / 'result.json', self.result)
        return self.result

    def build(self):
        a, b = self.clouds[0][0], self.clouds[-1][0]
        self.info['exposure_s'] = round(b - a, 3)
        try:
            T0 = interpolate(self.odom, a)
            moved = [np.linalg.inv(T0) @ interpolate(self.odom, t) for t, _ in self.clouds]
        except ValueError:
            return self.end(False, 'odometry_gap')
        xy = max(float(np.hypot(*M[:2, 3])) for M in moved); yaw = max(abs(math.degrees(math.atan2(M[1, 0], M[0, 0]))) for M in moved)
        self.info.update(exposure_motion_m=round(xy, 4), exposure_motion_deg=round(yaw, 3), clouds=len(self.clouds))
        if xy > STILL_M or yaw > STILL_YAW_DEG:
            return self.end(False, 'moved_during_exposure')
        t0 = time.monotonic()
        try:
            self.man = assemble(list(self.odom), list(self.clouds), list(self.imu), self.map_xyz, a, b - a + .1, self.dir / 'input')
        except ValueError as e:
            return self.end(False, f'submap_{e}'[:80])
        self.info['assemble_s'] = round(time.monotonic() - t0, 3)
        self.T_ci_first = interpolate(self.odom, self.man['stamps_s'][0])
        self.clouds, self.imu = [], []
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
        t, T = self.odom[-1]
        _, xyyaw, _ = rc.handoff_pose(T_src, np.asarray(self.man['T_level_local']), self.T_ci_first, T)
        self.info['moved_since_exposure_m'] = round(float(np.hypot(*(np.linalg.inv(self.T_ci_first) @ T)[:2, 3])), 4)
        return self.end(True, 'accepted', tuple(float(v) for v in rc.compose_planar(xyyaw, self.base_ext)), t)
