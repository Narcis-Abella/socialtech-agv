import subprocess
import time
import json
import math
import os
import tempfile
import unittest
from pathlib import Path

import numpy as np

from types import SimpleNamespace

import reloc_live as rl


class FakeProc:
    """Stands in for the search_v2.py worker: writes its decision on the first poll."""
    def __init__(self, out, decision, T, polls=200):
        self.out, self.decision, self.T, self.killed, self.polls = Path(out), decision, T, False, polls   # 200 polls = 1 s of IMU samples

    def poll(self):
        self.polls -= 1
        if self.polls > 0:
            return None
        self.out.mkdir(parents=True, exist_ok=True)
        (self.out / 'search.json').write_text(json.dumps({'decision': self.decision, 'accepted': self.decision == 'accepted', 'primary_id': 'c0',
                                                          'elapsed_search_s': 1.5, 'child_cpu_seconds': 3., 'child_maxrss_kib': 70000,
                                                          'orchestrator_maxrss_kib': 50000, 'support_margin': .3}))
        (self.out / 'c0').mkdir(exist_ok=True)
        (self.out / 'c0' / 'result.json').write_text(json.dumps({'T_map_lidar_refined': self.T.tolist()}))
        return 0

    def kill(self):
        self.killed = True


def planar(x, y, yaw):
    T = np.eye(4); T[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]; T[:2, 3] = [x, y]
    return T


class LiveRelocBase(unittest.TestCase):
    """The relocalizer sees the IMU, the raw lidar (body frame, with the count of valid returns) and, as flags, the wheels and the odometry of the robot's FAST-LIO2 (is the new
    filter publishing?). The odometry of the old filter is not an input any more."""
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); d = Path(self.tmp.name)
        rng = np.random.default_rng(0)
        np.savetxt(d / 'map.xyz', rng.uniform(-5, 5, (500, 3)))
        self.cloud = rng.uniform(-4, 4, (400, 3))
        self.map, self.work = d / 'map.xyz', d / 'work'
        self.procs, self.restarts = [], []

    def tearDown(self):
        self.tmp.cleanup()

    def make(self, decision='accepted', T=None, restart=None, **kw):
        def spawn(run_dir, out_dir, deadline_s):
            p = FakeProc(out_dir, decision, planar(3., 1., .5) if T is None else T); self.procs.append(p); return p
        return rl.LiveReloc(str(self.map), 'unused', '/opt', str(self.work), spawn=spawn, restart=restart, **kw)

    def make_restart(self, ok=True, **kw):
        def restart():
            self.restarts.append(1); return ok
        return self.make(restart=restart, **kw)

    def feed(self, r, t0, t1, gyro=lambda t: (0., 0., 0.), points=lambda t: 13000, wheel=lambda t: False, odom=lambda t: True):
        """IMU at 200 Hz (accelerometer in g, as the Livox driver), lidar frames at 10 Hz, the wheel flag at 50 Hz, the robot's odometry at 10 Hz while odom(t)."""
        for k in range(int(round((t1 - t0) * 200))):
            t = t0 + k / 200.
            r.add_imu(t, gyro(t), (0., 0., 1.))
            if k % 20 == 0:
                r.add_lidar(t + .001, self.cloud if r.stage == 'expose' else None, points(t))      # the node parses the whole cloud only while exposing
                if odom(t):
                    r.add_odom(t)
            if k % 4 == 0:
                r.add_wheel(wheel(t))
            res = r.poll()
            if res is not None:
                return res
        return None


class LiveRelocTest(LiveRelocBase):
    def test_still_robot_gives_the_pose_of_the_accepted_search(self):
        r = self.make(); r.start()
        res = self.feed(r, 0., 10.)
        self.assertIsNotNone(res)
        self.assertTrue(res['ok'], res)
        x, y, yaw = res['pose']   # still robot, levelled frame = first body frame: the base pose is the accepted transform itself
        self.assertAlmostEqual(x, 3., 6); self.assertAlmostEqual(y, 1., 6); self.assertAlmostEqual(yaw, .5, 6)
        self.assertEqual(res['why'], 'accepted')
        self.assertGreaterEqual(res['info']['exposure_s'], 2.9)

    def test_inputs_and_polls_before_the_first_start_do_nothing(self):
        # the node feeds and polls from boot; the first start() comes 8 s later (start-up localization) or at the first loss
        r = self.make()
        self.assertIsNone(self.feed(r, 0., 1.))
        self.assertIsNone(r.poll())
        self.assertFalse(r.active)

    def test_it_starts_as_soon_as_the_robot_has_been_still_for_two_seconds_with_the_lidar_flowing(self):
        r = self.make(); r.start()
        self.feed(r, 0., 2.3)
        self.assertEqual(r.stage, 'expose')
        self.assertLess(r.info['imu_wait_s'], 3.)

    def test_no_search_while_the_imu_says_moving(self):
        r = self.make(); r.start()
        self.assertIsNone(self.feed(r, 0., 6., gyro=lambda t: (0., 0., .3)))
        self.assertEqual(self.procs, [])
        self.assertEqual(r.stage, 'imu')

    def test_motion_during_the_exposure_is_refused(self):
        r = self.make(); r.start()
        res = self.feed(r, 0., 10., gyro=lambda t: (0., 0., .3) if t > 3. else (0., 0., 0.))     # the gate passed at 2 s, then it is turned
        self.assertFalse(res['ok'])
        self.assertEqual(res['why'], 'moved_during_exposure')
        self.assertEqual(self.procs, [])

    def test_wheels_turning_during_the_exposure_are_refused(self):
        r = self.make(); r.start()
        res = self.feed(r, 0., 10., wheel=lambda t: t > 3.)       # a push the IMU is too quiet to see
        self.assertFalse(res['ok'])
        self.assertEqual(res['why'], 'moved_during_exposure')

    def test_wheels_turning_while_it_searches_discard_the_pose(self):
        # nothing carries the exposure's pose along a push any more (the odometry of a restarted filter cannot): a retry, not a stale pose
        r = self.make(); r.start()
        res = self.feed(r, 0., 12., wheel=lambda t: t > 5.5)
        self.assertFalse(res['ok'])
        self.assertEqual(res['why'], 'moved_since_exposure')

    def test_a_rejected_search_reports_its_decision(self):
        r = self.make('ambiguous_no_alternative_margin'); r.start()
        res = self.feed(r, 0., 10.)
        self.assertFalse(res['ok'])
        self.assertEqual(res['why'], 'ambiguous_no_alternative_margin')

    def test_abort_kills_the_worker_and_a_new_start_is_clean(self):
        class Hang(FakeProc):
            def poll(self): return None
        procs = []
        r = rl.LiveReloc(str(self.map), 'unused', '/opt', str(self.work), spawn=lambda *a: procs.append(Hang(a[1], '', None)) or procs[-1])
        r.start(); self.assertIsNone(self.feed(r, 0., 10.))
        self.assertEqual(len(procs), 1)
        r.abort()
        self.assertTrue(procs[0].killed)
        self.assertIsNone(r.poll())
        r.start(); self.assertEqual(r.stage, 'imu')


class CoveredLidarTest(LiveRelocBase):
    """A covered lidar returns (almost) nothing: the exposure would be empty and a new FAST-LIO2 would get no points."""
    def test_it_waits_for_the_lidar_and_starts_half_a_second_after_it_flows_again(self):
        r = self.make(); r.start()
        self.feed(r, 0., 6., points=lambda t: 0)                         # covered, still: the 2 s of stillness are long over
        self.assertEqual(r.stage, 'imu')
        self.assertEqual(self.procs, [])
        self.feed(r, 6., 6.3, points=lambda t: 13000)
        self.assertEqual(r.stage, 'imu')                                 # flow must last flow_s (0.5 s)
        self.feed(r, 6.3, 6.7, points=lambda t: 13000)
        self.assertEqual(r.stage, 'expose')
        self.assertLess(r.t_ready, 6.8)                                  # not a second 2 s of stillness after the uncover: the sliding window has been still all along

    def test_a_robot_pushed_while_covered_must_be_still_again_before_it_starts(self):
        r = self.make(); r.start()
        self.feed(r, 0., 4., points=lambda t: 0)
        self.feed(r, 4., 6., points=lambda t: 13000, gyro=lambda t: (0., 0., .3))
        self.assertEqual(r.stage, 'imu')
        self.feed(r, 6., 7.5, points=lambda t: 13000)
        self.assertEqual(r.stage, 'imu')                                 # only 1.5 s of stillness since the push
        self.feed(r, 7.5, 8.5, points=lambda t: 13000)
        self.assertEqual(r.stage, 'expose')

    def test_a_cover_during_the_exposure_starts_it_again_after_the_flow_returns(self):
        r = self.make(); r.start()
        self.feed(r, 0., 3.5)
        self.assertEqual(r.stage, 'expose')
        self.feed(r, 3.5, 4.5, points=lambda t: 0)
        self.assertEqual(r.stage, 'imu')
        self.assertEqual(r.clouds, [])
        res = self.feed(r, 4.5, 14.)
        self.assertTrue(res['ok'], res)
        self.assertGreaterEqual(res['info']['exposure_s'], 2.9)
        self.assertEqual(r.info['exposure_restarts'], 1)

    def test_a_parked_check_does_not_start_while_covered_either(self):
        r = self.make(); r.start()
        self.assertIsNone(self.feed(r, 0., 8., points=lambda t: 0))
        self.assertEqual(self.procs, [])


class RestartTest(LiveRelocBase):
    """The robot's FAST-LIO2 is restarted when the exposure starts (a blind period leaves it with metres of z drift and degrees of attitude error that nothing corrects) and
    comes back while the exposure and the search run: the pose is released when it publishes again, so gn_loc has its odom -> base_footprint."""
    def test_the_restart_is_called_once_at_the_start_of_the_exposure_and_does_not_delay_it(self):
        r = self.make_restart(); r.start(restart=True)
        res = self.feed(r, 0., 10., odom=lambda t: t < 2.2 or t > 4.)    # the old filter is killed at ~2 s; the new one publishes from 4 s
        self.assertTrue(res['ok'], res)
        self.assertEqual(len(self.restarts), 1)
        self.assertTrue(res['info']['lio_restart'])
        self.assertLess(r.info['imu_wait_s'], 3.)
        self.assertGreater(res['info']['exposure_s'], 2.9)
        self.assertEqual(res['pose'][0], 3.)

    def test_the_pose_waits_for_the_new_filter(self):
        r = self.make_restart(); r.start(restart=True)
        self.assertIsNone(self.feed(r, 0., 9., odom=lambda t: t < 2.2))  # the search is answered at ~6.5 s, the filter has not come back
        self.assertEqual(r.stage, 'search')
        res = self.feed(r, 9., 12.)                                      # it comes back
        self.assertTrue(res['ok'], res)

    def test_the_old_filters_last_messages_do_not_count_as_the_new_one(self):
        r = self.make_restart(); r.start(restart=True)
        res = self.feed(r, 0., 10., odom=lambda t: t < 2.3)              # a few messages in flight after the kill
        self.assertIsNone(res)

    def test_a_failed_restart_goes_on_with_the_filter_it_has(self):
        r = self.make_restart(ok=False); r.start(restart=True)
        res = self.feed(r, 0., 10.)
        self.assertTrue(res['ok'], res)
        self.assertEqual(len(self.restarts), 1)
        self.assertFalse(res['info']['lio_restart'])

    def test_a_restart_that_raises_is_reported_in_the_result(self):
        def restart():
            raise PermissionError('not ours')
        r = self.make(restart=restart); r.start(restart=True)
        res = self.feed(r, 0., 10.)
        self.assertTrue(res['ok'], res)                                  # it goes on with the filter it has ...
        self.assertFalse(res['info']['lio_restart'])
        self.assertIn('PermissionError', res['info']['lio_restart_error'])   # ... and says why it could not do better

    def test_the_restart_waits_for_the_lidar_like_the_exposure(self):
        # a fresh FAST-LIO2 with the lidar covered gets no points: it publishes nothing and the attempt would be wasted
        r = self.make_restart(); r.start(restart=True)
        self.assertIsNone(self.feed(r, 0., 6.4, points=lambda t: 0))
        self.assertEqual(self.restarts, [])
        res = self.feed(r, 6.4, 18., odom=lambda t: t < 7. or t > 9.)
        self.assertEqual(len(self.restarts), 1)
        self.assertTrue(res['ok'], res)

    def test_no_restart_unless_asked(self):
        r = self.make_restart(); r.start()                               # a parked verification does not hold Nav2 and must not touch the odometry
        res = self.feed(r, 0., 10.)
        self.assertTrue(res['ok'], res)
        self.assertEqual(self.restarts, [])
        self.assertNotIn('lio_restart', res['info'])

    def test_a_filter_that_does_not_come_back_ends_with_a_reason(self):
        r = self.make_restart(lio_timeout_s=3.); r.start(restart=True)
        res = self.feed(r, 0., 14., odom=lambda t: t < 2.0)              # the old filter dies with the restart: nothing after it
        self.assertIsNotNone(res)
        self.assertFalse(res['ok'])
        self.assertEqual(res['why'], 'lio_restart_timeout')

    def test_a_filter_that_does_not_come_back_kills_the_search_under_way(self):
        r = self.make_restart(lio_timeout_s=3.); r.start(restart=True)
        worker = FakeProc('/nonexistent', 'accepted', np.eye(4), polls=10 ** 6)
        r.stage, r.proc, r.t_restart, r.t_imu = 'search', worker, 0., 4.      # the old stage list could not hold a worker: this one outlived the run, beside the next attempt's
        res = r.poll()
        self.assertEqual(res['why'], 'lio_restart_timeout')
        self.assertTrue(worker.killed)


class SignalByCommTest(unittest.TestCase):
    """The restart kills FAST-LIO2 by its exact process name (never `pkill -f`, which matches command lines and kills the caller's own shell)."""
    def proc(self, entries):
        d = tempfile.TemporaryDirectory(); self.addCleanup(d.cleanup)
        for pid, comm in entries.items():
            (Path(d.name) / str(pid)).mkdir()
            if comm is not None:
                (Path(d.name) / str(pid) / 'comm').write_text(comm + '\n')
        (Path(d.name) / 'self').mkdir(); (Path(d.name) / 'cpuinfo').write_text('x')
        return d.name

    def test_only_the_exact_name_is_signalled(self):
        sent = []
        proc = self.proc({100: 'fastlio_mapping', 101: 'fastlio_mapping_helper', 102: 'ros2', 103: 'python3', 104: 'fastlio_mappin'})
        n = rl.signal_by_comm('fastlio_mapping', 15, proc=proc, kill=lambda pid, sig: sent.append((pid, sig)))
        self.assertEqual(n, 1)
        self.assertEqual(sent, [(100, 15)])

    def test_nothing_running_is_zero_and_unreadable_entries_are_skipped(self):
        sent = []
        proc = self.proc({100: None, 101: 'ros2'})
        self.assertEqual(rl.signal_by_comm('fastlio_mapping', 15, proc=proc, kill=lambda pid, sig: sent.append(pid)), 0)
        self.assertEqual(sent, [])

    def test_a_process_that_already_died_is_not_an_error(self):
        def kill(pid, sig): raise ProcessLookupError
        self.assertEqual(rl.signal_by_comm('fastlio_mapping', 15, proc=self.proc({100: 'fastlio_mapping'}), kill=kill), 0)

    def test_it_never_signals_itself(self):
        sent = []
        proc = self.proc({os.getpid(): 'fastlio_mapping'})
        self.assertEqual(rl.signal_by_comm('fastlio_mapping', 15, proc=proc, kill=lambda pid, sig: sent.append(pid)), 0)


class WorkerTest(unittest.TestCase):
    def test_the_library_of_lib_dir_is_the_one_loaded(self):
        # the one gn_supervisor_node and nav_live.sh checked: not a stray cpp_search/*.so shipped with the tools (another board's build)
        seen = {}
        real, rl.subprocess.Popen = rl.subprocess.Popen, lambda cmd, **k: seen.update(k['env']) or SimpleNamespace(pid=0)
        self.addCleanup(setattr, rl.subprocess, 'Popen', real)
        with tempfile.TemporaryDirectory() as d:
            rl.Worker('/bin/true', '/staged/build', Path(d) / 'in', Path(d) / 'o', 60.)
        self.assertEqual(seen['CORR_SEARCH_LIB'], '/staged/build/libcorr_search.so')

    def test_kill_reaches_a_child_that_outlived_the_worker(self):
        # the OOM killer takes search_v2 while its register_v2 runs: that one must not stay beside the next attempt's search
        w = rl.Worker.__new__(rl.Worker)
        w.p = subprocess.Popen(['bash', '-c', 'sleep 30 & echo $!; exit 9'], stdout=subprocess.PIPE, text=True, start_new_session=True)
        child = int(w.p.stdout.readline())
        w.p.stdout.close(); w.p.wait()
        self.addCleanup(lambda: subprocess.run(['kill', '-9', str(child)], capture_output=True))
        w.kill()
        for _ in range(100):
            stat = Path(f'/proc/{child}/stat')
            if not stat.exists() or stat.read_text().split()[2] == 'Z':
                return
            time.sleep(.01)
        self.fail('the child survived')

    def test_kill_of_a_group_already_gone_is_quiet(self):
        w = rl.Worker.__new__(rl.Worker)
        w.p = subprocess.Popen(['true'], start_new_session=True)
        w.p.wait()
        w.kill()


if __name__ == '__main__':
    unittest.main()


class LazyStillWindowTest(unittest.TestCase):
    def test_a_window_evaluated_only_when_needed_answers_like_the_eager_one(self):
        """need=False skips the test but keeps the window: once needed it answers what an always-evaluated window does (blocks of motion, isolated spikes, jitter, gaps)."""
        rng = np.random.default_rng(7)
        eager, lazy, t, checked = rl.StillWindow(), rl.StillWindow(), 100., [0, 0]
        for k in range(40000):
            t += 1 / 200. + rng.normal(0, 2e-4) + (rng.random() < 2e-4) * rng.uniform(.2, 1.5)
            moving = (k // 600) % 4 == 1 or rng.random() < 5e-4
            g = tuple(rng.normal(0, .2 if moving else .001, 3)); a = tuple(np.array([0, 0, 1.]) + rng.normal(0, .001, 3))
            need = rng.random() < .3                  # needed again right after skipped grid times, often
            e, l = eager.add(t, g, a), lazy.add(t, g, a, need=need)
            if need:
                self.assertEqual(e, l, k)
                self.assertEqual(eager.calm, lazy.calm, k)
                checked[e] += 1
        self.assertGreater(min(checked), 1000)        # both answers exercised


class GyroNoiseTest(LiveRelocBase):
    """The Nano's gyro is noisier with the stack running (fans, load): p95(|gyro - median|) 0.006-0.009 rad/s against the 0.0061 limit that decides whether to START. That is noise, not a push:
    it used to end 4 of 4 accepted relocalizations in moved_since_exposure (NEXT-63)."""
    def noisy(self, from_t, sigma=.0033, seed=5):
        rng = np.random.default_rng(seed)
        return lambda t: tuple(rng.normal(0, sigma, 3)) if t > from_t else (0., 0., 0.)

    def test_noise_during_the_exposure_and_the_search_does_not_discard_the_pose(self):
        r = self.make(); r.start()
        res = self.feed(r, 0., 16., gyro=self.noisy(2.3))
        self.assertTrue(res['ok'], res)

    def test_a_shove_on_top_of_the_noise_is_still_refused(self):
        r = self.make(); r.start()
        rng = np.random.default_rng(6)
        res = self.feed(r, 0., 16., gyro=lambda t: tuple(rng.normal(0, .0033, 3) + ((0., 0., .3) if 5.4 < t < 5.8 else (0., 0., 0.))) if t > 2.3 else (0., 0., 0.))
        self.assertFalse(res['ok'])
        self.assertEqual(res['why'], 'moved_since_exposure')

    def test_the_start_still_waits_for_the_strict_stillness(self):
        r = self.make(); r.start()
        res = self.feed(r, 0., 6., gyro=self.noisy(-1.))                       # noisy from the first sample: never quiet enough to start
        self.assertIsNone(res)
        self.assertEqual(r.stage, 'imu')


class SkewedStampsTest(LiveRelocBase):
    """The IMU stamps of the Nano are jittery with a long tail (mean 5.15 ms, median 4.5): 2 s / median(dt) asked for 444 samples of a window that holds 388, so a perfectly clean stream was
    'missing more than 5 %' and the robot never counted as still (13-18 s waiting to start in the failing runs, and the 'moved' checks fired)."""
    def stamps(self, n, seed=4):
        return 1. + np.cumsum(np.random.default_rng(seed).choice([.0045, .0055, .008], size=n, p=[.6, .3, .1]))

    def feed_skewed(self, r, seconds, gyro=lambda t: (0., 0., 0.)):
        ts = self.stamps(int(seconds * 220))
        ts = ts[ts < 1. + seconds]
        for k, t in enumerate(ts):
            r.add_imu(t, gyro(t), (0., 0., 1.))
            if k % 20 == 0:
                r.add_lidar(t + .001, self.cloud if r.stage == 'expose' else None, 13000)
                r.add_odom(t)
            res = r.poll()
            if res is not None:
                return res
        return None

    def test_a_clean_stream_with_skewed_jitter_starts_and_gets_its_pose(self):
        r = self.make(); r.start()
        res = self.feed_skewed(r, 16.)
        self.assertTrue(res and res['ok'], res)
        self.assertLess(res['info']['imu_wait_s'], 3.5)                       # it waited for the 2 s window, not for luck

    def test_a_real_dropout_in_the_imu_is_still_not_still(self):
        r = self.make(); r.start()
        ts = self.stamps(2000); ts = ts[(ts < 3.0) | (ts > 3.4)]               # 0.4 s without IMU while it waits to start
        for k, t in enumerate(ts[ts < 3.45]):
            r.add_imu(t, (0., 0., 0.), (0., 0., 1.))
            if k % 20 == 0:
                r.add_lidar(t + .001, None, 13000)
        self.assertEqual(r.stage, 'imu')                                       # the 2 s window that contains the hole does not pass


class ReviewFixesTest(LiveRelocBase):
    def test_a_push_during_the_search_that_the_wheels_do_not_report_is_still_refused(self):
        # no wheel data at all (base off): the IMU is the witness
        r = self.make(); r.start()
        res = self.feed(r, 0., 14., gyro=lambda t: (0., 0., .3) if t > 5.4 else (0., 0., 0.))
        self.assertFalse(res['ok'])
        self.assertEqual(res['why'], 'moved_since_exposure')

    def test_a_shove_that_ended_before_the_search_returned_is_still_refused(self):
        r = self.make(); r.start()
        res = self.feed(r, 0., 14., gyro=lambda t: (0., 0., .3) if 5.4 < t < 5.8 else (0., 0., 0.))   # moved briefly, then still again for the 2 s window before the result
        self.assertFalse(res['ok'])
        self.assertEqual(res['why'], 'moved_since_exposure')

    def test_the_moved_flag_of_an_aborted_exposure_does_not_poison_the_next_one(self):
        r = self.make(); r.start()
        self.feed(r, 0., 3.5, wheel=lambda t: t > 3.2)                      # a nudge in the first exposure ...
        self.feed(r, 3.5, 4.5, points=lambda t: 0)                          # ... which the cover then aborts
        res = self.feed(r, 4.5, 16.)
        self.assertTrue(res['ok'], res)

    def test_a_cover_during_the_exposure_restarts_the_filter_again_for_the_new_exposure(self):
        r = self.make_restart(); r.start(restart=True)
        self.feed(r, 0., 3.5)
        self.feed(r, 3.5, 4.5, points=lambda t: 0)
        res = self.feed(r, 4.5, 16., odom=lambda t: t < 2.2 or t > 6.)
        self.assertTrue(res['ok'], res)
        self.assertEqual(len(self.restarts), 2)                             # the first filter started while the hand covered the window: no points to initialise on
