import json
import math
import os
import tempfile
import unittest
from pathlib import Path

import numpy as np

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
                r.add_lidar(t + .001, self.cloud, points(t))
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

    def test_a_cover_during_the_exposure_does_not_restart_the_filter_twice(self):
        r = self.make_restart(); r.start(restart=True)
        self.feed(r, 0., 3.5)
        self.feed(r, 3.5, 4.5, points=lambda t: 0)
        res = self.feed(r, 4.5, 14.)
        self.assertTrue(res['ok'], res)
        self.assertEqual(len(self.restarts), 1)

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


if __name__ == '__main__':
    unittest.main()
