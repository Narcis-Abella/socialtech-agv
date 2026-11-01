import json
import math
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


class LiveRelocTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); d = Path(self.tmp.name)
        rng = np.random.default_rng(0)
        np.savetxt(d / 'map.xyz', rng.uniform(-5, 5, (500, 3)))
        self.cloud = rng.uniform(-4, 4, (400, 3))
        self.map, self.work = d / 'map.xyz', d / 'work'
        self.procs = []

    def tearDown(self):
        self.tmp.cleanup()

    def make(self, decision='accepted', T=None):
        def spawn(run_dir, out_dir, deadline_s):
            p = FakeProc(out_dir, decision, planar(3., 1., .5) if T is None else T); self.procs.append(p); return p
        return rl.LiveReloc(str(self.map), 'unused', '/opt', str(self.work), spawn=spawn)

    def feed(self, r, t0, t1, odom=lambda t: planar(0., 0., 0.), gyro=(0., 0., 0.)):
        """IMU at 200 Hz (accelerometer in g, as the Livox driver), odometry and body clouds at 10 Hz."""
        for k in range(int((t1 - t0) * 200)):
            t = t0 + k / 200.
            r.add_imu(t, gyro, (0., 0., 1.))
            if k % 20 == 0:
                r.add_odom(t, odom(t)); r.add_cloud(t + .001, self.cloud)
            res = r.poll()
            if res is not None:
                return res
        return None

    def test_still_robot_gives_the_pose_of_the_accepted_search(self):
        r = self.make(); r.start()
        res = self.feed(r, 0., 8.)
        self.assertIsNotNone(res)
        self.assertTrue(res['ok'], res)
        x, y, yaw = res['pose']   # still robot, levelled frame = first odometry frame: the base pose is the accepted transform itself
        self.assertAlmostEqual(x, 3., 6); self.assertAlmostEqual(y, 1., 6); self.assertAlmostEqual(yaw, .5, 6)
        self.assertEqual(res['why'], 'accepted')
        self.assertGreaterEqual(res['info']['exposure_s'], 2.9)

    def test_pose_follows_the_odometry_to_the_newest_stamp(self):
        # robot 0.02 m further (inside the stillness bound) when the search answers: the pose is propagated, as reloc_node does
        r = self.make(); r.start()
        res = self.feed(r, 0., 8., odom=lambda t: planar(.02 if t > 5.5 else 0., 0., 0.))
        self.assertTrue(res['ok'], res)
        self.assertAlmostEqual(res['pose'][0], 3. + .02 * math.cos(.5), 3)

    def test_no_search_while_the_imu_says_moving(self):
        r = self.make(); r.start()
        self.assertIsNone(self.feed(r, 0., 6., gyro=(0., 0., .3)))
        self.assertEqual(self.procs, [])
        self.assertEqual(r.stage, 'imu')

    def test_motion_during_the_exposure_is_refused(self):
        r = self.make(); r.start()
        res = self.feed(r, 0., 8., odom=lambda t: planar(max(t - 2.5, 0.) * .2, 0., 0.))   # IMU gate passed, then 0.2 m/s
        self.assertFalse(res['ok'])
        self.assertEqual(res['why'], 'moved_during_exposure')
        self.assertEqual(self.procs, [])

    def test_a_rejected_search_reports_its_decision(self):
        r = self.make('ambiguous_no_alternative_margin'); r.start()
        res = self.feed(r, 0., 8.)
        self.assertFalse(res['ok'])
        self.assertEqual(res['why'], 'ambiguous_no_alternative_margin')

    def test_abort_kills_the_worker_and_a_new_start_is_clean(self):
        class Hang(FakeProc):
            def poll(self): return None
        procs = []
        r = rl.LiveReloc(str(self.map), 'unused', '/opt', str(self.work), spawn=lambda *a: procs.append(Hang(a[1], '', None)) or procs[-1])
        r.start(); self.assertIsNone(self.feed(r, 0., 8.))
        self.assertEqual(len(procs), 1)
        r.abort()
        self.assertTrue(procs[0].killed)
        self.assertIsNone(r.poll())
        r.start(); self.assertEqual(r.stage, 'imu')


if __name__ == '__main__':
    unittest.main()
