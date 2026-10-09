import unittest
import numpy as np
from scipy.spatial.transform import Rotation
import reloc_core as rc


class Clock:
    def __init__(self): self.t = 100.
    def __call__(self): return self.t


class StateTest(unittest.TestCase):
    def test_transitions_and_deadline(self):
        c = Clock(); a = rc.Attempt(1, c)
        a.go('WAIT_SENSORS')
        self.assertRaises(rc.IllegalTransition, a.go, 'ACCEPTED')   # cannot skip searching and verification
        self.assertFalse(a.expired())
        a.input_started(); c.t += 59.9
        self.assertFalse(a.expired()); c.t += .2
        self.assertTrue(a.expired())
        a.go('TIMEOUT')
        self.assertRaises(rc.IllegalTransition, a.go, 'ACCEPTED')   # terminal states stay terminal

    def test_only_search_can_accept(self):
        for s in rc.TRANSITIONS:
            if s != 'SEARCHING':
                self.assertNotIn('ACCEPTED', rc.TRANSITIONS[s])
        self.assertEqual(rc.TRANSITIONS['ACCEPTED'], {'HANDOFF', 'ERROR'})

    def test_stream_loss(self):
        s = rc.StreamStats('imu')
        for i in range(100):
            if i in (50, 51, 52): continue
            s.add(i * .005, i * .005)
        self.assertEqual(s.lost, 3)


def imu_stream(gate, seconds, rate=200., noise=1e-3, turn=0., t0=0.):
    rng = np.random.default_rng(0)
    for i in range(int(seconds * rate)):
        g = rng.normal(0, noise, 3) + [0, 0, turn]
        a = np.array([0, 0, 1.]) + rng.normal(0, 1e-3, 3)
        if gate.add(t0 + i / rate, g, a):
            return t0 + i / rate
    return None


class GateTest(unittest.TestCase):
    def test_ready_after_two_seconds_still(self):
        g = rc.ImuGate(); r = imu_stream(g, 4.)
        self.assertIsNotNone(r); self.assertAlmostEqual(g.ready_stamp, 2., delta=.15)

    def test_not_ready_while_turning(self):
        g = rc.ImuGate(); self.assertIsNone(imu_stream(g, 6., turn=.3)); self.assertGreater(g.failed_evaluations, 10)

    def test_ready_only_after_motion_stops(self):
        g = rc.ImuGate(); self.assertIsNone(imu_stream(g, 3., turn=.3))
        r = imu_stream(g, 4., t0=3.005)
        self.assertGreaterEqual(g.ready_stamp, 4.99)


def T(x=0., yaw=0., y=0.):
    M = np.eye(4); M[:3, :3] = Rotation.from_euler('z', yaw).as_matrix(); M[:3, 3] = [x, y, 0]; return M


class AcqTest(unittest.TestCase):
    def feed(self, acq, xs, dt=.1):
        for i, x in enumerate(xs):
            acq.add_odom(i * dt, T(x)); acq.add_odom(i * dt + .05, T(x))
            if not acq.add_cloud(i * dt + .02, np.zeros((5, 3))): return i
        return None

    def test_s3_stops_at_radius_and_excludes_breaking_cloud(self):
        a = rc.Acquisition('s3'); i = self.feed(a, np.arange(0, 6, .5))
        self.assertEqual(a.stop_reason, 'radius'); self.assertEqual(len(a.clouds), 6)

    def test_stale_messages_refused(self):
        a = rc.Acquisition('s3', min_stamp=5.)
        a.add_odom(4., T()); a.add_cloud(4., np.zeros((1, 3))); a.add_imu(4., np.zeros(3))
        self.assertEqual((len(a.odom), len(a.clouds), len(a.imu), a.refused), (0, 0, 0, 2))

    def test_s2_exposure(self):
        a = rc.Acquisition('s2'); self.feed(a, np.zeros(40)); self.assertEqual(a.stop_reason, 's2_exposure')


class HandoffTest(unittest.TestCase):
    def test_pose_propagates_with_odometry(self):
        T_map_src = T(10., .5, 4.); T_ci_first = T(0.); T_ci_now = T(1.5, .2)
        T_map_ci, xyyaw, q = rc.handoff_pose(T_map_src, np.eye(4), T_ci_first, T_ci_now)
        ref = T_map_src @ T_ci_now
        self.assertAlmostEqual(xyyaw[0], ref[0, 3]); self.assertAlmostEqual(xyyaw[2], .7)
        np.testing.assert_allclose(q, [0, 0, 0, 1], atol=1e-12)

    def test_monitor_needs_sustained_coherence(self):
        m = rc.HandoffMonitor()
        for t in np.arange(0, 3, .5): m.add(t, (0, 0, 0), (0, 0, 0))
        self.assertFalse(m.confirmed)
        m.add(3.5, (1., 0, 0), (0, 0, 0))                 # a bad update resets the run
        for t in np.arange(4, 10, .5): m.add(t, (.05, 0, .01), (0, 0, 0))
        self.assertTrue(m.confirmed)


class S3EquivalenceTest(unittest.TestCase):
    def test_incremental_matches_s3_limit(self):
        from lio_submap import s3_limit
        rng = np.random.default_rng(3)
        for trial in range(20):
            xy = np.cumsum(rng.normal(0, .25, (80, 2)), axis=0)
            poses = []
            for p in xy:
                M = np.eye(4); M[:2, 3] = p; poses.append(M)
            n, reason, _, _ = s3_limit(list(range(80)), [np.linalg.inv(poses[0]) @ P for P in poses], 2.8, 6.)
            a = rc.Acquisition('s3')
            for i, P in enumerate(poses):
                a.add_odom(i * .1, P); a.add_odom(i * .1 + .05, P)
            for i in range(80):
                if not a.add_cloud(i * .1 + .02, np.zeros((1, 3))): break
            # s3_limit reads poses at cloud stamps; interpolation between equal-valued neighbours differs slightly, so allow +-1 cloud
            self.assertLessEqual(abs(len(a.clouds) - n), 1, (trial, len(a.clouds), n))


if __name__ == '__main__':
    unittest.main()
