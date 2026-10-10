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
            if s not in rc.SEARCH_STATES:
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

    def test_stamp_jump_does_not_block_the_node(self):
        # Regression: a forward stamp jump (sensor clock -> epoch on time sync) made add() walk the 0.1 s grid across the whole
        # jump inside the IMU callback: 5.5 s for 1e5 s, hours for an epoch jump, with the 60 s deadline not served meanwhile.
        import time
        g = rc.ImuGate()
        for i in range(100):
            g.add(i * .005, [0, 0, 0], [0, 0, 1])
        t = time.monotonic()
        self.assertFalse(g.add(1e6, [0, 0, 0], [0, 0, 1]))
        self.assertLess(time.monotonic() - t, .5)
        self.assertEqual(g.evaluations, 0)
        ready = None                       # the gate still opens after 2 s of still samples on the new time base
        for i in range(1, 500):
            if g.add(1e6 + i * .005, [0, 0, 0], [0, 0, 1]):
                ready = i * .005
                break
        self.assertIsNotNone(ready)
        self.assertAlmostEqual(ready, 2.0, delta=.11)

    def test_short_gap_gives_the_same_ready_time_as_before(self):
        # 5 s gap: values recorded from the pre-fix grid walk (ab28151), which could still cross a gap this short.
        g = rc.ImuGate()
        for i in range(100):
            g.add(i * .005, [0, 0, 0], [0, 0, 1])
        for i in range(600):
            if g.add(5.5 + i * .005, [0, 0, 0], [0, 0, 1]):
                break
        self.assertAlmostEqual(g.ready_stamp, 7.5, places=6)
        self.assertEqual((g.evaluations, g.failed_evaluations), (20, 19))


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


class S2S3Test(unittest.TestCase):
    def test_s3_only_after_s2_insufficient_and_supervision(self):
        self.assertEqual({s for s, n in rc.TRANSITIONS.items() if 'S3_ACCUMULATING' in n}, {'S3_SUPERVISION_REQUIRED'})
        self.assertEqual({s for s, n in rc.TRANSITIONS.items() if 'S3_SUPERVISION_REQUIRED' in n}, {'S2_INSUFFICIENT'})
        self.assertEqual({s for s, n in rc.TRANSITIONS.items() if 'S2_INSUFFICIENT' in n}, {'S2_ACCUMULATING', 'S2_SEARCHING'})
        for s in ('S2_INSUFFICIENT', 'S3_SUPERVISION_REQUIRED', 'S2_ACCUMULATING', 'S3_ACCUMULATING'):
            self.assertNotIn('ACCEPTED', rc.TRANSITIONS[s]); self.assertNotIn('HANDOFF', rc.TRANSITIONS[s])
        self.assertEqual({s for s, n in rc.TRANSITIONS.items() if 'HANDOFF' in n}, {'ACCEPTED'})

    def test_one_clock_across_s2_and_s3(self):
        c = Clock(); a = rc.Attempt(1, c); a.input_started()
        for s in ('WAIT_SENSORS', 'IMU_CONDITIONING', 'LIO_STARTING', 'S2_ACCUMULATING', 'S2_SEARCHING'):
            a.go(s); c.t += 4.
        a.go('S2_INSUFFICIENT'); a.go('S3_SUPERVISION_REQUIRED')
        self.assertAlmostEqual(a.elapsed(), 20.)                    # the S2 time is not given back to S3
        self.assertTrue(rc.s3_viable(a.elapsed())); c.t += 18.1
        self.assertFalse(rc.s3_viable(a.elapsed()))                 # 38.1 s used: < 22 s left for build + search
        a.go('TIMEOUT')

    def test_s2s3_cut_then_s3_limit(self):
        a = rc.Acquisition('s2s3')
        xs = [0.] * 40 + list(np.arange(.1, 6, .1))
        for i, x in enumerate(xs):
            a.add_odom(i * .1, T(x)); a.add_odom(i * .1 + .05, T(x))
            if not a.add_cloud(i * .1 + .02, np.zeros((5, 3))): break
        self.assertEqual(a.s2_n, 30)                                 # 3.0 s at 10 Hz, same rule as the s2 baseline
        self.assertAlmostEqual(a.s2_radius, 0.)
        self.assertEqual(a.stop_reason, 'radius'); self.assertLessEqual(a.max_radius, rc.S3_MAX_RADIUS_M)
        self.assertGreater(len(a.clouds), a.s2_n)                    # accumulation continued past the S2 cut in the same buffers
        b = rc.Acquisition('s2'); 
        for i in range(40):
            b.add_odom(i * .1, T()); b.add_odom(i * .1 + .05, T())
            if not b.add_cloud(i * .1 + .02, np.zeros((5, 3))): break
        self.assertEqual(b.s2_n, a.s2_n)

    def test_s2_exposure_parameter(self):
        for exp, n in ((1., 10), (5., 50)):
            a = rc.Acquisition('s2s3', s2_exposure=exp)
            for i in range(80):
                a.add_odom(i * .1, T()); a.add_odom(i * .1 + .05, T()); a.add_cloud(i * .1 + .02, np.zeros((5, 3)))
            self.assertEqual(a.s2_n, n)


class StampJitterTest(unittest.TestCase):
    def test_complete_stream_with_alternating_stamps(self):
        # Mid-360 IMU pattern: steps cluster at 4.25 ms and 5.75 ms, more short than long, mean 5 ms. Nothing is missing.
        rng = np.random.default_rng(1)
        steps = np.where(rng.random(4000) < .55, .00425, .00575 + .0011); steps *= .005 / steps.mean()
        s = rc.StreamStats('imu'); t = 0.
        for d in steps:
            t += d; s.add(t, t)
        r = s.report()
        self.assertAlmostEqual(r['rate_hz_stamp'], 200., delta=.2)       # the stream is complete
        self.assertGreater(r['lost_estimate'], 0)                        # the gap counter is fooled: documented, do not trust it here


class HandoffTest(unittest.TestCase):
    def test_pose_propagates_with_odometry(self):
        T_map_src = T(10., .5, 4.); T_ci_first = T(0.); T_ci_now = T(1.5, .2)
        T_map_ci, xyyaw, q = rc.handoff_pose(T_map_src, np.eye(4), T_ci_first, T_ci_now)
        ref = T_map_src @ T_ci_now
        self.assertAlmostEqual(xyyaw[0], ref[0, 3]); self.assertAlmostEqual(xyyaw[2], .7)
        np.testing.assert_allclose(q, [0, 0, 0, 1], atol=1e-12)

    def test_mount_extrinsic(self):
        self.assertEqual(rc.compose_planar((1., 2., .3), (0., 0., 0.)), (1., 2., .3))       # default: behaviour of note 15 unchanged
        x, y, w = rc.compose_planar((1., 2., np.pi / 2), (.5, 0., .1))                      # base 0.5 m ahead of the sensor, robot facing +y
        self.assertAlmostEqual(x, 1.); self.assertAlmostEqual(y, 2.5); self.assertAlmostEqual(w, np.pi / 2 + .1)

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
