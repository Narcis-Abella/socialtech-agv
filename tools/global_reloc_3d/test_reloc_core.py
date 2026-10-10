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


class Phase2Test(unittest.TestCase):
    def test_no_sensor_input_is_bounded_and_separate_from_the_deadline(self):
        c = Clock(); a = rc.Attempt(1, c); a.go('WAIT_SENSORS')
        c.t += rc.SENSOR_WAIT_S - .1
        self.assertFalse(a.no_input()); self.assertFalse(a.expired())
        c.t += .2
        self.assertTrue(a.no_input()); self.assertFalse(a.expired())      # O1: F1 had no terminal state here
        b = rc.Attempt(2, c); b.go('WAIT_SENSORS'); c.t += 29.; b.input_started(); c.t += 59.
        self.assertFalse(b.no_input()); self.assertFalse(b.expired())       # a late first message still gets its full 60 s
        c.t += 1.1
        self.assertTrue(b.expired())

    def test_localized_is_revocable_and_never_comes_back(self):
        self.assertEqual(rc.TRANSITIONS['LOCALIZED'], {'LOCALIZATION_LOST'})
        self.assertEqual(rc.TRANSITIONS['LOCALIZATION_LOST'], set())
        self.assertIn('LOCALIZED', rc.TERMINAL); self.assertIn('LOCALIZATION_LOST', rc.TERMINAL)
        for s, nxt in rc.TRANSITIONS.items():
            if s != 'HANDOFF':
                self.assertNotIn('LOCALIZED', nxt)

    def test_odometry_divergence(self):
        o = rc.OdomPlausibility()
        for i in range(100):                                   # 1.5 m/s, duplicated and reversed stamps in between
            self.assertFalse(o.add(i * .1, [i * .15, 0, 0]))
            o.add(i * .1, [99, 0, 0]); o.add(i * .1 - .05, [-99, 0, 0])
        self.assertAlmostEqual(o.max_seen, 1.5, places=6)
        o.add(10., [15.85, 0, 0])                              # one glitch of 10 m/s does not latch
        self.assertFalse(o.add(10.1, [16., 0, 0]))
        x = 16.
        for i in range(3):
            x += .5; d = o.add(10.2 + i * .1, [x, 0, 0])
        self.assertTrue(d)
        self.assertTrue(o.add(11., [x, 0, 0]))                 # latched

    def test_localization_monitor(self):
        m = rc.LocalizationMonitor(0.)
        t = 0.
        for i in range(100):
            t = i * .1; m.add(t, .05, 1.)
        self.assertIsNone(m.lost(t))
        self.assertEqual(m.lost(t + 3.1), 'amcl_silent')       # AMCL stopped publishing map -> odom
        m.add(t + .1, .6, 1.); m.add(t + .2, .1, 1.)           # a single outlier is not a loss
        for i in range(29):
            m.add(t + .3 + i * .1, .6, 1.)
        self.assertIsNone(m.lost(t + 3.2))
        m.add(t + 3.4, .1, 20.)                                # yaw alone also counts
        self.assertEqual(m.lost(t + 3.4), 'amcl_incoherent')
        self.assertLessEqual(len(m.report()['trace_t_xy_yaw']), 720)

    def test_monitor_and_buffers_are_bounded(self):
        m = rc.LocalizationMonitor(0.)
        rows = []
        for i in range(200000):                                # 5.5 h at 10 Hz
            m.add(i * .1, .01, .1); rows.append((i * .1, None)); rc.trim(rows)
        self.assertLess(len(rows), 2 * rc.ODOM_KEEP)
        self.assertEqual(rows[-1][0], 199999 * .1)
        self.assertEqual(len(m.trace), 720)

    def test_permit_gate_fails_closed(self):
        g = rc.PermitGate()
        self.assertEqual(g.state(0.), (False, 'no_permit'))
        g.on_permit(True, 0.); g.on_permit(True, .1)
        self.assertFalse(g.state(.1)[0])                       # two heartbeats are not enough
        g.on_permit(True, .2)
        self.assertTrue(g.state(.2)[0])
        self.assertEqual(g.state(.2, publishers=2), (False, 'publishers_2'))   # a second publisher closes it and resets the streak
        self.assertFalse(g.state(.2)[0])
        for i in range(3):
            g.on_permit(True, .3 + i * .1)
        self.assertTrue(g.state(.5)[0])
        self.assertEqual(g.state(.5 + rc.PERMIT_TTL_S + .01), (False, 'permit_stale'))   # publisher died: closed after the TTL
        g.on_permit(True, 2.)
        self.assertFalse(g.state(2.)[0])                       # one heartbeat after a gap does not reopen it
        g.on_permit(True, 2.1); g.on_permit(True, 2.2)
        self.assertTrue(g.state(2.2)[0])
        g.on_permit(False, 2.3)
        self.assertEqual(g.state(2.3), (False, 'permit_false'))               # revoked at once
        g.on_permit(True, 2.4); g.on_permit(False, 2.5); g.on_permit(True, 2.6); g.on_permit(True, 2.7)
        self.assertFalse(g.state(2.7)[0])                      # interleaved true/false (two sources disagreeing) never opens
        self.assertEqual(g.state(2.7, publishers=0), (False, 'publishers_0'))


    def test_input_stamps_backward_or_jumping_latch_only_while_armed(self):
        i = rc.InputIntegrity('lidar')
        for k in range(5):
            i.add(100. + .1 * k, 10. + .1 * k, True)
        i.add(102.4, 12.4, True)                      # a 2 s gap in both clocks is a gap, not a jump
        i.add(102.5, 12.41, True)                     # catching up after a stall: stamps advance faster than the wall, by < 1 s
        self.assertIsNone(i.fault)
        i.add(102.4, 12.5, False)                     # backward while not armed (IMU conditioning has its own rule): counted only
        self.assertEqual((i.backward, i.fault), (1, None))
        i.add(102.6, 12.6, True); i.add(102.6, 12.7, True)      # a repeated stamp
        self.assertEqual(i.fault, 'input_stamp_backward_lidar')
        j = rc.InputIntegrity('imu')
        j.add(5., 1., True); j.add(1005.005, 1.005, True); j.add(1005.01, 1.01, True)
        self.assertEqual((j.jumps, j.fault), (1, 'input_stamp_jump_imu'))      # latched: later sane stamps do not clear it


    def test_planar_delta_takes_the_own_pose_onto_the_amcl_pose(self):
        own, amcl = (1., 2., .3), (1.4, 1.7, .5)
        D = rc.planar_delta(amcl, own)
        c, s = np.cos(own[2]), np.sin(own[2])
        T = np.eye(4); T[:2, :2] = [[c, -s], [s, c]]; T[:2, 3] = own[:2]
        M = D @ T
        self.assertTrue(np.allclose(M[:2, 3], amcl[:2]))
        self.assertAlmostEqual(np.arctan2(M[1, 0], M[0, 0]), amcl[2])


if __name__ == '__main__':
    unittest.main()
