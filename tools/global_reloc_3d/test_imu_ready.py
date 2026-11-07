import unittest

import numpy as np

from imu_ready import MOTION_RESIDUAL_SCALE, episodes, expected_samples, ready_times, window_ok


class ImuReadyTest(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(0)
        self.t = np.arange(0, 10, .005)
        self.g = np.tile([.02, 0, .01], (len(self.t), 1)) + rng.normal(0, .001, (len(self.t), 3))
        self.a = np.tile([0, 0, 1.], (len(self.t), 1)) + rng.normal(0, .002, (len(self.t), 3))

    def test_stationary_with_bias_passes_turning_fails(self):
        self.assertTrue(window_ok(self.g[:400], self.a[:400], 400))
        turning = self.g[:400] + [0, 0, .3]
        self.assertFalse(window_ok(turning, self.a[:400], 400))
        self.assertFalse(window_ok(self.g[:300], self.a[:300], 400))  # coverage < 95 %

    def test_gyro_noise_above_the_start_limit_is_not_motion(self):
        # seen on the Nano with the stack running: p95(|gyro - median|) 0.0063-0.0092 against the 0.0061 start limit, the robot not moving (IMU of 2026-11-07, imu_still.py)
        rng = np.random.default_rng(3)
        g = np.tile([.02, 0, .01], (400, 3 // 3)) + rng.normal(0, .0033, (400, 3))
        resid = np.percentile(np.linalg.norm(g - np.median(g, axis=0), axis=1), 95)
        self.assertGreater(resid, .0065); self.assertLess(resid, .012)        # the fixture is in the observed range
        self.assertFalse(window_ok(g, self.a[:400], 400))                      # too noisy to START a filter or an exposure ...
        self.assertTrue(window_ok(g, self.a[:400], 400, residual_scale=MOTION_RESIDUAL_SCALE))   # ... but nobody moved it

    def test_a_shove_is_motion_at_any_noise_scale(self):
        g = self.g[:400].copy(); g[100:140] += [0, 0, .3]                      # 0.2 s at 0.3 rad/s
        self.assertFalse(window_ok(g, self.a[:400], 400, residual_scale=MOTION_RESIDUAL_SCALE))
        a = self.a[:400].copy(); a[200:220] += [.5, 0, 0]                      # a bump the gyro does not see
        self.assertFalse(window_ok(self.g[:400], a, 400, residual_scale=MOTION_RESIDUAL_SCALE))

    def skewed_stamps(self, n, seed=4):
        # the Livox IMU on the Nano: mostly a bit under 5 ms, now and then 8 ms (packets batched): median 4.5 ms, mean 5.15 ms, i.e. ~194 Hz; a window of 2 s holds ~388 samples
        d = np.random.default_rng(seed).choice([.0045, .0055, .008], size=n, p=[.6, .3, .1])
        return np.cumsum(d)

    def test_expected_samples_does_not_ask_for_more_than_the_stream_has_when_its_jitter_is_skewed(self):
        t = self.skewed_stamps(2000)
        w = t[(t >= t[0] + 1) & (t < t[0] + 3)]                       # a 2 s window of a clean stream
        self.assertLess(w.size / (2. / np.median(np.diff(w))), .95)    # what the code did: 2 s / median(dt) -> the window "lacks" more than 5 % of its samples
        self.assertGreaterEqual(w.size / expected_samples(w, 2.), .97)
        self.assertLessEqual(w.size / expected_samples(w, 2.), 1.03)

    def test_a_real_dropout_still_counts_as_missing_samples(self):
        t = self.skewed_stamps(2000)
        w = t[(t >= t[0] + 1) & (t < t[0] + 3)]
        hole = np.concatenate([w[:100], w[100 + 60:]])                 # 60 samples (0.3 s) lost in the middle
        self.assertLess(hole.size / expected_samples(hole, 2.), .95)
        self.assertFalse(window_ok(np.zeros((hole.size, 3)), np.tile([0, 0, 1.], (hole.size, 1)), expected_samples(hole, 2.)))

    def test_the_offline_filter_finds_a_still_robot_with_skewed_jitter(self):
        t = self.skewed_stamps(4000)
        g = np.zeros((len(t), 3)); a = np.tile([0, 0, 1.], (len(t), 1))
        eps = episodes(ready_times(t, g, a))
        self.assertEqual(len(eps), 1)                                          # it used to find none: the same 2 s / median(dt) as the live window
        self.assertLess(eps[0][0] - t[0], 2.2)

    def test_degenerate_stamps_do_not_switch_the_coverage_check_off(self):
        z, a = np.zeros((5, 3)), np.tile([0, 0, 1.], (5, 1))
        self.assertFalse(window_ok(z, a, 0))                                   # expected 0 used to mean "no coverage requirement"
        self.assertFalse(window_ok(z, a, expected_samples([0, 0, 0, 0, 1], 2.)))
        self.assertFalse(window_ok(z, a, -3))

    def test_a_backward_step_in_the_stamps_does_not_change_the_typical_period(self):
        t = np.concatenate([np.arange(0, 1, .005), np.arange(0.5, 1.5, .005)])      # the clock stepped back 0.5 s in the middle
        self.assertTrue(380 <= expected_samples(t, 2.) <= 420)

    def test_episodes_end_where_motion_starts(self):
        g = self.g.copy()
        g[self.t >= 6] += [0, 0, .3]
        eps = episodes(ready_times(self.t, g, self.a))
        self.assertEqual(len(eps), 1)
        self.assertAlmostEqual(eps[0][0], 2., places=1)
        self.assertLessEqual(eps[0][1], 6.)
        self.assertGreater(eps[0][1], 5.8)


if __name__ == '__main__':
    unittest.main()
