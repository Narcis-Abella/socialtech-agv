import unittest

import numpy as np

from imu_ready import MOTION_RESIDUAL_SCALE, episodes, ready_times, window_ok


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
