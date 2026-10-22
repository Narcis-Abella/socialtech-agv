import unittest

import numpy as np

from imu_ready import episodes, ready_times, window_ok


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
