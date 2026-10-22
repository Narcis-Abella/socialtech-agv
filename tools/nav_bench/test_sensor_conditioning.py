import unittest
import numpy as np

import sensor_relative_scan as sv3


class ConditioningGravityTests(unittest.TestCase):
    def test_two_second_conditioning_accepts_nominal_200_hz(self):
        start=1_000_000_000;end=start+2_000_000_000
        samples=[(start+i*5_000_000,np.array([0.,0.,9.81])) for i in range(400)]
        gravity,meta=sv3.conditioning_gravity(samples,start,end)
        np.testing.assert_allclose(gravity,[0.,0.,9.81])
        self.assertEqual(meta['reason'],'ok')
        self.assertGreaterEqual(meta['coverage_ratio'],.95)

    def test_conditioning_rejects_hard_gap(self):
        start=0;end=2_000_000_000
        ts=[i*5_000_000 for i in range(400)]
        ts[200:]=[t+50_000_000 for t in ts[200:]]
        samples=[(t,np.array([0.,0.,9.81])) for t in ts]
        gravity,meta=sv3.conditioning_gravity(samples,start,end)
        self.assertIsNone(gravity)
        self.assertEqual(meta['reason'],'imu_hard_gap')

    def test_conditioning_rejects_low_coverage(self):
        start=0;end=2_000_000_000
        samples=[(i*5_000_000,np.array([0.,0.,9.81])) for i in range(300)]
        gravity,meta=sv3.conditioning_gravity(samples,start,end)
        self.assertIsNone(gravity)
        self.assertEqual(meta['reason'],'imu_coverage_below_95_percent')

    def test_conditioning_rejects_regressive_timestamps(self):
        start=0;end=100_000_000
        samples=[(i*5_000_000,np.array([0.,0.,9.81])) for i in range(20)]
        samples[10],samples[11]=samples[11],samples[10]
        gravity,meta=sv3.conditioning_gravity(samples,start,end)
        self.assertIsNone(gravity)
        self.assertEqual(meta['reason'],'non_monotonic_imu')

    def test_uses_fixed_dataset_cadence_not_window_local_median(self):
        start=0;end=2_000_000_000
        # Local timestamps jitter around 4.65 ms, while the audited whole-bag
        # median is 4.7858075 ms. The fixed cadence yields 418 expected samples.
        ts=[int(i*4_650_000) for i in range(400)]
        samples=[(t,np.array([0.,0.,9.81])) for t in ts]
        _,meta=sv3.conditioning_gravity(samples,start,end,4_785_807.5)
        self.assertEqual(meta['expected'],418)
        self.assertEqual(meta['observed'],400)
        self.assertGreaterEqual(meta['coverage_ratio'],.95)


if __name__=='__main__':
    unittest.main()
