#!/usr/bin/env python3
"""Synthetic sensor-only tests for gravity-aligned ground plane estimation."""
import unittest

import numpy as np

import estimate_sensor_height as eh


class HeightTests(unittest.TestCase):
    def test_estimates_supported_floor_with_ceiling_and_outliers(self):
        rng=np.random.default_rng(71)
        xy=rng.uniform(-2.,2.,(5000,2))
        floor=np.c_[xy,np.full(len(xy),-1.)+rng.normal(0.,.004,len(xy))]
        ceiling=np.c_[xy[:1800],np.full(1800,1.5)+rng.normal(0.,.01,1800)]
        noise=rng.uniform(-3.,3.,(1200,3))
        points=np.vstack([floor,ceiling,noise])
        result=eh.estimate_ground_plane(points,[0.,0.,1.],seed=4)
        self.assertTrue(result['supported'],result)
        self.assertAlmostEqual(result['height_m'],1.,delta=.02)
        self.assertGreaterEqual(result['inlier_cells'],8)
        self.assertLess(result['tilt_deg'],1.)

    def test_handles_floor_tilt_relative_to_gravity(self):
        rng=np.random.default_rng(17)
        xy=rng.uniform(-2.,2.,(4000,2))
        z=-1.+.03*xy[:,0]-.02*xy[:,1]+rng.normal(0.,.003,len(xy))
        result=eh.estimate_ground_plane(np.c_[xy,z],[0.,0.,1.],seed=2)
        self.assertTrue(result['supported'],result)
        self.assertAlmostEqual(result['height_m'],1.,delta=.02)
        self.assertAlmostEqual(result['tilt_deg'],np.degrees(np.arctan(np.hypot(.03,.02))),delta=.5)

    def test_rejects_upward_only_field_of_view(self):
        rng=np.random.default_rng(5)
        points=rng.normal(size=(3000,3));points[:,2]=np.abs(points[:,2])+.2
        result=eh.estimate_ground_plane(points,[0.,0.,1.])
        self.assertFalse(result['supported'])
        self.assertEqual(result['reason'],'no_downward_returns')

    def test_rejects_vertical_wall_as_ground(self):
        rng=np.random.default_rng(13)
        points=np.c_[rng.normal(0.,.005,3000),rng.uniform(-2.,2.,3000),rng.uniform(-1.5,-.5,3000)]
        result=eh.estimate_ground_plane(points,[0.,0.,1.],seed=9)
        self.assertFalse(result['supported'])
        self.assertEqual(result['reason'],'insufficient_horizontal_spread')

    def test_window_summary_requires_multiple_stable_frames(self):
        frames=[{'supported':True,'height_m':1.+d,'inliers':1000,'inlier_cells':30,'tilt_deg':1.,'inlier_ratio':.5}
                for d in (-.01,0.,.01)]
        summary=eh.summarize_frames(frames)
        self.assertEqual(summary['valid_frames'],3)
        self.assertTrue(summary['stable'])
        unstable=eh.summarize_frames([frames[0],frames[1],{**frames[2],'height_m':1.3}])
        self.assertFalse(unstable['stable'])


if __name__=='__main__':
    unittest.main()
