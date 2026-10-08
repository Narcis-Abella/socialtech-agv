#!/usr/bin/env python3
import unittest

import numpy as np

import sensor_relative_scan as srs


class RelativeScanTests(unittest.TestCase):
    def test_height_band_is_relative_to_sensor_origin(self):
        # A wall at floor height 0.5 m appears at sensor z=-0.5 for h=1.0.
        p=np.array([[2.,0.,-.5],[3.,0.,.5],[4.,0.,-1.2]])
        a=srs.project_scan(p,[0,0,1],1.0,min_height=.1,max_height=1.8)
        self.assertEqual(a['band_point_count'],2)
        self.assertEqual(a['finite_count'],1)
        b=srs.project_scan(p,[0,0,1],1.5,min_height=.1,max_height=1.8)
        self.assertEqual(b['band_point_count'],2)

    def test_gravity_tilt_is_removed_from_vertical_and_horizontal_projection(self):
        up=np.array([.1,0.,.995]);up/=np.linalg.norm(up)
        x=np.array([0.,1.,0.]); y=np.cross(up,x)
        point=2*x-0.8*up
        scan=srs.project_scan([point],up,1.,min_height=.1,max_height=.3,range_min=.1)
        self.assertEqual(scan['finite_count'],1)
        self.assertAlmostEqual(float(np.min(scan['ranges'][np.isfinite(scan['ranges'])])),2.,places=6)

    def test_out_of_band_points_are_not_projected(self):
        p=np.array([[1.,0.,2.],[-1.,0.,2.]])
        scan=srs.project_scan(p,[0,0,1],1.,min_height=.1,max_height=1.8)
        self.assertEqual(scan['finite_count'],0)

    def test_invalid_inputs_rejected(self):
        with self.assertRaises(ValueError):srs.project_scan([[1,2]], [0,0,1], 1.)
        with self.assertRaises(ValueError):srs.project_scan([[1,2,3]], [0,0,0], 1.)


if __name__=='__main__':unittest.main()
