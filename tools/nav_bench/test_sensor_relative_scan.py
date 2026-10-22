#!/usr/bin/env python3
import unittest

import numpy as np

import sensor_relative_scan as srs


class RelativeScanTests(unittest.TestCase):
    def test_complete_custommsg_exposure_excludes_overlapping_next_frame(self):
        # Point-5 eco01 index: frame 25 starts before frame 24 ends. Header
        # filtering admitted both into a strict one-frame S1 window.
        first,last=srs.custom_point_bounds_ns(1780999552138957824,[0,100348160])
        start=1780999552138957824;end=1780999552239305985
        self.assertEqual((first,last),(start,end-1))
        self.assertTrue(srs.complete_point_window(first,last,start,end))
        next_first,next_last=srs.custom_point_bounds_ns(
            1780999552238969600,[0,100225536])
        self.assertLess(next_first,end)
        self.assertFalse(srs.complete_point_window(next_first,next_last,start,end))
        with self.assertRaises(ValueError):srs.custom_point_bounds_ns(1,[])

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

    def test_fov_resolution_range_and_mask_are_explicit(self):
        p=np.array([[2.,0.,0.],[0.,2.,0.],[0.,-2.,0.],[-2.,0.,0.],[.5,0.,0.]])
        scan=srs.project_scan(p,[0,0,1],1.,min_height=.1,max_height=1.8,
            fov_deg=180.,fov_center_deg=0.,angle_increment=np.deg2rad(2.),
            range_min=1.,range_max=10.,excluded_angle_sectors_deg=[(-5.,5.)])
        self.assertEqual(scan['beam_count'],90)
        self.assertEqual(scan['finite_count'],1)
        self.assertAlmostEqual(float(np.nanmin(np.where(np.isfinite(scan['ranges']),scan['ranges'],np.nan))),2.)

    def test_sensor_relative_band_bypasses_floor_height(self):
        p=np.array([[2.,0.,-.3],[3.,0.,-.1],[4.,0.,.1]])
        scan=srs.project_scan(p,[0,0,1],0.,sensor_relative_band_m=(-.2,.0),
            angle_increment=np.deg2rad(1.),fov_deg=360.,range_min=.1)
        self.assertEqual(scan['band_point_count'],1)
        self.assertEqual(scan['finite_count'],1)

    def test_fov_must_align_to_increment(self):
        with self.assertRaises(ValueError):
            srs.project_scan([[2,0,0]],[0,0,1],1.,fov_deg=100.,angle_increment=np.deg2rad(3.))

    def test_invalid_inputs_rejected(self):
        with self.assertRaises(ValueError):srs.project_scan([[1,2]], [0,0,1], 1.)
        with self.assertRaises(ValueError):srs.project_scan([[1,2,3]], [0,0,0], 1.)


if __name__=='__main__':unittest.main()
