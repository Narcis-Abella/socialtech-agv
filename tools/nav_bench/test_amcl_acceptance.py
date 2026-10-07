#!/usr/bin/env python3
"""Unit tests for the sensor/map-only AMCL acceptance gate."""
import math
import unittest

import numpy as np

import amcl_acceptance as aa
import amcl_evaluator as ae
import window_contract as wc


class AcceptanceTests(unittest.TestCase):
    def test_angle_wrap(self):
        self.assertAlmostEqual(aa.angle_error(-math.pi + .01, math.pi - .01), .02)

    def test_particle_concentration_and_alternative(self):
        dominant=np.tile([1.,2.,math.pi-.01],(95,1))
        alternative=np.tile([3.,2.,-math.pi+.01],(5,1))
        particles=np.c_[np.vstack([dominant,alternative]),np.ones(100)]
        self.assertAlmostEqual(aa.concentration(particles,[1.,2.,-math.pi+.01]),.95)
        primary,runner=aa.particle_modes(particles)
        self.assertAlmostEqual(primary.weight,.95)
        self.assertAlmostEqual(runner.weight,.05)
        self.assertGreater(math.hypot(primary.x-runner.x,primary.y-runner.y),.30)

    def test_geometry_gate_rejects_sparse_beams(self):
        scan=aa.Scan(np.full(49,2.),-1.,.05,.1,10.)
        result=aa.beam_coverage(scan)
        self.assertFalse(result.passes)
        self.assertEqual(result.finite_count,49)

    def test_s1_fit_check_partition_is_deterministic_and_70_30(self):
        scan=aa.Scan(np.arange(720,dtype=float),-math.pi,.01,.1,1000.)
        fit,check=aa.split_s1_scan(scan)
        self.assertEqual(int(np.isfinite(fit.ranges).sum()),504)
        self.assertEqual(int(np.isfinite(check.ranges).sum()),216)
        fit2,check2=aa.split_s1_scan(scan)
        np.testing.assert_array_equal(fit.ranges,fit2.ranges)
        np.testing.assert_array_equal(check.ranges,check2.ranges)

    def test_geometry_support_and_runner_up_margin(self):
        grid=np.zeros((80,80),dtype=np.int16)
        grid[20:60,40]=100
        occ=aa.Occupancy(grid,.05,-2.,-2.,0.)
        angles=np.linspace(-math.pi/2,math.pi/2,181)
        ranges=np.full(len(angles),np.inf)
        # Synthetic wall is 1 m to the right of pose (-1, 0, 0).
        mask=np.abs(angles)<math.radians(12)
        ranges[mask]=1./np.cos(angles[mask])
        scan=aa.Scan(ranges,-math.pi/2,math.pi/180,.1,10.)
        score=aa.map_support(scan,[-1.,0.,0.],occ)
        self.assertGreater(score,.7)
        shifted=aa.map_support(scan,[-1.,1.,0.],occ)
        self.assertLess(shifted,score)

    def test_map_support_uses_exact_point_to_cell_distance(self):
        grid=np.zeros((10,10),dtype=np.int16);grid[5,6]=100
        occ=aa.Occupancy(grid,.05,0.,0.,0.)
        y=.275
        def one(distance):
            scan=aa.Scan(np.array([distance]),0.,1.,.1,10.)
            return aa.map_support(scan,[0.,y,0.],occ)
        self.assertEqual(one(.15),1.)
        self.assertEqual(one(.149999),0.)

    def test_gate_abstains_without_a_searched_distinct_alternative(self):
        angles=np.linspace(-math.pi/2,math.pi/2,181)
        ranges=np.full(len(angles),2.)
        scan=aa.Scan(ranges,-math.pi/2,math.pi/180,.1,10.)
        grid=np.zeros((121,121),dtype=np.int16)
        cells_x=np.floor((2*np.cos(angles)+3)/.05).astype(int)
        cells_y=np.floor((2*np.sin(angles)+3)/.05).astype(int)
        grid[cells_y,cells_x]=100
        occ=aa.Occupancy(grid,.05,-3.,-3.,0.)
        particles=np.tile([0.,0.,0.,1.],(100,1))
        result=aa.evaluate(scan,particles,[0.,0.,0.],occ)
        self.assertGreaterEqual(result.particle_concentration,.90)
        self.assertGreaterEqual(result.primary_support,.70)
        self.assertEqual(result.reason,'runner_up_margin')
        self.assertFalse(result.passes)

    def test_s2_requires_distinct_persistent_batches(self):
        tracker=aa.AcceptanceTracker('S2')
        gate=aa.GateResult(True,'ok',.95,.8,.1,100,math.pi)
        self.assertIsNone(tracker.observe(1_000_000_000,0.,[1.,0.,0.],gate))
        with self.assertRaises(ValueError):
            tracker.observe(1_000_000_000,1.,[1.,0.,0.],gate)
        self.assertIsNone(tracker.observe(1_500_000_000,1.5,[1.,0.,0.],gate))
        accepted=tracker.observe(2_000_000_000,2.,[1.,0.,0.],gate)
        self.assertEqual(accepted.sensor_stamp_ns,2_000_000_000)
        self.assertEqual(accepted.confirmation_count,3)

    def test_s2_resets_when_pairwise_pose_spread_exceeds_gate(self):
        tracker=aa.AcceptanceTracker('S2')
        gate=aa.GateResult(True,'ok',.95,.8,.1,100,math.pi)
        tracker.observe(1_000_000_000,0.,[0.,0.,0.],gate)
        tracker.observe(1_500_000_000,.5,[.15,0.,0.],gate)
        # Each endpoint is within .20 m of the first, but their pairwise spread is .30 m.
        self.assertIsNone(tracker.observe(2_000_000_000,1.,[-.15,0.,0.],gate))
        self.assertEqual(len(tracker.samples),1)

    def test_s1_single_observation_and_timeout(self):
        tracker=aa.AcceptanceTracker('S1')
        gate=aa.GateResult(True,'ok',.95,.8,.1,100,math.pi)
        self.assertIsNotNone(tracker.observe(10,0.,[0.,0.,0.],gate))
        self.assertEqual(aa.timeout_outcome(False,59.9),'running')
        self.assertEqual(aa.timeout_outcome(False,60.),'timeout')
        self.assertEqual(aa.timeout_outcome(True,60.),'accepted')

    def test_truth_classification_is_evaluator_only(self):
        self.assertEqual(ae.classify(True,.30,math.radians(8)), 'correct_accept')
        self.assertEqual(ae.classify(True,.30,2*math.pi+math.radians(8)), 'correct_accept')
        self.assertEqual(ae.classify(True,.300001,0.), 'wrong_accept')
        self.assertEqual(ae.classify(False,None,None), 'false_reject_timeout')
        self.assertEqual(ae.classify(False,None,None,negative=True), 'correct_reject')
        self.assertNotIn('reference',aa.evaluate.__code__.co_varnames)
        self.assertFalse(hasattr(aa,'classify'))
        packet={'kind':'lidar','sensor_stamp_ns':123,'payload':b'cdr'}
        self.assertEqual(wc.validate_method_packet(packet),packet)
        with self.assertRaises(ValueError):
            wc.validate_method_packet({**packet,'evaluator_only_true_pose':[1.,2.,3.]})


if __name__=='__main__':
    unittest.main()
