#!/usr/bin/env python3
import unittest

import numpy as np

from scan_projection_sweep import configurations, summarize_config
from evaluator_scan_map_diagnostic import reference_at


class ProjectionSweepTests(unittest.TestCase):
    def test_fixed_grid_is_unique_and_spans_requested_height_range(self):
        rows=configurations()
        self.assertEqual(len(rows),99)
        self.assertEqual(len({r['config_id'] for r in rows}),len(rows))
        heights={r['height_m'] for r in rows if r['config_id'].startswith('h')}
        self.assertEqual(min(heights),.1)
        self.assertEqual(max(heights),2.)

    def test_summary_detects_empty_vertical_slice(self):
        config=next(r for r in configurations() if r['config_id']=='h10_b05_075')
        xyz=np.asarray([[2.,0.,2.],[0.,2.,2.]],float)
        result=summarize_config(config,[xyz,xyz],[0,0,1])
        self.assertEqual(result['finite_median'],0.)
        self.assertEqual(result['covered_fraction'],0.)

    def test_truth_bracket_is_bounded_and_never_nearest(self):
        t=np.asarray([100.,100.1,100.401])
        p=np.asarray([[0.,0.,0.],[1.,0.,0.],[2.,0.,0.]])
        self.assertAlmostEqual(reference_at(100_050_000_000,t,p)[0],0.5)
        self.assertIsNone(reference_at(100_200_000_000,t,p))
        self.assertIsNone(reference_at(99_999_000_000,t,p))


if __name__=='__main__':unittest.main()
