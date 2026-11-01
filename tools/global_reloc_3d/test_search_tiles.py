import unittest
import numpy as np
import tempfile
from pathlib import Path
from search_tiles import distinct, agreement, temporal_evidence, run_one, candidate_key


class SearchPolicyTest(unittest.TestCase):
    def test_a_candidate_without_matched_points_sorts_last_instead_of_raising(self):
        def cand(support, rmse, inl):
            return {'geometry': {'support_015': support, 'matched_rmse_m': rmse}, 'result': {'translation_inliers': inl}}
        rows = [cand(0., None, 5), cand(0., None, 9), cand(0., .05, 1), cand(.4, .2, 1)]
        ranked = sorted(rows, key=candidate_key)                      # equal support and a None rmse used to raise TypeError
        self.assertEqual([r['geometry']['support_015'] for r in ranked], [.4, 0., 0., 0.])
        self.assertEqual(ranked[1]['geometry']['matched_rmse_m'], .05)  # a measured rmse beats a missing one at equal support

    def test_distinct_and_agreement(self):
        a = np.eye(4)
        b = np.eye(4)
        b[0, 3] = .21
        self.assertFalse(distinct(a, b))
        self.assertFalse(agreement(a, b))
        b[0, 3] = .31
        self.assertTrue(distinct(a, b))
        c = np.eye(4)
        self.assertTrue(agreement(a, c))

    def test_one_second_ablation_cannot_accept(self):
        self.assertFalse(temporal_evidence([.0,.1,.2,.3,.4,.5,.6,.7,.8,.9],0.,1.)[0])
        self.assertTrue(temporal_evidence([.0,.5,1.,1.5,2.,2.5,2.9],0.,3.)[0])

    def test_single_matcher_call_has_deadline(self):
        with tempfile.TemporaryDirectory() as d:
            sleeper=Path(d)/'sleeper'
            sleeper.write_text('#!/bin/sh\nsleep 2\n')
            sleeper.chmod(0o755)
            self.assertEqual(run_one(sleeper,'a','b','c',.3,Path(d)/'result',.02)['exit_code'],124)


if __name__ == '__main__':
    unittest.main()
