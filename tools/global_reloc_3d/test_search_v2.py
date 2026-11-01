import math
import threading
import unittest

import numpy as np

from correlative_search import pose
from make_hole_map import hole
from search_v2 import rank, run_concurrently, source_reach_z, tilt_deg
from summarize_batch import classify, wilson


def row(i, support, T):
    return {'id': i, 'geometry': {'valid': True, 'support_015': support, 'matched_rmse_m': .05},
            'result': {'T_map_lidar_refined': T.tolist()}}


class SearchV2Test(unittest.TestCase):
    def test_reach_z_of_an_empty_crop_is_zero_instead_of_a_numpy_error(self):
        near = np.array([[1., 0., .5], [2., 0., -1.5]])
        far = np.array([[100., 0., 3.]])                              # outside the 15 m search radius: the crop is empty
        self.assertEqual(source_reach_z(near, far, 15.), 1.5)
        self.assertEqual(source_reach_z(far, far, 15.), 0.)           # a near-empty LiDAR frame: the search then rejects it cleanly

    def test_rank_keeps_distinct_hypotheses_only(self):
        a = row('a', .9, pose(0., [0, 0, 0]))
        same = row('same', .8, pose(math.radians(2), [.1, 0, 0]))  # within 0.30 m / 8 deg of a
        far = row('far', .7, pose(0., [5, 0, 0]))
        self.assertEqual([r['id'] for r in rank([far, same, a])], ['a', 'far'])

    def test_the_two_searches_run_at_the_same_time_and_both_results_come_back(self):
        barrier = threading.Barrier(2, timeout=5)                     # each waits for the other: a sequential run would break the barrier

        def search(name):
            barrier.wait()
            return name
        self.assertEqual(run_concurrently(lambda: search('primary'), lambda: search('confirmation')), ('primary', 'confirmation'))

    def test_an_error_in_either_search_is_raised_not_swallowed(self):
        def boom():
            raise ValueError('the search failed')
        with self.assertRaisesRegex(ValueError, 'the search failed'):
            run_concurrently(lambda: 1, boom)
        with self.assertRaisesRegex(ValueError, 'the search failed'):
            run_concurrently(boom, lambda: 1)

    def test_tilt(self):
        self.assertAlmostEqual(tilt_deg(pose(1., [0, 0, 0])), 0., places=6)
        T = np.eye(4)
        c, s = math.cos(math.radians(10)), math.sin(math.radians(10))
        T[1:3, 1:3] = [[c, -s], [s, c]]
        self.assertAlmostEqual(tilt_deg(T), 10., places=6)

    def test_negative_classes_never_reward_acceptance(self):
        self.assertEqual(classify('cross', {'accepted': True, 'decision': 'accepted'}, None), 'wrong_accept')
        self.assertEqual(classify('hole', {'accepted': False, 'decision': 'reject_no_hypothesis'}, None), 'reject')
        bad = {'refined': {'within_limits': False}}
        self.assertEqual(classify('hole', {'accepted': True, 'decision': 'accepted'}, bad), 'wrong_accept')
        self.assertEqual(classify('own', {'accepted': True, 'decision': 'accepted'}, bad), 'wrong_accept')
        self.assertEqual(classify('x01', {'accepted': False, 'decision': 'reject_geometry'}, bad), 'false_reject')

    def test_wilson_and_hole(self):
        self.assertAlmostEqual(wilson(0, 50)[1], .0714, places=3)
        pts = np.array([[0, 0, 0], [1, 0, 0], [9, 0, 0.]])
        self.assertEqual(len(hole(pts, (0, 0), 8.)), 1)


if __name__ == '__main__':
    unittest.main()
