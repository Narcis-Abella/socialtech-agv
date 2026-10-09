import math
import unittest

import numpy as np

from correlative_search import pose
from make_hole_map import hole
from search_v2 import rank, tilt_deg
from summarize_batch import classify, wilson


def row(i, support, T):
    return {'id': i, 'geometry': {'valid': True, 'support_015': support, 'matched_rmse_m': .05},
            'result': {'T_map_lidar_refined': T.tolist()}}


class SearchV2Test(unittest.TestCase):
    def test_rank_keeps_distinct_hypotheses_only(self):
        a = row('a', .9, pose(0., [0, 0, 0]))
        same = row('same', .8, pose(math.radians(2), [.1, 0, 0]))  # within 0.30 m / 8 deg of a
        far = row('far', .7, pose(0., [5, 0, 0]))
        self.assertEqual([r['id'] for r in rank([far, same, a])], ['a', 'far'])

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

    def test_wilson_and_hole(self):
        self.assertAlmostEqual(wilson(0, 50)[1], .0714, places=3)
        pts = np.array([[0, 0, 0], [1, 0, 0], [9, 0, 0.]])
        self.assertEqual(len(hole(pts, (0, 0), 8.)), 1)


if __name__ == '__main__':
    unittest.main()
