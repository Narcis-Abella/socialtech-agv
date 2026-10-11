import unittest

import numpy as np

from verify import decide, score


class VerifyTest(unittest.TestCase):
    def test_geometry_and_false_positive_abstention(self):
        pts = np.array([[x, y, z] for x in range(4) for y in range(4) for z in range(2)], float)
        g = score(pts, pts, np.eye(4))
        self.assertEqual(g['support_015'], 1.)
        solver = {'kiss_valid': True, 'gicp_converged': True}
        self.assertEqual(decide(solver, g), 'ambiguous_no_alternative_margin')
        self.assertEqual(decide(solver, g, .11), 'ambiguous_no_temporal_confirmation')
        self.assertEqual(decide(solver, g, .11, True), 'accepted')
        wrong = np.eye(4)
        wrong[0, 3] = 20
        self.assertEqual(decide(solver, score(pts, pts, wrong)), 'reject_geometry')


if __name__ == '__main__':
    unittest.main()
