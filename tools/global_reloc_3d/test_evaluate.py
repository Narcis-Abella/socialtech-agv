import unittest
import numpy as np

from evaluate import error, reference_at
from evaluate_online import classify


class EvaluatorTest(unittest.TestCase):
    def test_quaternion_interpolation_and_gap_guard(self):
        q0=[0,0,0,1]
        q1=[0,0,np.sin(np.pi/4),np.cos(np.pi/4)]
        rows=np.array([[1.,0,0,0,*q0],[1.1,1,0,0,*q1]])
        mid=reference_at(rows,1.05)
        self.assertTrue(np.allclose(mid[:3,3],[.5,0,0]))
        self.assertAlmostEqual(np.degrees(np.arctan2(mid[1,0],mid[0,0])),45.,places=5)
        self.assertRaises(ValueError,reference_at,rows,1.2)
        rows[1,0]=1.3
        self.assertRaises(ValueError,reference_at,rows,1.15)

    def test_xy_yaw_error(self):
        a=np.eye(4); b=np.eye(4)
        a[0,3]=.3
        self.assertAlmostEqual(error(a,b)[0],.3)
        a[:2,:2]=[[-1,0],[0,-1]]
        self.assertAlmostEqual(error(a,b)[1],180.)


class ClassTest(unittest.TestCase):
    """A success class needs a valid reference at the pose stamp. Nothing else may produce one."""
    ROWS = np.array([[t, 1., 2., 0., 0, 0, 0, 1] for t in (10., 10.1, 10.2, 11.0, 11.1)])

    def ref(self, t, rows=None):
        return reference_at(self.ROWS if rows is None else rows, t)

    def m(self, accepted=True, stamp=10.05, x=1., state='LOCALIZED', pose=True):
        return {'attempt': 1, 'state': state, 'reason': None, 'accepted': accepted,
                'initialpose': {'stamp': stamp, 'x': x, 'y': 2., 'yaw': 0.} if pose else None}

    def test_valid_reference(self):
        self.assertEqual(classify(self.m(), self.ref, False)['class'], 'correct_accept')
        self.assertEqual(classify(self.m(x=1.31), self.ref, False)['class'], 'wrong_accept')

    def test_no_success_without_a_reference(self):
        for stamp in (10.6, 9.9, 11.2, 10.2 + 1e-9, float('nan')):       # inside a gap, before the start, after the end, gap edge, bad stamp
            c = classify(self.m(stamp=stamp), self.ref, False)
            self.assertEqual(c['class'], 'unscorable_reference_gap', stamp)
            self.assertNotIn('initialpose_error', c)
        bad = self.ROWS.copy(); bad[1, 1] = np.nan                      # a reference row that is not a pose
        self.assertEqual(classify(self.m(), lambda t: self.ref(t, bad), False)['class'], 'unscorable_reference_gap')
        self.assertEqual(classify(self.m(stamp=10.2), self.ref, False)['class'], 'correct_accept')     # exactly on a sample, bracket 0.1 s

    def test_a_gap_keeps_the_class_but_the_pose_still_gets_a_labelled_number(self):
        rows = np.array([[t, 1., 2., 0., 0, 0, 0, 1] for t in (10., 10.1, 10.301, 10.4)])       # one sample missing: 0.201 s bracket
        narrow, wide = (lambda t: reference_at(rows, t)), (lambda t: reference_at(rows, t, .5))
        c = classify(self.m(stamp=10.2), narrow, False, wide)
        self.assertEqual(c['class'], 'unscorable_reference_gap')
        self.assertTrue(c['initialpose_error_wide_bracket']['within_limits'])
        w = classify(self.m(stamp=10.2, x=3.), narrow, False, wide)
        self.assertEqual(w['class'], 'unscorable_reference_gap')            # never promoted to a success or hidden
        self.assertFalse(w['initialpose_error_wide_bracket']['within_limits'])
        self.assertNotIn('initialpose_error_wide_bracket', classify(self.m(stamp=10.2), narrow, False))

    def test_negative_accept_is_wrong_even_in_a_gap(self):
        self.assertEqual(classify(self.m(stamp=10.6), self.ref, True)['class'], 'wrong_accept')      # F1 evaluator said "unscorable"
        self.assertEqual(classify(self.m(), self.ref, True)['class'], 'wrong_accept')
        self.assertEqual(classify(self.m(accepted=False, state='REJECTED', pose=False), self.ref, True)['class'], 'correct_reject')

    def test_accept_without_pose_and_pose_without_accept(self):
        self.assertEqual(classify(self.m(state='HANDOFF_FAILED', pose=False), self.ref, False)['class'], 'accepted_not_published')
        self.assertEqual(classify(self.m(accepted=False, state='ERROR'), self.ref, False)['class'], 'pose_without_acceptance')
        self.assertEqual(classify(self.m(accepted=False, state='ERROR', pose=False), self.ref, False)['class'], 'error_no_pose')
        self.assertEqual(classify(self.m(accepted=False, state='TIMEOUT', pose=False), self.ref, False)['class'], 'false_reject_timeout')


if __name__=='__main__':
    unittest.main()
