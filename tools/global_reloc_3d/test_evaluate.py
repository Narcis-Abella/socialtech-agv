import unittest
import numpy as np

from evaluate import error, reference_at


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


if __name__=='__main__':
    unittest.main()
