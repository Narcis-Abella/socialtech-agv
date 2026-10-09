import unittest
import numpy as np
from lio_submap import interpolate, rebase


class LioJoinTest(unittest.TestCase):
    def test_rebase_and_timestamp_guard(self):
        T0=np.eye(4); T0[0,3]=10
        T1=np.eye(4); T1[0,3]=11
        p=np.array([[1.,2.,3.]])
        self.assertTrue(np.allclose(rebase(p,T0,T0),p))
        self.assertTrue(np.allclose(rebase(p,T0,T1),p+[1,0,0]))
        self.assertTrue(np.allclose(interpolate([(1.,T0),(1.1,T1)],1.05)[:3,3],[10.5,0,0]))
        self.assertRaises(ValueError,interpolate,[(1.,T0),(1.3,T1)],1.15)


if __name__=='__main__':
    unittest.main()
