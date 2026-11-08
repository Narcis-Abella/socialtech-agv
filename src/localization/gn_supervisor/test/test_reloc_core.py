import unittest
import numpy as np
from scipy.spatial.transform import Rotation
import reloc_core as rc



def T(x=0., yaw=0., y=0.):
    M = np.eye(4); M[:3, :3] = Rotation.from_euler('z', yaw).as_matrix(); M[:3, 3] = [x, y, 0]; return M



class HandoffTest(unittest.TestCase):
    def test_pose_propagates_with_odometry(self):
        T_map_src = T(10., .5, 4.); T_ci_first = T(0.); T_ci_now = T(1.5, .2)
        T_map_ci, xyyaw, q = rc.handoff_pose(T_map_src, np.eye(4), T_ci_first, T_ci_now)
        ref = T_map_src @ T_ci_now
        self.assertAlmostEqual(xyyaw[0], ref[0, 3]); self.assertAlmostEqual(xyyaw[2], .7)
        np.testing.assert_allclose(q, [0, 0, 0, 1], atol=1e-12)

    def test_mount_extrinsic(self):
        self.assertEqual(rc.compose_planar((1., 2., .3), (0., 0., 0.)), (1., 2., .3))       # default: behaviour of note 15 unchanged
        x, y, w = rc.compose_planar((1., 2., np.pi / 2), (.5, 0., .1))                      # base 0.5 m ahead of the sensor, robot facing +y
        self.assertAlmostEqual(x, 1.); self.assertAlmostEqual(y, 2.5); self.assertAlmostEqual(w, np.pi / 2 + .1)


if __name__ == "__main__":
    unittest.main()
