import math
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from correlative_search import pose
from handoff_pose import handoff
from lio_submap import level_rotation


class HandoffTest(unittest.TestCase):
    def test_frames_compose_and_level_has_no_yaw(self):
        R_tilt = Rotation.from_euler('xy', [2., -1.5], degrees=True).as_matrix()  # camera_init tilted vs gravity
        T_ci_first = np.eye(4)
        T_ci_first[:3, 3] = [.02, -.01, 0]
        T_level = np.eye(4)
        T_level[:3, :3] = level_rotation(R_tilt.T @ [0, 0, 1.])
        T_map_src = pose(math.radians(40), [3., -2., .5])
        T_map_ci, q = handoff(T_map_src, T_level, T_ci_first)
        self.assertTrue(np.allclose(T_map_ci @ T_ci_first, T_map_src @ T_level))
        R = Rotation.from_quat(q).as_matrix()
        self.assertTrue(np.allclose(R @ (R_tilt.T @ [0, 0, 1.]), [0, 0, 1], atol=1e-9))
        x = R @ [1., 0, 0]
        self.assertLess(abs(math.degrees(math.atan2(x[1], x[0]))), .1)


if __name__ == '__main__':
    unittest.main()
