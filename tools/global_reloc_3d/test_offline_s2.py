import tempfile
import unittest
from pathlib import Path

import numpy as np

import offline_s2 as m


class TestOfflineS2(unittest.TestCase):
    def test_transform_inverse_and_frame(self):
        T = np.eye(4)
        T[:3, :3] = m.quat_matrix([0, 0, np.sin(0.4), np.cos(0.4)])
        T[:3, 3] = [1, 2, 3]
        self.assertTrue(np.allclose(T @ np.linalg.inv(T), np.eye(4)))
        p = np.array([[2., 0., 0.]])
        self.assertTrue(np.allclose(m.transform(p, T), p @ T[:3, :3].T + T[:3, 3]))

    def test_voxel_and_time(self):
        p = np.array([[.01, .01, .01], [.02, .01, .01], [1., 0., 0.]])
        self.assertEqual(len(m.voxel(p, .1)), 2)
        self.assertRaises(ValueError, m.voxel, p, 0)
        self.assertEqual(m.scan_end(10., np.array([0, 100000000], dtype=np.uint32)), 10.1)

    def test_map_metadata(self):
        with tempfile.TemporaryDirectory() as d:
            f = Path(d) / 'map.yaml'
            f.write_text('# T_map_world (row-major):\n# [1, 0, 0, 2]\n# [0, 1, 0, 3]\n# [0, 0, 1, 4]\n# [0, 0, 0, 1]\n')
            T = m.map_transform(f)
            self.assertTrue(np.allclose(T[:3, 3], [2, 3, 4]))

    def test_truth_isolation(self):
        self.assertNotIn('reference', m.prepare.__code__.co_varnames)
        self.assertNotIn('truth', m.prepare.__code__.co_varnames)


if __name__ == '__main__':
    unittest.main()
