import unittest

import numpy as np

import pointcloud as m


class PointcloudTest(unittest.TestCase):
    def test_transform_is_rotation_then_translation(self):
        c, s = np.cos(0.8), np.sin(0.8)
        T = np.eye(4)
        T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
        T[:3, 3] = [1, 2, 3]
        p = np.array([[2., 0., 0.]])
        self.assertTrue(np.allclose(m.transform(p, T), p @ T[:3, :3].T + T[:3, 3]))

    def test_voxel_keeps_one_point_per_occupied_cell_and_refuses_a_zero_size(self):
        p = np.array([[.01, .01, .01], [.02, .01, .01], [1., 0., 0.]])
        self.assertEqual(len(m.voxel(p, .1)), 2)
        self.assertRaises(ValueError, m.voxel, p, 0)


class VoxelEquivalenceTest(unittest.TestCase):
    def test_voxel_equals_the_np_unique_axis0_formula_including_the_order(self):
        rng = np.random.default_rng(4)
        pts = np.vstack([rng.normal(0, 6, (4000, 3)), rng.normal(0, 6, (500, 3))[:100], [[-3.3, 0.1, 7.7]]])
        for size in (.2, .5):
            _, inv, cnt = np.unique(np.floor(pts / size).astype(np.int64), axis=0, return_inverse=True, return_counts=True)
            inv = inv.ravel()
            want = np.column_stack([np.bincount(inv, weights=pts[:, j]) for j in range(3)]) / cnt[:, None]
            np.testing.assert_allclose(m.voxel(pts, size), want, atol=1e-12)


if __name__ == '__main__':
    unittest.main()
