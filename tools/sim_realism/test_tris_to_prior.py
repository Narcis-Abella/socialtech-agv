import numpy as np

import tris_to_prior as tp


def test_a_unit_square_floor_gives_one_point_per_voxel():
    tris = np.array([[[0, 0, 0], [1, 0, 0], [1, 1, 0]], [[0, 0, 0], [1, 1, 0], [0, 1, 0]]], float)
    p = tp.voxelize(tp.sample(tris, .05), .2)
    assert len(p) == 25 or len(p) == 36          # a 1 m square in 0.2 m voxels (edge voxels shared with the neighbour cells)
    assert np.abs(p[:, 2]).max() < 1e-9 and p[:, :2].min() >= 0 and p[:, :2].max() <= 1.0


def test_voxel_centroid_is_inside_its_voxel():
    pts = np.array([[0.01, 0.01, 0.01], [0.19, 0.19, 0.19], [0.5, 0.5, 0.5]])
    v = tp.voxelize(pts, .2)
    assert len(v) == 2 and np.allclose(sorted(v.tolist())[0], [0.1, 0.1, 0.1])
