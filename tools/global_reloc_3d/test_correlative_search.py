import math
import unittest

import numpy as np

from scipy import fft

from correlative_search import MapField, pose, search, unique_cells, yaw_distance_deg, yaw_peaks, rot_z
from offline_s2 import transform


def synthetic_map(rng):
    """Asymmetric room with pillars: walls, floor, ceiling and boxes."""
    pts = []
    for x in np.arange(0, 30, .1):
        for z in np.arange(0, 3, .2):
            pts += [[x, 0, z], [x, 20, z]]
    for y in np.arange(0, 20, .1):
        for z in np.arange(0, 3, .2):
            pts += [[0, y, z], [30, y, z]]
    for cx, cy in rng.uniform([3, 3], [27, 17], size=(12, 2)):
        for a in np.linspace(0, 2 * math.pi, 24, endpoint=False):
            for z in np.arange(0, 3, .2):
                pts.append([cx + .3 * math.cos(a), cy + .3 * math.sin(a), z])
    g = np.mgrid[0:30:.3, 0:20:.3].reshape(2, -1).T
    pts += [[x, y, 0.] for x, y in g] + [[x, y, 3.] for x, y in g]
    return np.asarray(pts)


class CorrelativeSearchTest(unittest.TestCase):
    def test_recovers_known_pose_without_prior(self):
        rng = np.random.default_rng(3)
        world = synthetic_map(rng)
        T_true = pose(math.radians(137.), [17.3, 8.6, .55])
        local = transform(world, np.linalg.inv(T_true))
        local = local[np.linalg.norm(local[:, :2], axis=1) < 10.] + rng.normal(0, .02, (1, 3))
        r = search(world, local, res=.5, yaw_step_deg=2.5, radius=10., top=5)
        best = r['candidates'][0]
        self.assertLess(np.linalg.norm(np.subtract(best['xyz'][:2], [17.3, 8.6])), .6)
        self.assertLess(yaw_distance_deg(best['yaw_rad'], math.radians(137.)), 2.6)
        self.assertFalse(r['truth_used'])

    def test_yaw_distance_wraps(self):
        self.assertAlmostEqual(yaw_distance_deg(math.radians(359.), math.radians(1.)), 2.)


class ExactSpeedupsTest(unittest.TestCase):
    """The speed-ups must not change a score or a candidate: each is compared with the formula it replaced."""
    def setUp(self):
        rng = np.random.default_rng(5)
        self.world = synthetic_map(rng)
        T = pose(math.radians(40.), [14., 9., .5])
        local = transform(self.world, np.linalg.inv(T))
        self.source = local[np.linalg.norm(local[:, :2], axis=1) < 9.]

    def test_layered_padded_correlation_gives_the_3d_scores_inside_the_valid_window(self):
        f = MapField(self.world, .5, .5, 9.5, float(np.abs(self.source[:, 2]).max()) + .5)
        self.assertTrue(all(n >= m for n, m in zip(f.fft_xy, f.shape)))
        self.assertTrue(all(fft.next_fast_len(n, real=True) == n for n in f.fft_xy), f.fft_xy)
        lo, hi = f.valid_lo, f.valid_hi
        for yaw in (0., 1.1, 4.0):
            cells = np.unique(np.floor((self.source @ rot_z(yaw).T) / f.res).astype(int), axis=0)
            S = np.zeros(f.shape, np.float32)
            S[tuple((cells % np.array(f.shape)).T)] = 1.
            ref = fft.irfftn(fft.rfftn(f.field) * np.conj(fft.rfftn(S)), s=f.shape) / len(cells)
            got, n = f.correlate(self.source, yaw)
            self.assertEqual(n, len(cells))
            win = (slice(lo[0], hi[0] + 1), slice(lo[1], hi[1] + 1), slice(lo[2], hi[2] + 1))
            self.assertEqual(got.shape, ref[win].shape)
            np.testing.assert_allclose(got, ref[win], atol=2e-5)

    def test_unique_cells_is_np_unique_axis0(self):
        rng = np.random.default_rng(1)
        pts = rng.normal(0, 8, (5000, 3)) @ rot_z(.7).T
        pts = np.vstack([pts, pts[:700]])                       # repeats
        ref = np.unique(np.floor(pts / .5).astype(int), axis=0)
        got = unique_cells(pts, .5)
        self.assertEqual(len(got), len(ref))
        self.assertEqual({tuple(c) for c in got}, {tuple(c) for c in ref})

    def test_yaw_peaks_is_the_greedy_loop_it_replaced(self):
        rng = np.random.default_rng(2)
        inside = rng.random((40, 50, 7)).astype(np.float32)
        inside[10:13, 10:13, 3] = 2.                            # a plateau of ties, the case where order matters
        lo, origin, res = np.array([3, 4, 1]), np.array([-5., -6., 0.]), .5
        per_yaw, nms = 8, 1.
        flat = inside.ravel()
        k = min(per_yaw * 20, flat.size)
        best = np.argpartition(-flat, k - 1)[:k]
        kept = []
        for i in best[np.argsort(-flat[best])]:
            xyz = origin + (np.array(np.unravel_index(i, inside.shape)) + lo) * res
            if all(np.linalg.norm(xyz[:2] - o[:2]) > nms for o in kept):
                kept.append(xyz)
                if len(kept) == per_yaw:
                    break
        got = yaw_peaks(inside, lo, origin, res, per_yaw, nms)
        self.assertEqual(len(got), len(kept))
        for (xyz, score), want in zip(got, kept):
            np.testing.assert_array_equal(xyz, want)
            self.assertEqual(score, float(inside[tuple(np.round((want - origin) / res).astype(int) - lo)]))


if __name__ == '__main__':
    unittest.main()
