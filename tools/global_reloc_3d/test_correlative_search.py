import math
import unittest

import numpy as np

from correlative_search import pose, search, yaw_distance_deg
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


if __name__ == '__main__':
    unittest.main()
