"""map_check: the propagated pose must fit the map by itself; a refined pose that is a different pose is a failure."""
import unittest

import numpy as np

import map_check as mc


def room(seed=0):
    r = np.random.default_rng(seed)
    w = []
    for x0, y0, x1, y1 in ((0, 0, 8, 0), (8, 0, 8, 5), (8, 5, 3, 5), (3, 5, 3, 7), (3, 7, 0, 7), (0, 7, 0, 0)):
        n = 600
        u = r.random(n)[:, None]
        w.append(np.c_[x0 + u * (x1 - x0), y0 + u * (y1 - y0), r.random(n) * 2.])
    return np.concatenate(w)


def pose(x, y, yaw):
    T = np.eye(4); c, s = np.cos(yaw), np.sin(yaw)
    T[:2, :2] = [[c, -s], [s, c]]; T[:2, 3] = (x, y)
    return T


class MapCheckTest(unittest.TestCase):
    def setUp(self):
        self.map = room()
        self.T = pose(4., 2.5, .4)
        seen = self.map[np.linalg.norm(self.map[:, :2] - self.T[:2, 3], axis=1) < 6.]
        self.scan = (seen - self.T[:3, 3]) @ self.T[:3, :3]           # the same walls in the body frame

    def res(self, T, ok=True):
        return {'gicp_converged': ok, 'T_map_lidar_refined': T.tolist()}

    def test_a_pose_that_still_fits_passes(self):
        v = mc.judge(self.map, self.scan, self.T, self.res(self.T))
        self.assertTrue(v['ok']); self.assertGreater(v['geometry']['support_015'], .95)

    def test_small_odometry_error_absorbed_by_the_refinement_passes(self):
        v = mc.judge(self.map, self.scan, self.T @ pose(.1, 0., np.radians(2.)), self.res(self.T))
        self.assertTrue(v['ok'])

    def test_refinement_that_lands_on_a_different_pose_fails_even_if_it_fits(self):
        v = mc.judge(self.map, self.scan, self.T @ pose(.5, 0., 0.), self.res(self.T))        # 0.5 m away from the propagated pose
        self.assertEqual((v['ok'], v['reason']), (False, 'shift'))
        v = mc.judge(self.map, self.scan, self.T @ pose(0., 0., np.radians(12.)), self.res(self.T))
        self.assertEqual((v['ok'], v['reason']), (False, 'shift'))

    def test_scans_that_do_not_fit_the_map_fail(self):
        bad = self.T @ pose(3.4, 0., np.radians(70.))           # h27: the odometry carried the pose away
        v = mc.judge(self.map, self.scan, bad, self.res(bad))
        self.assertEqual((v['ok'], v['reason']), (False, 'geometry'))

    def test_no_or_unconverged_refinement_fails(self):
        self.assertEqual(mc.judge(self.map, self.scan, self.T, None)['reason'], 'solver')
        self.assertEqual(mc.judge(self.map, self.scan, self.T, self.res(self.T, ok=False))['reason'], 'solver')

    def test_batches_split_in_time_and_need_enough_scans(self):
        odom = [(t, pose(.1 * t, 0., 0.)) for t in np.arange(0., 4., .1)]
        clouds = [(t, self.scan[::20]) for t in np.arange(.5, 3.5, .1)]
        fit, check, first = mc.batches(odom, clouds)
        self.assertTrue(np.allclose(first, pose(.05, 0., 0.)))
        self.assertGreater(len(fit), len(check) * .5)
        with self.assertRaises(ValueError):
            mc.batches(odom, clouds[:3])


if __name__ == '__main__':
    unittest.main()
