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

    def test_newest_scans_misplaced_fail_only_under_the_tail_rule(self):
        moved = (self.scan - np.array([.4, 0., 0.])) @ pose(0., 0., np.radians(10.))[:3, :3]   # where a drifting odometry puts the newest scans
        v = mc.judge(self.map, self.scan, self.T, self.res(self.T), tail=moved)                 # recorded, not enforced
        self.assertTrue(v['ok']); self.assertLess(v['tail']['support_015'], .7)
        v = mc.judge(self.map, self.scan, self.T, self.res(self.T), tail=moved, tail_rule=True)
        self.assertEqual((v['ok'], v['reason']), (False, 'tail_geometry'))
        v = mc.judge(self.map, self.scan, self.T, self.res(self.T), tail=self.scan, tail_rule=True)
        self.assertTrue(v['ok'])
        v = mc.judge(self.map, self.scan, self.T, self.res(self.T), tail=None, tail_rule=True)   # too few newest scans to tell
        self.assertEqual((v['ok'], v['reason']), (False, 'tail_missing'))

    def test_tail_is_the_newest_half_second(self):
        odom = [(t, pose(.1 * t, 0., 0.)) for t in np.arange(0., 4., .1)]
        clouds = [(float(t), self.scan[::20]) for t in np.arange(.5, 3.5, .1)]
        first = mc.batches(odom, clouds)[2]
        tail = mc.tail_batch(odom, clouds, first)
        self.assertIsNotNone(tail)
        gap = mc.tail_batch(odom, clouds[:10] + clouds[-2:], first)                               # 2 scans in the last 0.5 s: the 3 newest are used
        self.assertTrue(np.array_equal(gap, mc.voxel(np.concatenate([mc.rebase(p, first, mc.interpolate(odom, t)) for t, p in [clouds[9]] + clouds[-2:]]), mc.VOXEL_M)))
        # placed through the relative odometry: a point of the newest scan sits ~0.29 m ahead of where the first scan saw it
        self.assertAlmostEqual(float(np.median(tail[:, 0]) - np.median(mc.voxel(self.scan[::20], mc.VOXEL_M)[:, 0])), .29 - .025, delta=.06)

    def test_fast_voxel_is_bit_identical_to_the_frozen_one(self):
        import offline_s2
        r = np.random.default_rng(3)
        for pts in (r.normal(0., 15., (50000, 3)), (r.random((20000, 3)) * [40., 3., 2.] - [20., 1., .3]).astype(np.float32), self.scan):
            a, b = offline_s2.voxel(pts, mc.VOXEL_M), mc.voxel(pts, mc.VOXEL_M)
            self.assertEqual(a.shape, b.shape); self.assertTrue(np.array_equal(a, b))
        self.assertEqual(mc.voxel(np.zeros((0, 3)), .2).shape, (0, 3))

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


class ServeTest(unittest.TestCase):
    def test_serve_answers_one_line_per_request_and_survives_a_bad_one(self):
        import json, subprocess, sys, tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            np.savetxt(d / 'map.xyz', room())
            np.savez(d / 'few.npz', T_map_ci=np.eye(4), odom_t=np.array([0., 1.]), odom_T=np.array([np.eye(4)] * 2),
                     cloud_t=np.array([.1, .2]), cloud_n=np.array([3, 3]), cloud_xyz=np.zeros((6, 3), np.float32))
            reqs = [{'input': str(d / 'missing.npz'), 'out': str(d / 'o1')}, {'input': str(d / 'few.npz'), 'out': str(d / 'o2')}]
            p = subprocess.run([sys.executable, '-B', str(Path(mc.__file__)), '--serve', '--map', str(d / 'map.xyz'), '--binary', '/nonexistent'],
                               input=''.join(json.dumps(r) + '\n' for r in reqs), capture_output=True, text=True, timeout=60)
            out = [json.loads(l) for l in p.stdout.splitlines()]
        self.assertEqual(p.returncode, 0)
        self.assertEqual([(o['ok'], o['reason']) for o in out], [(False, 'worker_FileNotFoundError'), (False, 'too_few_clouds')])


if __name__ == '__main__':
    unittest.main()
