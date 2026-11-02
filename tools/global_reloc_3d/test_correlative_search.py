import math
import unittest

import numpy as np

from scipy import fft

from correlative_search import MapField, pose, search, spread_order, tenth_of_separated, unique_cells, yaw_distance_deg, yaw_peaks, z_cell_range, rot_z
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
        reach = float(np.abs(self.source[:, 2]).max()) + .5
        f = MapField(self.world, .5, .5, 9.5, reach)
        field = ReferenceMapField(self.world, .5, .5, 9.5, reach).field      # the field itself (the reference keeps it; MapField only keeps its spectrum)
        self.assertTrue(all(n >= m for n, m in zip(f.fft_xy, f.shape)))
        self.assertTrue(all(fft.next_fast_len(n, real=True) == n for n in f.fft_xy), f.fft_xy)
        lo, hi = f.valid_lo, f.valid_hi
        for yaw in (0., 1.1, 4.0):
            cells = np.unique(np.floor((self.source @ rot_z(yaw).T) / f.res).astype(int), axis=0)
            S = np.zeros(f.shape, np.float32)
            S[tuple((cells % np.array(f.shape)).T)] = 1.
            ref = fft.irfftn(fft.rfftn(field) * np.conj(fft.rfftn(S)), s=f.shape) / len(cells)
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


class ZBandTest(unittest.TestCase):
    def test_the_band_only_removes_sensor_heights_outside_it_and_keeps_the_scores_inside(self):
        rng = np.random.default_rng(5)
        world = synthetic_map(rng)
        T = pose(math.radians(40.), [14., 9., .5])
        src = transform(world, np.linalg.inv(T)); src = src[np.linalg.norm(src[:, :2], axis=1) < 9.]
        reach = float(np.abs(src[:, 2]).max()) + .5
        full = MapField(world, .5, .5, 9.5, reach)
        band = MapField(world, .5, .5, 9.5, reach, z_band=(.2, .9))
        self.assertLess(band.valid_hi[2] - band.valid_lo[2], full.valid_hi[2] - full.valid_lo[2])
        a, _ = full.correlate(src, 1.1)
        b, _ = band.correlate(src, 1.1)
        k = band.valid_lo[2] - full.valid_lo[2]
        np.testing.assert_allclose(b, a[:, :, k:k + b.shape[2]], atol=2e-5)
        zs = band.origin[2] + (np.arange(band.valid_lo[2], band.valid_hi[2] + 1)) * band.res
        self.assertTrue(zs.min() >= .2 - band.res and zs.max() <= .9 + band.res)

    def test_search_with_the_band_finds_the_same_pose(self):
        rng = np.random.default_rng(3)
        world = synthetic_map(rng)
        T = pose(math.radians(137.), [17.3, 8.6, .55])
        local = transform(world, np.linalg.inv(T)); local = local[np.linalg.norm(local[:, :2], axis=1) < 10.]
        a = search(world, local, res=.5, radius=10., top=5)['candidates'][0]
        b = search(world, local, res=.5, radius=10., top=5, z_band=(.25, .85))['candidates'][0]
        self.assertEqual(a['xyz'], b['xyz']); self.assertEqual(a['yaw_rad'], b['yaw_rad'])


class ReferenceMapField:
    """The search as it was before the CPU work (A1-A6): the plain layered correlation, kept here so that every later speed-up has to return the SAME candidates."""

    def __init__(self, points, res, sigma, reach_xy, reach_z, z_band=None):
        from scipy import ndimage
        points = np.asarray(points, float)
        self.res = res
        lo, hi = points.min(axis=0), points.max(axis=0)
        pad = np.array([reach_xy, reach_xy, reach_z])
        self.origin = lo - pad
        self.shape = tuple(int(n) for n in np.ceil((hi + pad - self.origin) / res) + 1)
        self.valid_lo = np.floor((lo - self.origin) / res).astype(int)
        self.valid_hi = np.ceil((hi - self.origin) / res).astype(int)
        if z_band is not None:
            self.valid_lo[2] = max(self.valid_lo[2], int(np.floor((z_band[0] - self.origin[2]) / res)))
            self.valid_hi[2] = min(self.valid_hi[2], int(np.ceil((z_band[1] - self.origin[2]) / res)))
        occ = np.zeros(self.shape, bool)
        occ[tuple(np.floor((points - self.origin) / res).astype(int).T)] = True
        d = ndimage.distance_transform_edt(~occ) * res
        field = self.field = np.exp(-.5 * (d / sigma) ** 2).astype(np.float32)
        self.fft_xy = tuple(fft.next_fast_len(n, real=True) for n in self.shape[:2])
        self.layers = fft.rfft2(np.moveaxis(field, 2, 0), s=self.fft_xy, axes=(1, 2), workers=2)

    def correlate(self, source_points, yaw):
        cells = unique_cells(np.asarray(source_points) @ rot_z(yaw).T, self.res)
        kz = cells[:, 2]
        k0, k1 = int(kz.min()), int(kz.max())
        z0 = np.arange(self.valid_lo[2], self.valid_hi[2] + 1)
        S = np.zeros((k1 - k0 + 1, *self.fft_xy), np.float32)
        S[kz - k0, cells[:, 0] % self.fft_xy[0], cells[:, 1] % self.fft_xy[1]] = 1.
        spectrum = np.conj(fft.rfft2(S, axes=(1, 2), workers=2))
        acc = np.empty((len(z0), *self.layers.shape[1:]), self.layers.dtype)
        for i, z in enumerate(z0):
            acc[i] = (self.layers[z + k0:z + k1 + 1] * spectrum).sum(axis=0)
        c = fft.irfft2(acc, s=self.fft_xy, axes=(1, 2), workers=2)
        lo, hi = self.valid_lo, self.valid_hi
        return np.moveaxis(c[:, lo[0]:hi[0] + 1, lo[1]:hi[1] + 1], 0, 2) / len(cells), len(cells)


def reference_candidates(map_points, source_points, res=.5, sigma=.5, yaw_step_deg=2.5, radius=15., top=10, per_yaw=8, nms_xy=1., nms_yaw_deg=10., z_band=None):
    source = np.asarray(source_points, float)
    source = source[np.hypot(source[:, 0], source[:, 1]) <= radius]
    field = ReferenceMapField(map_points, res, sigma, radius + res, float(np.abs(source[:, 2]).max()) + res, z_band)
    peaks = []
    for yaw in np.radians(np.arange(0., 360., yaw_step_deg)):
        inside, n = field.correlate(source, yaw)
        for xyz, sc in yaw_peaks(inside, field.valid_lo, field.origin, field.res, per_yaw, nms_xy):
            peaks.append({'yaw_rad': float(yaw), 'xyz': xyz.tolist(), 'score': sc, 'source_voxels': n})
    peaks.sort(key=lambda p: -p['score'])
    selected = []
    for p in peaks:
        if all(np.linalg.norm(np.subtract(p['xyz'][:2], q['xyz'][:2])) > nms_xy or yaw_distance_deg(p['yaw_rad'], q['yaw_rad']) > nms_yaw_deg for q in selected):
            selected.append(p)
        if len(selected) == top:
            break
    return selected


class SkipTest(unittest.TestCase):
    def test_spread_order_visits_every_yaw_once_coarse_first(self):
        o = spread_order(144)
        self.assertEqual(sorted(o), list(range(144)))
        self.assertEqual(o[:4], [0, 72, 36, 108])          # opposite yaws first, then quarter turns

    def test_tenth_of_separated_is_the_tenth_score_of_the_greedy_set_of_far_apart_peaks(self):
        # (score, yaw_rad, xyz): ten peaks 5 m apart, then clones 0.5 m from the best that must not count as new members
        peaks = [(1. - .01 * i, 0., [5. * i, 0., 0.]) for i in range(10)] + [(.99, 0., [.5, 0., 0.]), (.98, 0., [5.5, 0., 0.])]
        self.assertAlmostEqual(tenth_of_separated(peaks, 1., 10.), .91)
        self.assertEqual(tenth_of_separated(peaks[:9], 1., 10.), -np.inf)   # fewer than ten far-apart peaks: nothing can be skipped yet
        # a peak 1.9 m away at the same yaw is closer than 2 x nms_xy: not a new member; the same place 25 deg apart in yaw is (>2 x 10 deg)
        near = [(1., 0., [0., 0., 0.]), (.9, 0., [1.9, 0., 0.])] + [(.8 - .01 * i, 0., [10. + 5. * i, 0., 0.]) for i in range(8)]
        self.assertEqual(tenth_of_separated(near, 1., 10.), -np.inf)
        turned = [(1., 0., [0., 0., 0.]), (.9, math.radians(25.), [0., 0., 0.])] + [(.8 - .01 * i, 0., [10. + 5. * i, 0., 0.]) for i in range(8)]
        self.assertAlmostEqual(tenth_of_separated(turned, 1., 10.), .73)

    def test_the_tenth_selected_peak_never_scores_below_the_bar(self):
        # the property the skip rests on, on random dense peaks (many conflicts): select as search() does, compare the tenth with tenth_of_separated of the same peaks
        rng = np.random.default_rng(0)
        checked = 0
        for _ in range(400):
            n = int(rng.integers(12, 80))
            peaks = [(float(s), float(np.radians(rng.choice(np.arange(0., 360., 2.5)))), [float(x), float(y), 0.]) for s, x, y in zip(rng.uniform(0, 1, n), rng.uniform(0, 6, n), rng.uniform(0, 6, n))]
            bar = tenth_of_separated(peaks, 1., 10.)
            chosen = []
            for p in sorted(peaks, key=lambda p: -p[0]):
                if all(math.hypot(p[2][0] - q[2][0], p[2][1] - q[2][1]) > 1. or yaw_distance_deg(p[1], q[1]) > 10. for q in chosen):
                    chosen.append(p)
                if len(chosen) == 10:
                    break
            if bar > -math.inf:
                checked += 1
                self.assertEqual(len(chosen), 10)
                self.assertGreaterEqual(chosen[9][0], bar)
        self.assertGreater(checked, 100)             # the property was exercised, not vacuous

    def test_the_search_really_skips_yaws(self):
        rng = np.random.default_rng(3)
        world = synthetic_map(rng)
        local = transform(world, np.linalg.inv(pose(math.radians(137.), [17.3, 8.6, .55])))
        local = local[np.linalg.norm(local[:, :2], axis=1) < 9.] + rng.normal(0, .02, (1, 3))
        r = search(world, local, res=.5, radius=9., top=10)
        self.assertGreater(r['yaws_skipped'], 0)
        self.assertLess(r['yaws_skipped'], 144)


class LeanBuildTest(unittest.TestCase):
    def setUp(self):
        self.world = synthetic_map(np.random.default_rng(5))
        local = transform(self.world, np.linalg.inv(pose(math.radians(40.), [14., 9., .5])))
        self.source = local[np.linalg.norm(local[:, :2], axis=1) < 9.]
        self.reach = float(np.abs(self.source[:, 2]).max()) + .5

    def test_z_cell_range_is_the_floor_of_the_heights_inside_the_radius(self):
        a = np.array([[1., 0., -.7], [0., 2., 1.3], [30., 0., 9.]])          # the last point is outside the radius
        self.assertEqual(z_cell_range([a], .5, 15.), (-2, 2))
        b = np.array([[1., 0., 3.1]])
        self.assertEqual(z_cell_range([a, b], .5, 15.), (-2, 6))

    def test_only_the_layers_the_source_reaches_are_transformed_and_the_scores_do_not_change(self):
        full = MapField(self.world, .5, .5, 9.5, self.reach)
        zr = z_cell_range([self.source], .5, 9.)
        lean = MapField(self.world, .5, .5, 9.5, self.reach, z_cells=zr)
        self.assertLess(len(lean.layers), len(full.layers))
        for yaw in (0., 1.1, 4.0):
            a, na = full.correlate(self.source, yaw)
            b, nb = lean.correlate(self.source, yaw)
            self.assertEqual(na, nb)
            np.testing.assert_array_equal(a, b)
        high = self.source + [0., 0., 6.]                                    # a source taller than the range it was built for
        with self.assertRaises(ValueError):
            lean.correlate(high, 0.)

    def test_the_build_allocates_far_less_than_scipys_edt_does(self):
        import tracemalloc
        tracemalloc.start()
        f = MapField(self.world, .5, .5, 9.5, self.reach, z_cells=z_cell_range([self.source], .5, 9.))
        _, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        self.assertLess(peak / np.prod(f.shape), 36., 'bytes per cell at the peak (scipy edt + exp temporaries: 57; now ~30)')


class OneFftThreadTest(unittest.TestCase):
    def test_the_field_transforms_on_one_thread_by_default(self):
        # the primary and the confirmation search already run side by side: a second FFT thread each costs ~40 % more CPU (laptop, 144 yaws: 1.13 vs 0.80 CPU-s) for no gain in wall time
        f = MapField(np.random.default_rng(0).uniform(0, 5, (400, 3)), .5, .5, 2., 1.)
        self.assertEqual(f.workers, 1)


class SameCandidatesTest(unittest.TestCase):
    """The speed-ups of the search are exact: the ten candidates (yaw, xyz, score, source voxels) are those of the reference above, bit for bit."""

    def check(self, z_band, seed, yaw_deg, at):
        rng = np.random.default_rng(seed)
        world = synthetic_map(rng)
        T = pose(math.radians(yaw_deg), at)
        local = transform(world, np.linalg.inv(T))
        local = local[np.linalg.norm(local[:, :2], axis=1) < 9.] + rng.normal(0, .02, (1, 3))
        new = search(world, local, res=.5, radius=9., top=10, z_band=z_band)['candidates']
        old = reference_candidates(world, local, radius=9., z_band=z_band)
        self.assertEqual(len(new), len(old))
        for a, b in zip(new, old):
            for k in ('yaw_rad', 'xyz', 'score', 'source_voxels'):
                self.assertEqual(a[k], b[k], k)

    def test_without_z_band(self):
        self.check(None, 3, 137., [17.3, 8.6, .55])

    def test_with_z_band(self):
        self.check((.2, .9), 5, 40., [14., 9., .5])

    def test_another_pose(self):
        self.check((.1, 1.), 11, 275., [6.1, 14.2, .45])


if __name__ == '__main__':
    unittest.main()
