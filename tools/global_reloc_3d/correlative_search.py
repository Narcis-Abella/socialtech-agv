"""Truth-free exhaustive correlative search over yaw, x, y, z on gravity-levelled clouds.

Descriptor-free alternative to KISS candidate generation. Both clouds must be
levelled (map: T_map_world; source: IMU gravity), so only yaw is unknown in the
rotation. For each yaw the levelled source is voxelized and cross-correlated
with a likelihood field of the whole map via FFT; every translation in the map
box is scored. Score = mean field value at the source voxels, in [0, 1].
No pose prior: the whole map box and all yaws are searched.
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
from scipy import fft, ndimage


def rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def pose(yaw, t):
    T = np.eye(4)
    T[:3, :3] = rot_z(yaw)
    T[:3, 3] = t
    return T


def crop_source(points, radius):
    points = np.asarray(points, float)
    return points[np.hypot(points[:, 0], points[:, 1]) <= radius]


def unique_cells(points, res):
    """Distinct voxels (n, 3) of the points: np.unique on one integer key per voxel, which is several times faster than np.unique(axis=0)."""
    cells = np.floor(points / res).astype(np.int64)
    base = cells.min(axis=0)
    span = cells.max(axis=0) - base + 1
    key = np.unique(((cells[:, 0] - base[0]) * span[1] + (cells[:, 1] - base[1])) * span[2] + (cells[:, 2] - base[2]))
    kx, rem = np.divmod(key, span[1] * span[2])
    ky, kz = np.divmod(rem, span[2])
    return np.stack([kx, ky, kz], axis=1) + base


def z_cell_range(sources, res, radius):
    """(lowest, highest) z cell index of the points of the sources inside the search radius (the cells a rotation about z leaves where they are), None when there are no points."""
    zs = np.concatenate([crop_source(s, radius)[:, 2] for s in sources])
    if not len(zs):                 # a near-empty frame: no range (every layer), so that the search itself rejects it ('too few points') as before
        return None
    return int(np.floor(zs.min() / res)), int(np.floor(zs.max() / res))


def yaw_peaks(inside, lo, origin, res, per_yaw, nms_xy):
    """The per_yaw best voxels of one yaw's score volume, at least nms_xy apart in xy, best first, as (position, score)."""
    flat = inside.ravel()
    k = min(per_yaw * 20, flat.size)
    best = np.argpartition(-flat, k - 1)[:k]
    best = best[np.argsort(-flat[best])]
    xyz = origin + (np.stack(np.unravel_index(best, inside.shape), axis=1) + lo) * res
    kept = np.empty((per_yaw, 3))
    chosen = []
    for j in range(k):
        if not chosen or (np.linalg.norm(kept[:len(chosen), :2] - xyz[j, :2], axis=1) > nms_xy).all():
            kept[len(chosen)] = xyz[j]
            chosen.append(j)
            if len(chosen) == per_yaw:
                break
    return [(xyz[j], float(flat[best[j]])) for j in chosen]


MIN_SOURCE_POINTS = 50


class SparseSource(ValueError):
    """This scene has too little to search: the next exposure may have more."""


class MapField:
    """Likelihood field of the map on a grid padded by the source reach.

    The correlation is the sum over the source's z layers of 2D correlations with the matching field layer, which is the 3D correlation with its z axis written
    out. Only the layers the source occupies are transformed and only the z shifts of the valid window are inverted (the window is 13 of 24 layers and the source
    6 on a real map). The xy FFT runs on the grid padded again, at the end, to a size the FFT does fast (the 191 of a real map is prime and costs several times
    more): the extra cells are zero and lie beyond the reach of every valid sensor position, so the scores of the valid window do not change.

    z_cells (lowest, highest z cell of the sources, z_cell_range) limits the transformed layers to those the sources can touch: the field then only serves sources inside that
    range (correlate raises ValueError otherwise). The build computes scipy's distance transform in slabs with the same operations as scipy 1.11 (feature transform, float64
    differences, squares, add.reduce, sqrt), so the field is bit-identical to the plain build; the reference copy in test_correlative_search.py fails if another scipy differs."""

    def __init__(self, points, res, sigma, reach_xy, reach_z, workers=1, z_band=None, z_cells=None):
        points = np.asarray(points, float)
        self.res, self.workers = res, workers
        lo = points.min(axis=0)
        hi = points.max(axis=0)
        pad = np.array([reach_xy, reach_xy, reach_z])
        self.origin = lo - pad
        self.shape = tuple(int(n) for n in np.ceil((hi + pad - self.origin) / res) + 1)
        # Sensor positions are restricted to the map box (no prior inside it).
        self.valid_lo = np.floor((lo - self.origin) / res).astype(int)
        self.valid_hi = np.ceil((hi - self.origin) / res).astype(int)
        if z_band is not None:       # sensor heights the robot can have (its mounting height +- a margin): the rest of the map box cannot hold the sensor
            self.valid_lo[2] = max(self.valid_lo[2], int(np.floor((z_band[0] - self.origin[2]) / res)))
            self.valid_hi[2] = min(self.valid_hi[2], int(np.ceil((z_band[1] - self.origin[2]) / res)))
            if self.valid_hi[2] < self.valid_lo[2]:
                raise ValueError('z_band outside the map box')
        occ = np.zeros(self.shape, bool)
        idx = np.floor((points - self.origin) / res).astype(int)
        occ[tuple(idx.T)] = True
        # scipy's distance_transform_edt allocates ~50 B per cell (the feature transform, then float64 differences, squares and the sum for all cells at once). Same operations, one x slab at
        # a time from a preallocated int32 feature transform: the same numbers (the reference test compares them bit for bit) for ~12 B per cell
        ft = np.empty((3, *self.shape), np.int32)
        ndimage.distance_transform_edt(~occ, return_distances=False, return_indices=True, indices=ft)
        del occ
        field = np.empty(self.shape, np.float32)
        gy, gz = np.indices(self.shape[1:], dtype=np.int32)
        for x in range(self.shape[0]):
            dt = np.stack([ft[0, x] - np.int32(x), ft[1, x] - gy, ft[2, x] - gz]).astype(np.float64)
            np.multiply(dt, dt, dt)
            d = np.sqrt(np.add.reduce(dt, axis=0)) * res
            field[x] = np.exp(-.5 * (d / sigma) ** 2)
        del ft
        self.fft_xy = tuple(fft.next_fast_len(n, real=True) for n in self.shape[:2])
        # the layers a source can touch from the valid window (z_cells = its lowest and highest z cell): the others are never read
        self.z_first, z_end = 0, self.shape[2]
        if z_cells is not None:
            self.z_first, z_end = max(0, self.valid_lo[2] + z_cells[0]), min(self.shape[2], self.valid_hi[2] + z_cells[1] + 1)
        self.layers = fft.rfft2(np.moveaxis(field[:, :, self.z_first:z_end], 2, 0), s=self.fft_xy, axes=(1, 2), workers=workers)     # (z, x, y spectrum)

    def correlate(self, source_points, yaw):
        """Return (score volume of the valid window, number of source voxels) for one yaw; the window indexes are field indexes minus valid_lo."""
        rotated = np.asarray(source_points) @ rot_z(yaw).T
        cells = unique_cells(rotated, self.res)
        kz = cells[:, 2]
        k0, k1 = int(kz.min()), int(kz.max())
        z0 = np.arange(self.valid_lo[2], self.valid_hi[2] + 1)
        if z0[0] + k0 < self.z_first or z0[-1] + k1 >= self.z_first + len(self.layers):
            raise ValueError('source reaches outside the field in z (or the layers this field was built for)')
        nx, ny = self.fft_xy
        # rfft2 is the real FFT along y then the complex FFT along x. A source occupies a few dozen of the nx rows (62 of 270 on a real map) and the real FFT of an empty row is zero, so only
        # the occupied rows are transformed along y and the rest of the spectrum is left at zero: the same numbers, a fraction of the work
        rows, ri = np.unique(cells[:, 0] % nx, return_inverse=True)
        S = np.zeros((k1 - k0 + 1, len(rows), ny), np.float32)
        S[kz - k0, ri, cells[:, 1] % ny] = 1.
        spectrum = np.zeros((k1 - k0 + 1, nx, ny // 2 + 1), self.layers.dtype)
        spectrum[:, rows] = fft.rfft(S, axis=2, workers=self.workers)
        spectrum = fft.fft(spectrum, axis=1, workers=self.workers, overwrite_x=True)
        np.conjugate(spectrum, out=spectrum)
        acc = np.empty((len(z0), *self.layers.shape[1:]), self.layers.dtype)
        tmp = np.empty(self.layers.shape[1:], self.layers.dtype)
        for i, z in enumerate(z0):       # the sum over the source's layers, one layer at a time and in place, in the order of .sum(axis=0): no (layers x spectrum) temporary
            layers = self.layers[z + k0 - self.z_first:z + k1 + 1 - self.z_first]
            np.multiply(layers[0], spectrum[0], out=acc[i])
            for k in range(1, len(spectrum)):
                np.multiply(layers[k], spectrum[k], out=tmp)
                acc[i] += tmp
        c = fft.irfft2(acc, s=self.fft_xy, axes=(1, 2), workers=self.workers)
        lo, hi = self.valid_lo, self.valid_hi
        return np.moveaxis(c[:, lo[0]:hi[0] + 1, lo[1]:hi[1] + 1], 0, 2) / len(cells), len(cells)

    def position(self, index):
        return self.origin + np.asarray(index) * self.res


def yaw_distance_deg(a, b):
    d = math.degrees(a - b)
    return abs((d + 180.) % 360. - 180.)


def spread_order(n):
    """0..n-1 coarse to fine (0, n/2, n/4, 3n/4, ...): a few yaws far apart first, so a good candidate is known early and the others can be skipped."""
    seen, order, step = set(), [], n
    while step >= 1:
        for i in range(0, n, step):
            if i not in seen:
                seen.add(i)
                order.append(i)
        step //= 2
    return order


def top_of_separated(peaks, nms_xy, nms_yaw_deg, top, *, slack=1e-6):
    """Score of the `top`-th best of a greedy set of peaks that are far apart: more than twice nms_xy in xy OR twice nms_yaw_deg in yaw (and `slack` more, for the rounding of the
    triangle inequality), as (score, yaw_rad, xyz); -inf while there are fewer than `top`. The final selection takes the `top` best peaks that are more than nms_xy OR nms_yaw_deg from
    those already taken. A peak that is within both of two members of this set would put those two within twice both of each other, which they are not: every selected peak
    blocks at most one member, each member is selected or blocked by a peak that scores at least as much. So `top` selected peaks score >= the value returned, and a yaw whose best
    score is below it cannot contribute to the selection. The cost grows with `top` (sorting and a Python loop per yaw: ~0.04 s per search at the default 10, ~0.2 s at 40): made for the default."""
    chosen = []
    for p in sorted(peaks, key=lambda p: -p[0]):
        if all(np.hypot(p[2][0] - q[2][0], p[2][1] - q[2][1]) > 2 * nms_xy + slack or yaw_distance_deg(p[1], q[1]) > 2 * nms_yaw_deg + slack for q in chosen):
            chosen.append(p)
            if len(chosen) == top:
                return p[0]
    return -math.inf


def search(map_points, source_points, res=.5, sigma=.5, yaw_step_deg=2.5, radius=15.,
           top=10, per_yaw=8, nms_xy=1., nms_yaw_deg=10., field=None, z_band=None):
    t0 = time.monotonic()
    source = crop_source(source_points, radius)
    if len(source) < MIN_SOURCE_POINTS:
        raise SparseSource('source has too few points inside the search radius')
    reach_z = float(np.abs(source[:, 2]).max()) + res
    if field is None:
        field = MapField(map_points, res, sigma, radius + res, reach_z, z_band=z_band, z_cells=z_cell_range([source], res, radius))
    t_field = time.monotonic() - t0
    lo = field.valid_lo
    peaks = []                      # (score, yaw_rad, xyz, yaw index, rank in the yaw, source voxels)
    yaws = np.radians(np.arange(0., 360., yaw_step_deg))
    if getattr(field, 'native', False):     # corr_cpp.CppMapField: the same loop in C++
        rows, skipped = field.loop(source, yaws, spread_order(len(yaws)), per_yaw, nms_xy, nms_yaw_deg, top)
        peaks = [(r[0], r[1], r[2:5].tolist(), int(r[5]), int(r[6]), int(r[7])) for r in rows]
    else:
        bar, skipped = -math.inf, 0
        for i in spread_order(len(yaws)):
            inside, n = field.correlate(source, yaws[i])
            if inside.max() < bar:      # no peak of this yaw can be among the selected ones (top_of_separated: the `top`-th score of far-apart peaks)
                skipped += 1
                continue
            for j, (xyz, sc) in enumerate(yaw_peaks(inside, lo, field.origin, field.res, per_yaw, nms_xy)):
                peaks.append((sc, float(yaws[i]), xyz.tolist(), i, j, n))
            bar = top_of_separated(peaks, nms_xy, nms_yaw_deg, top)
    peaks.sort(key=lambda p: (-p[0], p[3], p[4]))     # ties in the order a plain 0..360 sweep would have produced them
    peaks = [{'yaw_rad': p[1], 'xyz': p[2], 'score': p[0], 'source_voxels': p[5]} for p in peaks]
    selected = []
    for p in peaks:
        if all(np.linalg.norm(np.subtract(p['xyz'][:2], q['xyz'][:2])) > nms_xy or
               yaw_distance_deg(p['yaw_rad'], q['yaw_rad']) > nms_yaw_deg for q in selected):
            selected.append(p)
        if len(selected) == top:
            break
    for p in selected:
        p['T_map_source'] = pose(p['yaw_rad'], p['xyz']).tolist()
    scores = [p['score'] for p in peaks]
    return {'candidates': selected, 'grid_shape': list(field.shape), 'res_m': res, 'sigma_m': sigma,
            'yaw_step_deg': yaw_step_deg, 'radius_m': radius, 'source_points_used': int(len(source)),
            'field_seconds': t_field, 'seconds': time.monotonic() - t0, 'yaws_skipped': skipped,
            'score_quantiles_unskipped_peaks': np.percentile(scores, [50, 90, 99, 100]).tolist(),   # only the peaks of the yaws that were not skipped: biased high, a diagnostic
            'truth_used': False}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--map', required=True)
    p.add_argument('--source', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--res', type=float, default=.5)
    p.add_argument('--yaw-step', type=float, default=2.5)
    p.add_argument('--radius', type=float, default=15.)
    a = p.parse_args()
    r = search(np.loadtxt(a.map), np.loadtxt(a.source), a.res, yaw_step_deg=a.yaw_step, radius=a.radius)
    Path(a.out).write_text(json.dumps(r, indent=2) + '\n')
    print(json.dumps({k: v for k, v in r.items() if k != 'candidates'}, indent=2))


if __name__ == '__main__':
    main()
