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


class MapField:
    """Likelihood field of the map on a grid padded by the source reach.

    The correlation is the sum over the source's z layers of 2D correlations with the matching field layer, which is the 3D correlation with its z axis written
    out. Only the layers the source occupies are transformed and only the z shifts of the valid window are inverted (the window is 13 of 24 layers and the source
    6 on a real map). The xy FFT runs on the grid padded again, at the end, to a size the FFT does fast (the 191 of a real map is prime and costs several times
    more): the extra cells are zero and lie beyond the reach of every valid sensor position, so the scores of the valid window do not change."""

    def __init__(self, points, res, sigma, reach_xy, reach_z, workers=1, z_band=None):
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
        d = ndimage.distance_transform_edt(~occ) * res
        self.field = np.exp(-.5 * (d / sigma) ** 2).astype(np.float32)
        self.fft_xy = tuple(fft.next_fast_len(n, real=True) for n in self.shape[:2])
        self.layers = fft.rfft2(np.moveaxis(self.field, 2, 0), s=self.fft_xy, axes=(1, 2), workers=workers)     # (z, x, y spectrum)

    def correlate(self, source_points, yaw):
        """Return (score volume of the valid window, number of source voxels) for one yaw; the window indexes are field indexes minus valid_lo."""
        rotated = np.asarray(source_points) @ rot_z(yaw).T
        cells = unique_cells(rotated, self.res)
        kz = cells[:, 2]
        k0, k1 = int(kz.min()), int(kz.max())
        z0 = np.arange(self.valid_lo[2], self.valid_hi[2] + 1)
        if z0[0] + k0 < 0 or z0[-1] + k1 >= self.shape[2]:
            raise ValueError('source reaches outside the field in z')
        S = np.zeros((k1 - k0 + 1, *self.fft_xy), np.float32)
        S[kz - k0, cells[:, 0] % self.fft_xy[0], cells[:, 1] % self.fft_xy[1]] = 1.
        spectrum = fft.rfft2(S, axes=(1, 2), workers=self.workers)
        np.conjugate(spectrum, out=spectrum)
        acc = np.empty((len(z0), *self.layers.shape[1:]), self.layers.dtype)
        tmp = np.empty(self.layers.shape[1:], self.layers.dtype)
        for i, z in enumerate(z0):       # the sum over the source's layers, one layer at a time and in place, in the order of .sum(axis=0): no (layers x spectrum) temporary
            layers = self.layers[z + k0:z + k1 + 1]
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


def search(map_points, source_points, res=.5, sigma=.5, yaw_step_deg=2.5, radius=15.,
           top=10, per_yaw=8, nms_xy=1., nms_yaw_deg=10., field=None, z_band=None):
    t0 = time.monotonic()
    source = crop_source(source_points, radius)
    if len(source) < 50:
        raise ValueError('source has too few points inside the search radius')
    reach_z = float(np.abs(source[:, 2]).max()) + res
    if field is None:
        field = MapField(map_points, res, sigma, radius + res, reach_z, z_band=z_band)
    t_field = time.monotonic() - t0
    lo = field.valid_lo
    peaks = []
    yaws = np.radians(np.arange(0., 360., yaw_step_deg))
    for yaw in yaws:
        inside, n = field.correlate(source, yaw)
        for xyz, sc in yaw_peaks(inside, lo, field.origin, field.res, per_yaw, nms_xy):
            peaks.append({'yaw_rad': float(yaw), 'xyz': xyz.tolist(), 'score': sc, 'source_voxels': n})
    peaks.sort(key=lambda p: -p['score'])
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
            'field_seconds': t_field, 'seconds': time.monotonic() - t0,
            'score_quantiles_all_peaks': np.percentile(scores, [50, 90, 99, 100]).tolist(),
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
