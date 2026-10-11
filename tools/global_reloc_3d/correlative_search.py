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


class MapField:
    """Likelihood field of the map on a grid padded by the source reach."""

    def __init__(self, points, res, sigma, reach_xy, reach_z):
        points = np.asarray(points, float)
        self.res = res
        lo = points.min(axis=0)
        hi = points.max(axis=0)
        pad = np.array([reach_xy, reach_xy, reach_z])
        self.origin = lo - pad
        self.shape = tuple(int(n) for n in np.ceil((hi + pad - self.origin) / res) + 1)
        # Sensor positions are restricted to the map box (no prior inside it).
        self.valid_lo = np.floor((lo - self.origin) / res).astype(int)
        self.valid_hi = np.ceil((hi - self.origin) / res).astype(int)
        occ = np.zeros(self.shape, bool)
        idx = np.floor((points - self.origin) / res).astype(int)
        occ[tuple(idx.T)] = True
        d = ndimage.distance_transform_edt(~occ) * res
        self.field = np.exp(-.5 * (d / sigma) ** 2).astype(np.float32)
        self.spectrum = fft.rfftn(self.field, workers=2)

    def correlate(self, source_points, yaw):
        """Return (score volume, number of source voxels) for one yaw."""
        rotated = np.asarray(source_points) @ rot_z(yaw).T
        cells = np.unique(np.floor(rotated / self.res).astype(int), axis=0)
        S = np.zeros(self.shape, np.float32)
        S[tuple((cells % np.array(self.shape)).T)] = 1.
        c = fft.irfftn(self.spectrum * np.conj(fft.rfftn(S, workers=2)), s=self.shape, workers=2)
        return c / len(cells), len(cells)

    def position(self, index):
        return self.origin + np.asarray(index) * self.res


def yaw_distance_deg(a, b):
    d = math.degrees(a - b)
    return abs((d + 180.) % 360. - 180.)


def search(map_points, source_points, res=.5, sigma=.5, yaw_step_deg=2.5, radius=15.,
           top=10, per_yaw=8, nms_xy=1., nms_yaw_deg=10., field=None):
    t0 = time.monotonic()
    source = crop_source(source_points, radius)
    if len(source) < 50:
        raise ValueError('source has too few points inside the search radius')
    reach_z = float(np.abs(source[:, 2]).max()) + res
    if field is None:
        field = MapField(map_points, res, sigma, radius + res, reach_z)
    t_field = time.monotonic() - t0
    lo, hi = field.valid_lo, field.valid_hi
    peaks = []
    yaws = np.radians(np.arange(0., 360., yaw_step_deg))
    for yaw in yaws:
        score, n = field.correlate(source, yaw)
        inside = score[lo[0]:hi[0] + 1, lo[1]:hi[1] + 1, lo[2]:hi[2] + 1]
        flat = inside.ravel()
        k = min(per_yaw * 20, flat.size)
        best = np.argpartition(-flat, k - 1)[:k]
        kept = []
        for i in best[np.argsort(-flat[best])]:
            idx = np.array(np.unravel_index(i, inside.shape)) + lo
            xyz = field.position(idx)
            if all(np.linalg.norm(xyz[:2] - other[:2]) > nms_xy for other in kept):
                kept.append(xyz)
                peaks.append({'yaw_rad': float(yaw), 'xyz': xyz.tolist(), 'score': float(flat[i]),
                              'source_voxels': n})
            if len(kept) == per_yaw:
                break
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
