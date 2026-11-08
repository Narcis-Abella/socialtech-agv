"""The 2D occupancy map the full-chain simulation localizes on, from the same triangle soup the simulated Mid-360 sees (world_to_tris.py): obstacle = surface in the band the robot's /scan
uses (0.10 .. 1.80 m over the floor, socialtech_bringup/config/scan.yaml), free = what can be reached from the seeds without crossing an obstacle, the rest unknown (what make_map.sh gives the real
robot). The old loyola.pgm was surveyed with the simulator's 2D lidar: 0.1-0.3 m away from what the Mid-360 chain sees, which GN follows (constant offset against the truth).
usage: tris_to_map.py WORLD.tris OUT_PREFIX x,y [x,y ...]      writes OUT_PREFIX.pgm and OUT_PREFIX.yaml; the seeds are floor points inside the building (the spawn)."""
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage

from tris_to_prior import read_tris, sample

FREE, OCC, UNKNOWN = 0, 1, -1


def rasterize(tris, seeds, res=.05, band=(.10, 1.80), margin=1.0, spacing=None):
    """(image [row = y index, column = x index; FREE/OCC/UNKNOWN], origin (x, y) of cell (0, 0), res)."""
    pts = sample(tris, spacing or res / 2)
    xy = pts[(pts[:, 2] >= band[0]) & (pts[:, 2] <= band[1]), :2]
    lo, hi = pts[:, :2].min(axis=0) - margin, pts[:, :2].max(axis=0) + margin
    shape = tuple(int(n) for n in np.ceil((hi - lo) / res)[::-1] + 1)
    occ = np.zeros(shape, bool)
    idx = np.floor((xy - lo) / res).astype(int)
    occ[idx[:, 1], idx[:, 0]] = True
    occ = ndimage.binary_dilation(occ, iterations=1)                  # a closed outline: no diagonal leaks for the flood fill
    lab, _ = ndimage.label(~occ)
    # shortcut: the free space is cut to the bounding box of the largest connected obstacle (the outer walls): a rectangular building with an opening wider than the flood can close
    # (the simulated hospital's outline has one) would otherwise flood the outside. A non-rectangular building needs its footprint instead.
    ol, n = ndimage.label(occ)
    outer = ol == (1 + int(np.argmax(ndimage.sum(occ, ol, range(1, n + 1)))))
    ys, xs = np.where(outer)
    inside = np.zeros(shape, bool); inside[ys.min():ys.max() + 1, xs.min():xs.max() + 1] = True
    img = np.full(shape, UNKNOWN, np.int8)
    for sx, sy in seeds:
        j, i = int(round((sy - lo[1]) / res)), int(round((sx - lo[0]) / res))
        if occ[j, i]:
            raise ValueError(f'seed {(sx, sy)} is on an obstacle')
        img[(lab == lab[j, i]) & inside] = FREE
    img[occ] = OCC
    return img, (float(lo[0]), float(lo[1])), res


def to_pgm(img):
    """Image rows top-down in PGM convention (the highest y first): 254 free, 0 occupied, 205 unknown."""
    out = np.where(img == FREE, 254, np.where(img == OCC, 0, 205)).astype(np.uint8)
    return out[::-1]


def write(img, origin, res, prefix):
    pgm = to_pgm(img)
    with open(f'{prefix}.pgm', 'wb') as f:
        f.write(f'P5\n{pgm.shape[1]} {pgm.shape[0]}\n255\n'.encode()); f.write(pgm.tobytes())
    Path(f'{prefix}.yaml').write_text(f'image: {Path(prefix).name}.pgm\nmode: trinary\nresolution: {res}\norigin: [{origin[0]:.3f}, {origin[1]:.3f}, 0]\n'
                                      'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')


if __name__ == '__main__':
    seeds = [tuple(float(v) for v in a.split(',')) for a in sys.argv[3:]]
    img, origin, res = rasterize(read_tris(sys.argv[1]), seeds)
    write(img, origin, res, sys.argv[2])
    print(f'{img.shape[1]}x{img.shape[0]} cells, {int((img == FREE).sum()) * res * res:.0f} m2 free, origin {origin}')
