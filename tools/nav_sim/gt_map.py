"""Occupancy map from 2D scans taken at exactly known poses (offline: numpy, PIL). No SLAM, so the map is registered to the simulated world by construction.
integrate(): each scan is {pose: [x, y, yaw], angle_min, angle_increment, range_min, range_max, ranges}; a beam with no return (inf) clears the cells up to range_max."""
import math
import os

import numpy as np
from PIL import Image


def integrate(scans, shape, res, origin, min_hits=2, hit_fraction=0.2, min_free=2):
    h, w = shape
    free_n, hit_n = np.zeros(shape, np.int32), np.zeros(shape, np.int32)
    for s in scans:
        x, y, yaw = s["pose"]
        r = np.asarray(s["ranges"], float)
        ang = s["angle_min"] + s["angle_increment"] * np.arange(len(r)) + yaw
        hit = np.isfinite(r) & (r >= s["range_min"]) & (r < s["range_max"] * 0.999)
        reach = np.where(hit, r, s["range_max"])                     # no return or beyond the range: the ray clears up to range_max
        usable = ~np.isnan(r) & ~(np.isfinite(r) & (r < s["range_min"]))   # NaN and returns inside the blind zone say nothing
        for i in np.flatnonzero(usable):
            end = reach[i] - (res if hit[i] else 0.0)
            if end <= 0:
                continue
            t = np.arange(0.0, end, res / 2)
            col = np.floor((x + t * math.cos(ang[i]) - origin[0]) / res).astype(int)
            row = h - 1 - np.floor((y + t * math.sin(ang[i]) - origin[1]) / res).astype(int)
            ok = (col >= 0) & (col < w) & (row >= 0) & (row < h)
            np.add.at(free_n, (row[ok], col[ok]), 1)
            if hit[i]:
                c = int(math.floor((x + r[i] * math.cos(ang[i]) - origin[0]) / res))
                rr = h - 1 - int(math.floor((y + r[i] * math.sin(ang[i]) - origin[1]) / res))
                if 0 <= c < w and 0 <= rr < h:
                    hit_n[rr, c] += 1
    occupied = (hit_n >= min_hits) & (hit_n >= hit_fraction * (hit_n + free_n))
    free = (free_n >= min_free) & ~occupied
    return {"occupied": occupied, "free": free, "hits": hit_n, "passes": free_n}


def write_map(est, prefix, res, origin):
    img = np.full(est["occupied"].shape, 205, np.uint8)
    img[est["free"]] = 254
    img[est["occupied"]] = 0
    Image.fromarray(img).save(prefix + ".pgm")
    open(prefix + ".yaml", "w").write(f"image: {os.path.basename(prefix)}.pgm\nmode: trinary\nresolution: {res}\norigin: [{origin[0]}, {origin[1]}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")


if __name__ == "__main__":
    import json
    import sys

    import scenarios as sc
    if len(sys.argv) != 4:
        raise SystemExit("usage: gt_map.py <scans.json> <out prefix> <reference map.yaml (grid size and origin)>")
    ref = sc.load_map(sys.argv[3])
    est = integrate(json.load(open(sys.argv[1]))["scans"], ref["occ"].shape, ref["res"], ref["origin"])
    write_map(est, sys.argv[2], ref["res"], ref["origin"])
    print(f"occupied {int(est['occupied'].sum())} cells, free {int(est['free'].sum())}, of {est['occupied'].size}")
