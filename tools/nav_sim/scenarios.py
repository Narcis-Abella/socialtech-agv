"""Start/goal scenarios on an occupancy map (offline: numpy, scipy, PIL, yaml). Reference lengths come from Dijkstra on the map eroded by the robot clearance and shortened by line of sight,
so SPL has a near-Euclidean optimum instead of a grid one (8-connected grid paths are up to ~8 % longer)."""
import math
import os

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

import metrics


def load_map(yaml_path):
    """map_server trinary semantics: occupied above occupied_thresh, free below free_thresh, otherwise unknown."""
    m = yaml.safe_load(open(yaml_path))
    v = np.asarray(Image.open(os.path.join(os.path.dirname(yaml_path), m["image"])).convert("L"), float)
    p = (255 - v) / 255 if m.get("negate", 0) == 0 else v / 255
    occ, free = p > m["occupied_thresh"], p < m["free_thresh"]
    return {"occ": occ, "blocked": ~free, "res": float(m["resolution"]), "origin": (float(m["origin"][0]), float(m["origin"][1]))}


def to_cell(x, y, origin, res, h):
    return h - 1 - int(math.floor((y - origin[1]) / res)), int(math.floor((x - origin[0]) / res))


def to_xy(row, col, origin, res, h):
    return origin[0] + (col + 0.5) * res, origin[1] + (h - 1 - row + 0.5) * res


distance_to_occupied_cells = metrics.distance_to_occupied


def eroded_free(blocked, radius, res):
    """Cells whose centre is at least `radius` from every blocked cell."""
    return distance_to_occupied_cells(blocked, res) >= radius


_STEPS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


class _Grid:
    def __init__(self, free):
        self.free = free
        h, w = free.shape
        idx = np.arange(h * w).reshape(h, w)
        rows, cols, wts = [], [], []
        for dr, dc in _STEPS:
            r0, r1, c0, c1 = max(0, -dr), h - max(0, dr), max(0, -dc), w - max(0, dc)
            a, b = idx[r0:r1, c0:c1], idx[r0 + dr:r1 + dr, c0 + dc:c1 + dc]
            ok = free[r0:r1, c0:c1] & free[r0 + dr:r1 + dr, c0 + dc:c1 + dc]
            rows.append(a[ok])
            cols.append(b[ok])
            wts.append(np.full(ok.sum(), math.hypot(dr, dc)))
        self.graph = coo_matrix((np.concatenate(wts), (np.concatenate(rows), np.concatenate(cols))), shape=(h * w, h * w)).tocsr()
        self.shape = (h, w)

    def line_of_sight(self, a, b):
        n = int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) * 2)) + 1
        r = np.rint(np.linspace(a[0], b[0], n)).astype(int)
        c = np.rint(np.linspace(a[1], b[1], n)).astype(int)
        return bool(self.free[r, c].all())

    def shortest(self, a, b):
        """Length in cells of the shortened path, inf when unreachable."""
        w = self.shape[1]
        s, t = a[0] * w + a[1], b[0] * w + b[1]
        if not (self.free[a] and self.free[b]):
            return float("inf")
        d, pred = dijkstra(self.graph, indices=s, return_predecessors=True)
        if not math.isfinite(d[t]):
            return float("inf")
        cells, k = [], t
        while k != s:
            cells.append((k // w, k % w))
            k = pred[k]
        cells.append((s // w, s % w))
        cells.reverse()
        out, i = [cells[0]], 0
        while i < len(cells) - 1:
            j = len(cells) - 1
            while j > i + 1 and not self.line_of_sight(cells[i], cells[j]):
                j -= 1
            out.append(cells[j])
            i = j
        return sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(out, out[1:]))


def shortest_length(free, a, b, res):
    return _Grid(free).shortest(a, b) * res


def sample_pairs(blocked, n, seed, clearance, dmin, dmax, res, origin):
    """n reproducible start/goal pairs in the largest connected eroded-free region, Euclidean distance in [dmin, dmax], reachable; with the reference shortest length."""
    h = blocked.shape[0]
    free = eroded_free(blocked, clearance, res)
    lab, k = ndimage.label(free, structure=np.ones((3, 3)))
    if k == 0:
        raise ValueError("no free cell at that clearance")
    comp = lab == (1 + int(np.argmax(ndimage.sum(free, lab, range(1, k + 1)))))
    cells = np.argwhere(comp)
    grid, rng, pairs = _Grid(comp), np.random.default_rng(seed), []
    for _ in range(1000 * n):
        if len(pairs) == n:
            break
        a, b = cells[rng.integers(len(cells))], cells[rng.integers(len(cells))]
        (sx, sy), (gx, gy) = to_xy(a[0], a[1], origin, res, h), to_xy(b[0], b[1], origin, res, h)
        d = math.hypot(gx - sx, gy - sy)
        if not dmin <= d <= dmax:
            continue
        length = grid.shortest(tuple(a), tuple(b)) * res
        if not math.isfinite(length):
            continue
        yaws = np.round(rng.uniform(-math.pi, math.pi, 2), 3)
        pairs.append({"id": f"p{len(pairs):02d}", "start": [round(sx, 3), round(sy, 3), float(yaws[0])], "goal": [round(gx, 3), round(gy, 3), float(yaws[1])],
                      "euclid": round(d, 3), "shortest": round(length, 3)})
    if len(pairs) < n:
        raise RuntimeError(f"only {len(pairs)} of {n} pairs found")
    return pairs
