"""The 2D occupancy map: rasterize a levelled cloud, free the unknown cells the beams crossed, write the PGM + map_server YAML. numpy + scipy."""
from pathlib import Path

import numpy as np
from scipy.ndimage import binary_closing, label, maximum_filter
from scipy.spatial import cKDTree

from floor_level import OBST_HI, OBST_LO

OCC, FREE, UNK = 0, 254, 205
STRAY_PTS = 5           # points a 1 m cell needs to count for the map extent (walls and floor have hundreds)
MIN_HITS = 2            # obstacle points an occupied cell needs
CLOSE_M = 0.30          # m: gaps in the free space up to about this size are closed
HALO = (0.10, 0.10)     # m, ratio: an occupied cell with < ratio x the points of the densest cell within the radius is wall fuzz (r03)
ISLAND_CELLS_5CM = 10   # cells of the smallest occupied island kept, counted in 5 cm cells (0 = keep every island)
HEIGHTS = (0.3, 0.9, 1.5)               # m above the floor, where unknown cells are tested
NEAR, FILL_MARGIN = 8.0, 0.05           # m from the path; HIT margin of the fill test (m)
N_PASS, MIN_HEIGHTS, MAX_HIT_RATIO = 3, 2, 0.02


def island_cells(res):
    """The smallest occupied island kept, in cells of size res: the same physical speck size at any cell size (the area scales with res squared)."""
    return max(1, round(ISLAND_CELLS_5CM * (0.05 / res) ** 2)) if ISLAND_CELLS_5CM else 0


def rasterize(Q, res, band=(OBST_LO, OBST_HI)):
    """(grid, origin): grid[row, col] in OCC/FREE/UNK, row 0 = max y (PGM order); origin = (x, y) of the lower-left corner.
    band = (floor band, ceiling): points with |z| < band[0] count as floor, band[0] <= z <= band[1] as obstacle. Points alone in their 1 m cell (fewer than STRAY_PTS) are dropped:
    stray points far away would inflate the grid. Free space is closed over gaps of about CLOSE_M (the floor is seen sparsely); obstacles are never overwritten. An obstacle cell with
    fewer than HALO[1] x the points of the densest cell within HALO[0] is wall fuzz, not occupied; occupied islands of fewer than island_cells(res) cells (8-connected) are freed."""
    floor_band, ceil = band
    min_hits, fill, halo, min_component = MIN_HITS, CLOSE_M, HALO, island_cells(res)
    h = Q[:, 2]
    P = Q[(h > -floor_band) & (h <= ceil)]
    ij = np.floor(P[:, :2]).astype(np.int64)
    ij -= ij.min(0)
    _, inv, cnt = np.unique(ij[:, 0] * (ij[:, 1].max() + 1) + ij[:, 1], return_inverse=True, return_counts=True)
    P = P[cnt[inv.ravel()] >= STRAY_PTS]
    origin = (np.floor(P[:, 0].min() / res) * res, np.floor(P[:, 1].min() / res) * res)
    cols = ((P[:, 0] - origin[0]) / res).astype(int)
    rows = ((P[:, 1] - origin[1]) / res).astype(int)
    shape = (rows.max() + 1, cols.max() + 1)

    def count(sel):
        return np.bincount(rows[sel] * shape[1] + cols[sel], minlength=shape[0] * shape[1]).reshape(shape)

    obstacle = count(P[:, 2] >= floor_band)
    floor = count(np.abs(P[:, 2]) < floor_band)
    grid = np.full(shape, UNK, dtype=np.uint8)
    grid[floor > 0] = FREE
    if fill > 0:
        r = max(1, round(fill / res))
        y, x = np.ogrid[-r:r + 1, -r:r + 1]
        closed = binary_closing(grid == FREE, structure=(x * x + y * y <= r * r))
        grid[closed & (grid == UNK)] = FREE
    occ = obstacle >= min_hits
    if halo[0] > 0:
        r = max(1, round(halo[0] / res))
        y, x = np.ogrid[-r:r + 1, -r:r + 1]
        occ &= obstacle >= halo[1] * maximum_filter(obstacle, footprint=(x * x + y * y <= r * r))
    grid[occ] = OCC
    if min_component > 1:
        lab, _ = label(grid == OCC, structure=np.ones((3, 3)))
        grid[(lab > 0) & (np.bincount(lab.ravel())[lab] < min_component)] = FREE
    return grid[::-1], origin


def free_cells(hit, passed):
    """(cells, heights) beam counts -> bool per cell: the evidence rule of step 2."""
    seen = hit + passed > 0
    enough = ((passed >= N_PASS) | ~seen).all(1) & (seen.sum(1) >= MIN_HEIGHTS)
    h, p = hit.sum(1), passed.sum(1)
    return enough & (h <= MAX_HIT_RATIO * (h + p))


def connected_to_free(mask, grid):
    """The cells of mask (8-connected) that touch known free space, directly or through other cells of mask."""
    lab, _ = label(mask | (grid == FREE), structure=np.ones((3, 3)))
    return mask & np.isin(lab, np.unique(lab[grid == FREE]))


def fill_unknown(grid, res, origin, T, traj, count):
    """New grid (same layout: row 0 = max y) with the unknown cells freed: each unknown cell within NEAR of the path, at the HEIGHTS above the floor, becomes FREE when it is PASSED >= N_PASS times
    at every observed height (at least MIN_HEIGHTS), HITs are <= MAX_HIT_RATIO of the observations and it connects to cells that were already free. Only unknown cells change.
    origin = (x, y) of the lower-left corner; T: PLY -> map; traj: (n, 3) positions in the PLY frame; count(C, margin) -> (hit, passed) counts the beams per candidate C (n, 3, PLY frame)."""
    Rm, tm = T[:3, :3], T[:3, 3]
    rows, cols = np.nonzero(grid == UNK)
    cx, cy = origin[0] + (cols + .5) * res, origin[1] + (grid.shape[0] - 1 - rows + .5) * res
    near = cKDTree((np.asarray(traj) @ Rm.T + tm)[:, :2]).query(np.c_[cx, cy], distance_upper_bound=NEAR)[0] < np.inf
    rows, cols, cx, cy = rows[near], cols[near], cx[near], cy[near]
    n, nh = len(rows), len(HEIGHTS)
    if n == 0:
        return grid.copy()
    C = (np.column_stack([np.repeat(cx, nh), np.repeat(cy, nh), np.tile(HEIGHTS, n)]) - tm) @ Rm   # candidates in the PLY frame, cell-major
    hit, passed = count(C, FILL_MARGIN)
    mask = np.zeros(grid.shape, bool)
    free = free_cells(hit.reshape(n, nh), passed.reshape(n, nh))
    mask[rows[free], cols[free]] = True
    out = grid.copy()
    out[connected_to_free(mask, grid)] = FREE
    return out


def write_map(prefix, grid, res, origin, T=None):
    """PGM + map_server YAML. T (4x4, the PLY's frame -> the map's frame) goes in as a comment: map_server ignores it, you need it to relate waypoints to the SLAM world."""
    prefix = Path(prefix)
    Path(f"{prefix}.pgm").write_bytes(b"P5\n%d %d\n255\n" % (grid.shape[1], grid.shape[0]) + grid.tobytes())
    # free_thresh 0.196, not Nav2's 0.25: map_server reads a pixel as free when (255 - pixel) / 255 <= free_thresh, and unknown (205) is 0.19608
    Path(f"{prefix}.yaml").write_text(f"image: {prefix.name}.pgm\nmode: trinary\nresolution: {res}\norigin: [{origin[0]}, {origin[1]}, 0.0]\n"
                                           "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
                                           + ("" if T is None else "# T_map_world (row-major; p_map = T p_ply):\n" + "".join(f"# {row}\n" for row in np.asarray(T).tolist())))
