"""Wake of a dynamic obstacle in the global costmap: lethal cells that are free in the static map and farther than `radius` from every real moving obstacle (ground truth). Pure numpy; trials.py samples it."""
import numpy as np


def wake_cells(costmap, static, width, height, origin, res, objects, radius=0.6):
    """costmap, static: flat OccupancyGrid data of the same geometry (row 0 = lowest y). Lethal = 100 in the costmap, free = 0 in the static map. objects: [(x, y)] of the real obstacles that may legitimately be there."""
    cm = np.asarray(costmap, np.int16).reshape(height, width)
    st = np.asarray(static, np.int16).reshape(height, width)
    rows, cols = np.nonzero((cm == 100) & (st == 0))
    if rows.size == 0:
        return 0
    x, y = origin[0] + (cols + 0.5) * res, origin[1] + (rows + 0.5) * res
    keep = np.ones(rows.size, bool)
    for ox, oy in objects:
        keep &= np.hypot(x - ox, y - oy) > radius
    return int(keep.sum())
