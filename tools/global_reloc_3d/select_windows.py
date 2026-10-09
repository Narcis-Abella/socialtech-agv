"""Administrative, evaluator-only window selection for spatial generalization runs.

Reads the reference trajectory to choose WHERE to test, never what the method
receives: only the bag offset and header start time are passed to the capture.
Rule (fixed before any run): walk the trajectory from `skip` s; open a window
whenever the reference XY is >= `spacing` m from every window already chosen
and the next `duration` s of reference exist without gaps > 0.2 s.
"""
import argparse
import csv
from pathlib import Path

import numpy as np
import yaml

from offline_s2 import map_transform


def select(rows, T_map_world, skip, spacing, duration, limit):
    t = rows[:, 0]
    xy = (rows[:, 1:4] @ T_map_world[:3, :3].T + T_map_world[:3, 3])[:, :2]
    chosen = []
    for i in np.flatnonzero(t >= t[0] + skip):
        j = np.searchsorted(t, t[i] + duration)
        if j >= len(t) or np.max(np.diff(t[i:j + 1])) > .2:
            continue
        if all(np.linalg.norm(xy[i] - xy[k]) >= spacing for k in chosen):
            chosen.append(i)
        if len(chosen) == limit:
            break
    return [(float(t[i]), xy[i].tolist()) for i in chosen]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--reference', required=True)
    p.add_argument('--map-yaml', required=True)
    p.add_argument('--bag-metadata', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--skip', type=float, default=15.)
    p.add_argument('--spacing', type=float, default=10.)
    p.add_argument('--duration', type=float, default=7.)
    p.add_argument('--preroll', type=float, default=3.)
    p.add_argument('--limit', type=int, default=12)
    a = p.parse_args()
    meta = yaml.safe_load(Path(a.bag_metadata).read_text())['rosbag2_bagfile_information']
    bag_start = meta['starting_time']['nanoseconds_since_epoch'] * 1e-9
    rows = np.loadtxt(a.reference, usecols=range(8))
    windows = select(rows, map_transform(a.map_yaml), a.skip, a.spacing, a.duration, a.limit)
    with open(a.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['window', 'start_header_s', 'capture_offset_s', 'duration_s', 'evaluator_only_x', 'evaluator_only_y'])
        for k, (t, xy) in enumerate(windows):
            w.writerow([f'w{k:02d}', f'{t:.9f}', f'{t - a.preroll - bag_start:.6f}', a.duration, f'{xy[0]:.3f}', f'{xy[1]:.3f}'])
    print(Path(a.out).read_text())


if __name__ == '__main__':
    main()
