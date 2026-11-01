"""S3 windows that start with an IMU-certified cold FAST-LIO2 (truth-free).

For each stationary episode from imu_ready.py, conditioning = the last certified
2 s before motion: [last_ready - 2, last_ready). FAST-LIO2 starts there (capture
offset), the submap starts there too, and acquisition is capped at `duration`
(30 s); the online odometry stop (2.8 m / 6 m) applies in lio_submap.py.
Evaluator-only XY (for hole maps and reporting) is added with --reference.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import yaml

from offline_s2 import map_transform


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--ready', required=True)
    p.add_argument('--bag-metadata', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--duration', type=float, default=30.)
    p.add_argument('--reference')
    p.add_argument('--map-yaml')
    p.add_argument('--link')
    a = p.parse_args()
    meta = yaml.safe_load(Path(a.bag_metadata).read_text())['rosbag2_bagfile_information']
    bag_start = meta['starting_time']['nanoseconds_since_epoch'] * 1e-9
    episodes = json.loads(Path(a.ready).read_text())['episodes']
    rows = None
    if a.reference:
        rows = np.loadtxt(a.reference, usecols=range(8))
        T = map_transform(a.map_yaml)
        if a.link:
            T = np.asarray(json.loads(Path(a.link).read_text())['T_A_B']) @ T
    with open(a.out, 'w', newline='') as f:
        w = csv.writer(f, lineterminator='\n')
        w.writerow(['window', 'start_header_s', 'capture_offset_s', 'duration_s', 'evaluator_only_x', 'evaluator_only_y',
                    'capture_s', 'stationary_s'])
        for k, e in enumerate(episodes):
            start = e['last_ready_s'] - 2.
            x = y = ''
            if rows is not None:
                i = min(np.searchsorted(rows[:, 0], start), len(rows) - 1)
                x, y = (T[:3, :3] @ rows[i, 1:4] + T[:3, 3])[:2]
                x, y = f'{x:.3f}', f'{y:.3f}'
            w.writerow([f'c{k:02d}', f'{start:.9f}', f'{start - bag_start:.6f}', a.duration, x, y, a.duration + 1,
                        f"{e['last_ready_s'] - e['first_ready_s'] + 2:.1f}"])
    print(Path(a.out).read_text())


if __name__ == '__main__':
    main()
