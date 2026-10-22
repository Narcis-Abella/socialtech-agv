"""Integrity of the recorded sensor streams, read straight from the bag (no ROS graph, no replay, the bag is opened read-only).

For /livox/imu and /livox/lidar: header-stamp gaps against the nominal period (missing samples already absent in the recording),
duplicates, non-monotonic stamps, and the recorder's receive-time spacing. Compare with what a subscriber counts during replay to
place a loss: here = at or before recording; only downstream = replay / DDS / subscriber.
usage: bag_stream_integrity.py BAG_DIR [--start HEADER_S --end HEADER_S] > report.json
"""
import argparse
import json
import struct
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader


def header_stamp(raw):
    sec, nsec = struct.unpack_from('<iI' if raw[1] == 1 else '>iI', raw, 4)   # CDR encapsulation header, then std_msgs/Header.stamp
    return sec + nsec * 1e-9


def analyse(stamps, log, start=None, end=None):
    s, g = np.asarray(stamps), np.asarray(log)
    if start is not None:
        keep = (s >= start) & (s < end)
        s, g = s[keep], g[keep]
    if len(s) < 3:
        return {'n': int(len(s))}
    d = np.diff(s)
    period = float(np.median(d[d > 0]))
    missing = np.clip(np.round(d / period).astype(int) - 1, 0, None)
    dl = np.diff(g)
    steps_ms, counts = np.unique(np.round(d * 4000) / 4, return_counts=True)       # 0.25 ms bins
    top = np.argsort(-counts)[:12]
    mean_rate = (len(s) - 1) / float(s[-1] - s[0])
    return {'n': int(len(s)), 'nominal_period_s': period, 'rate_hz': 1 / period, 'span_s': float(s[-1] - s[0]),
            # Two readings of the same stamps. If the median step is shorter than the mean step, the stamps are irregular and the
            # gap count below overstates the loss: (n-1)/span against the sensor's specified rate is the loss that is actually bounded.
            'mean_rate_hz': mean_rate, 'median_step_rate_hz': 1 / period,
            'header_step_ms_histogram_top12': {f'{steps_ms[i]:.2f}': int(counts[i]) for i in sorted(top)},
            'missing_by_header_gap': int(missing.sum()), 'missing_fraction': float(missing.sum() / (len(s) + missing.sum())),
            'gap_events': int((missing > 0).sum()), 'max_gap_s': float(d.max()), 'duplicates': int((d == 0).sum()),
            'nonmonotonic': int((d < 0).sum()),
            'gap_size_histogram': {str(k): int(v) for k, v in zip(*np.unique(missing[missing > 0], return_counts=True))},
            'header_step_s_p50_p99_max': [float(np.percentile(d, 50)), float(np.percentile(d, 99)), float(d.max())],
            'log_time_step_s_p50_p99_max': [float(np.percentile(dl, 50)), float(np.percentile(dl, 99)), float(dl.max())],
            'log_minus_header_s_p50_p99': [float(np.percentile(g - s, 50)), float(np.percentile(g - s, 99))]}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('bag')
    p.add_argument('--start', type=float)
    p.add_argument('--end', type=float)
    a = p.parse_args()
    data = {'/livox/imu': ([], []), '/livox/lidar': ([], [])}
    with AnyReader([Path(a.bag)]) as reader:
        conns = [c for c in reader.connections if c.topic in data]
        declared = {c.topic: c.msgcount for c in conns}
        for c, t_log, raw in reader.messages(connections=conns):
            data[c.topic][0].append(header_stamp(raw)); data[c.topic][1].append(t_log * 1e-9)
    out = {'bag': str(a.bag), 'window': [a.start, a.end], 'declared_message_count': declared}
    for topic, (s, g) in data.items():
        out[topic] = analyse(s, g, a.start, a.end)
    print(json.dumps(out, indent=2))


if __name__ == '__main__':
    main()
