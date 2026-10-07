"""Summarize an independently recorded stationary IMU-only MCAP bag.

Run with the existing mapper image; the bag is read-only and the output is a
small JSON report. This tool never reads map/reference or candidate results.
"""
import argparse
import hashlib
import json
import struct
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from window_sensor_index import header


def decode(raw):
    stamp, frame, off = header(raw)
    off += (-(off - 4)) % 8
    if len(raw) != off + 37 * 8:
        raise ValueError('Invalid Imu CDR length')
    return stamp, frame, struct.unpack_from('<3d', raw, off + 13 * 8), struct.unpack_from('<3d', raw, off + 25 * 8)


def describe(x):
    return {k: float(v) for k, v in zip(('mean','median','std','p90','p95','p99','p999','min','max'),
        (np.mean(x), np.median(x), np.std(x), *np.percentile(x, [90,95,99,99.9]), np.min(x), np.max(x)))}


parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('bag',type=Path)
parser.add_argument('out',type=Path)
args=parser.parse_args()
bag=args.bag
out=args.out
with AnyReader([bag]) as reader:
    connections = [c for c in reader.connections if c.topic == '/livox/imu']
    if len(connections) != 1 or connections[0].msgtype != 'sensor_msgs/msg/Imu':
        raise ValueError('Expected one /livox/imu sensor_msgs/msg/Imu connection')
    n = connections[0].msgcount
    if n < 2:
        raise ValueError('At least two IMU samples required')
    timestamps = np.empty(n, dtype=np.int64)
    storage = np.empty(n, dtype=np.int64)
    gyro = np.empty((n, 3), dtype=np.float64)
    accel = np.empty((n, 3), dtype=np.float64)
    frame_ids=set()
    for i, (_, t, raw) in enumerate(reader.messages(connections=connections)):
        ts, frame, g, a = decode(raw)
        frame_ids.add(frame)
        if i == 0:
            message=reader.deserialize(raw,connections[0].msgtype)
            if (message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec != ts or
                [message.angular_velocity.x,message.angular_velocity.y,message.angular_velocity.z] != list(g) or
                [message.linear_acceleration.x,message.linear_acceleration.y,message.linear_acceleration.z] != list(a)):
                raise ValueError('Fast CDR decoder disagrees with schema decoder')
        timestamps[i] = ts
        storage[i] = t
        gyro[i] = g
        accel[i] = a
    if i + 1 != n or not np.isfinite(gyro).all() or not np.isfinite(accel).all():
        raise ValueError('Missing/nonfinite IMU messages')
    raw_norm = np.linalg.norm(gyro, axis=1)
    bias = np.median(gyro, axis=0)
    residual_norm = np.linalg.norm(gyro - bias, axis=1)
    dt = np.diff(timestamps)
    if np.any(dt <= 0):
        raise ValueError('Duplicate or regressive IMU header timestamp')
    duration_s = (timestamps[-1] - timestamps[0]) / 1e9
    blocks = []
    # Fixed one-hour blocks anchored at the first sensor timestamp.
    block_ids = (timestamps - timestamps[0]) // 3_600_000_000_000
    for b in range(int(block_ids[-1]) + 1):
        mask = block_ids == b
        if mask.sum() == 0:
            continue
        gb = gyro[mask]
        ab = accel[mask]
        blocks.append({'hour':b,'n':int(mask.sum()),'gyro_axis_median':np.median(gb,axis=0).tolist(),
                       'raw_p95':float(np.percentile(np.linalg.norm(gb,axis=1),95)),
                       'residual_p95_global_bias':float(np.percentile(np.linalg.norm(gb-bias,axis=1),95)),
                       'accel_axis_median':np.median(ab,axis=0).tolist(),
                       'accel_variance_sum':float(np.var(ab,axis=0,ddof=1).sum())})
    # Non-overlapping 2 s windows anchored at the first sensor timestamp.
    median_delta_ns=float(np.median(dt[dt>0]))
    expected=int(np.ceil(2_000_000_000/median_delta_ns))
    min_count=(95*expected+99)//100
    window_ids = (timestamps - timestamps[0]) // 2_000_000_000
    window_features = []
    for wid in range(int(window_ids[-1])+1):
        lo = int(np.searchsorted(window_ids,wid,'left'))
        hi = int(np.searchsorted(window_ids,wid,'right'))
        if hi-lo < min_count: # Same 95% coverage convention as the inventory.
            continue
        g = gyro[lo:hi]
        wm = np.median(g,axis=0)
        window_features.append((float(np.percentile(np.linalg.norm(g-wm,axis=1),95)),
                                float(np.linalg.norm(wm-bias)),
                                float(np.linalg.norm(wm)),
                                float(np.var(accel[lo:hi]*9.80665,axis=0,ddof=1).sum())))
    features=np.asarray(window_features)
    sources=[{'name':p.name,'size':p.stat().st_size,'mtime_ns':p.stat().st_mtime_ns,
              **({'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} if p.name=='metadata.yaml' else {})}
             for p in sorted(bag.iterdir()) if p.is_file()]
    result = {'path':str(bag),'sources':sources,'topic':'/livox/imu','type':connections[0].msgtype,
              'frame_ids':sorted(frame_ids),'count':n,'first_header_ns':int(timestamps[0]),'last_header_ns':int(timestamps[-1]),
              'duration_s':duration_s,'storage_start_ns':int(storage[0]),'storage_end_ns':int(storage[-1]),
              'header_delta':describe(dt / 1e9),'nonmonotonic':int(np.count_nonzero(dt <= 0)),
              'header_storage_offset_s':describe((storage-timestamps)/1e9),
              'gyro_axes':{xyz:describe(gyro[:,j]) for j,xyz in enumerate('xyz')},
              'accel_axes':{xyz:describe(accel[:,j]) for j,xyz in enumerate('xyz')},
              'bias_median_xyz':bias.tolist(), 'raw_norm':describe(raw_norm),
              'residual_norm':describe(residual_norm),'blocks_1h':blocks,
              'windows_2s':{'count':len(features),'expected_samples':expected,'minimum_samples':min_count,
                            'centered_residual_p95':describe(features[:,0]),
                            'bias_distance_from_bag_median':describe(features[:,1]),
                            'window_bias_norm':describe(features[:,2]),
                            'accel_variance_sum_si':describe(features[:,3])}}
    out.write_text(json.dumps(result,indent=2,sort_keys=True))
    print(json.dumps({'path':str(bag),'n':n,'duration_s':duration_s,'raw_p95':result['raw_norm']['p95'],
                      'bias':result['bias_median_xyz'],'residual_p95':result['residual_norm']['p95'],
                      'blocks':len(blocks),'output':str(out)}))
