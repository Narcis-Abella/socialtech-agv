"""Exploratory stationary Mid-360 submap and prior-map preparation.

This method process has no reference input. It does not claim FAST-LIO2 or
motion compensation: its contract is a stationary S2 prototype only.
"""
import argparse
import json
import re
import struct
import time
from pathlib import Path

import numpy as np

POINT = np.dtype([('t', '<u4'), ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                  ('r', 'u1'), ('tag', 'u1'), ('line', 'u1'), ('pad', 'u1')])
PLY_TYPES = {'char': 'i1', 'uchar': 'u1', 'short': '<i2', 'ushort': '<u2',
             'int': '<i4', 'uint': '<u4', 'float': '<f4', 'double': '<f8'}


def quat_matrix(q):
    q = np.asarray(q, dtype=float)
    q /= np.linalg.norm(q)
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def transform(points, T):
    return np.asarray(points) @ T[:3, :3].T + T[:3, 3]


def voxel(points, size):
    if size <= 0:
        raise ValueError('voxel size must be positive')
    points = np.asarray(points, dtype=np.float64)
    if not len(points):
        return points.reshape(0, 3)
    _, inverse, counts = np.unique(np.floor(points / size).astype(np.int64),
                                   axis=0, return_inverse=True, return_counts=True)
    return np.column_stack([np.bincount(inverse, weights=points[:, j]) for j in range(3)]) / counts[:, None]


def scan_end(stamp, offsets):
    return stamp + int(np.max(offsets)) * 1e-9


def custom_points(raw):
    b = bytes(raw)
    if b[1] != 1:
        raise ValueError('big endian CDR unsupported')
    sec, nsec, nstr = struct.unpack_from('<iiI', b, 4)
    off = 16 + nstr
    off += (-(off - 4)) % 8
    off += 16
    (n,) = struct.unpack_from('<I', b, off)
    off += 4
    if n <= 0 or n > 1000000:
        raise ValueError('invalid CustomMsg point count')
    b += b'\0' * max(0, off + n * POINT.itemsize - len(b))
    pts = np.frombuffer(b, POINT, n, off)
    xyz = np.column_stack([pts[k] for k in 'xyz']).astype(np.float64)
    return sec + nsec * 1e-9, scan_end(sec + nsec * 1e-9, pts['t']), xyz


def read_ply(path):
    with open(path, 'rb') as f:
        header = b''
        while not header.endswith(b'end_header\n'):
            line = f.readline()
            if not line:
                raise ValueError('missing PLY header end')
            header += line
        lines = header.decode().splitlines()
        if 'format binary_little_endian 1.0' not in lines:
            raise ValueError('expected binary little endian PLY')
        count = int(next(l.split()[2] for l in lines if l.startswith('element vertex')))
        props = [(s[2], PLY_TYPES[s[1]]) for l in lines if (s := l.split()) and s[0] == 'property']
        data = np.frombuffer(f.read(count * np.dtype(props).itemsize), dtype=np.dtype(props), count=count)
    return np.column_stack([data[k] for k in 'xyz']).astype(np.float64)


def map_transform(path):
    lines = Path(path).read_text().splitlines()
    i = next(i for i, line in enumerate(lines) if 'T_map_world' in line)
    rows = []
    for line in lines[i+1:i+5]:
        numbers = re.findall(r'[-+]?(?:\d*\.\d+|\d+\.?\d*)(?:[eE][-+]?\d+)?', line)
        rows.append([float(x) for x in numbers])
    T = np.asarray(rows)
    if T.shape != (4, 4) or not np.allclose(T[3], [0, 0, 0, 1]) or not np.allclose(T[:3,:3].T @ T[:3,:3], np.eye(3), atol=1e-3):
        raise ValueError('invalid T_map_world')
    return T


def write_xyz(path, points):
    np.savetxt(path, points, fmt='%.6f')


def prepare(bag, world, yaml, start, duration, out, voxel_m=.20, max_range=30.):
    from rosbags.highlevel import AnyReader
    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    t0 = time.monotonic()
    scans, timestamps, ends, counts = [], [], [], []
    with AnyReader([Path(bag)]) as reader:
        conn = [c for c in reader.connections if c.topic == '/livox/lidar' and c.msgtype == 'livox_ros_driver2/msg/CustomMsg']
        if len(conn) != 1:
            raise ValueError('expected exactly one Livox CustomMsg connection')
        # Storage time is about 0.1 s later than the sensor stamp. Wide padding
        # only limits I/O; the selection below uses sensor header time.
        for _, _, raw in reader.messages(connections=conn, start=int((start-1)*1e9), stop=int((start+duration+2)*1e9)):
            stamp, end, xyz = custom_points(raw)
            if not start <= stamp < start + duration:
                continue
            ok = np.isfinite(xyz).all(1)
            rng = np.linalg.norm(xyz, axis=1)
            ok &= (rng >= .5) & (rng <= max_range)
            scans.append(xyz[ok])
            timestamps.append(stamp)
            ends.append(end)
            counts.append(int(ok.sum()))
    if len(scans) < 2 or any(b <= a for a, b in zip(timestamps, timestamps[1:])):
        raise ValueError('insufficient or non-monotonic scans')
    # The query is explicitly stationary; no source-frame rotation/translation
    # is inferred from reference or map. Points remain in the LiDAR frame.
    fit = np.concatenate([s for t, s in zip(timestamps, scans) if t < start + duration*2/3])
    check = np.concatenate([s for t, s in zip(timestamps, scans) if t >= start + duration*2/3])
    if not len(fit) or not len(check):
        raise ValueError('fit/check split empty')
    fit = voxel(fit, voxel_m)
    check = voxel(check, voxel_m)
    map_points = transform(read_ply(world), map_transform(yaml))
    # Prior-map envelope, independent of query pose. Preserve floor and ceiling.
    map_points = map_points[np.isfinite(map_points).all(1)]
    map_points = map_points[(map_points[:,2] >= -.5) & (map_points[:,2] <= 5.)]
    map_points = voxel(map_points, voxel_m)
    write_xyz(out/'fit.xyz', fit)
    write_xyz(out/'check.xyz', check)
    write_xyz(out/'map.xyz', map_points)
    manifest = {'bag': str(bag), 'world': str(world), 'map_yaml': str(yaml),
                'start_header_s': start, 'duration_s': duration, 'last_scan_end_s': ends[-1],
                'stamps_s': timestamps, 'scan_points_filtered': counts,
                'fit_points': len(fit), 'check_points': len(check), 'map_points': len(map_points),
                'voxel_m': voxel_m, 'max_range_m': max_range,
                'frame_fit': 'livox_lidar_stationary', 'frame_map': 'map',
                'elapsed_prepare_s': time.monotonic()-t0,
                'truth_used': False, 'frontend': 'stationary_raw_prototype_no_FAST_LIO2'}
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--bag', required=True)
    p.add_argument('--world', required=True)
    p.add_argument('--yaml', required=True)
    p.add_argument('--start', type=float, required=True, help='sensor header epoch seconds')
    p.add_argument('--duration', type=float, default=3.)
    p.add_argument('--out', required=True)
    p.add_argument('--voxel', type=float, default=.20)
    p.add_argument('--max-range', type=float, default=30.)
    a = p.parse_args()
    prepare(a.bag, a.world, a.yaml, a.start, a.duration, a.out, a.voxel, a.max_range)
