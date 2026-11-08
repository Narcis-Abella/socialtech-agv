"""Point-cloud helpers of the relocalizer: rigid transform, voxel grid (the mean of each occupied voxel, in first-seen order), PLY read, XYZ write."""
import numpy as np

PLY_TYPES = {'char': 'i1', 'uchar': 'u1', 'short': '<i2', 'ushort': '<u2',
             'int': '<i4', 'uint': '<u4', 'float': '<f4', 'double': '<f8'}


def transform(points, T):
    return np.asarray(points) @ T[:3, :3].T + T[:3, 3]


def voxel(points, size):
    if size <= 0:
        raise ValueError('voxel size must be positive')
    points = np.asarray(points, dtype=np.float64)
    if not len(points):
        return points.reshape(0, 3)
    cells = np.floor(points / size).astype(np.int64)
    base = cells.min(axis=0)
    span = cells.max(axis=0) - base + 1
    # one integer key per voxel (x major, so the order is np.unique(axis=0)'s), several times faster than np.unique on rows
    key = ((cells[:, 0] - base[0]) * span[1] + (cells[:, 1] - base[1])) * span[2] + (cells[:, 2] - base[2])
    _, inverse, counts = np.unique(key, return_inverse=True, return_counts=True)
    inverse = inverse.ravel()
    return np.column_stack([np.bincount(inverse, weights=points[:, j]) for j in range(3)]) / counts[:, None]


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


def write_xyz(path, points):
    np.savetxt(path, points, fmt='%.6f')
