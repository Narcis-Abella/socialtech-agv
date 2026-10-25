"""Recorded frames of the global costmap (OccupancyGrid data, int8 -1..100): zlib + base64 so a trial JSON stays small (a mostly free 520 x 280 frame is ~1 kB)."""
import base64
import zlib

import numpy as np


def encode(data):
    return base64.b64encode(zlib.compress(np.asarray(data, np.int8).tobytes(), 6)).decode("ascii")


def decode(s):
    return np.frombuffer(zlib.decompress(base64.b64decode(s)), np.int8)


def frame_at(frames, t):
    """The payload of the last [t_i, payload] with t_i <= t, or None before the first frame."""
    last = None
    for ti, payload in frames:
        if ti > t:
            break
        last = payload
    return last
