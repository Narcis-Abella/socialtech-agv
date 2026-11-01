"""Ghost removal: which points of a SLAM cloud the lidar beams go through. numpy; the beam counting is libsocialtech_mapping_beams.so (docker/ros_mapper, all cores).

A point is HIT in a scan when a beam (nearest return of a 1 degree range-image bin) ends within a margin (+ 1 % of the range) of it, PASSED when the beam ends farther away.
A voxel of 5 cm (the centroid of its points is tested) with at least MIN_OBS observations and hits / (hits + passed) < MIN_RATIO is a ghost: walls measure ~0.75, isolated specks ~0.01.
The counter is a function count(C, margin) -> (hit, passed) so the rule is testable without the engine; make_map builds it from count_beams and a BagScans."""
import ctypes
import os
import sys

import numpy as np

EXTR = (-0.011, -0.02329, 0.04412)      # m, lidar -> IMU translation (mid360.yaml)
RMAX = 15.0                             # m: beams farther than this from a point are not used
MIN_RANGE = 0.1                         # m, the nearest returns are not used (the Mid-360 blind zone; Voxel-SLAM blind: 0.1 as well)
EL_LO, EL_HI = -7.0, 52.0               # deg, the Mid-360's vertical field of view: the range image covers it in 1 deg bins
RATIO_VOX, RATIO_MARGIN, MIN_OBS, MIN_RATIO = 0.05, 0.03, 10, 0.10   # ghost voxel (m), HIT margin (m), observations needed, hit ratio under which a voxel is a ghost
MAX_GAP = 0.5                           # s, poses further apart than this are not interpolated
SCAN_MID = 0.05                         # s after the header stamp: the pose used for a 10 Hz Livox scan


def _library():
    return ctypes.CDLL(os.environ.get("BEAMS_LIB", "libsocialtech_mapping_beams.so"))


def engine_error():
    """None when the C++ library loads; else the message with the way out."""
    try:
        _library()
    except OSError as e:
        return f"the C++ beam engine is not available here ({e}). Run in the mapper image, or set BEAMS_LIB to the path of libsocialtech_mapping_beams.so."
    return None


class BagScans:
    """The scans of a bag as the C++ engine reads them itself: count_beams hands it the bag instead of decoded scans."""
    def __init__(self, bag, poses, topic):
        self.bag, self.poses, self.topic = str(bag), np.ascontiguousarray(poses[:, :8], float), topic


def count_beams(C, scans, margin):
    """(hit, passed) per candidate point C (n, 3, PLY frame): in how many scans of the bag a beam ended at the point (within margin + 1 % of its range) or went through it and ended
    farther. The C++ engine reads the bag itself (every scan of the topic that has a pose) and counts on all cores; scans: BagScans."""
    lib = _library()
    f, dp, ip = lib.smap_count_beams, ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int32)
    f.restype = ctypes.c_long
    f.argtypes = [ctypes.c_char_p, ctypes.c_char_p, dp, ctypes.c_long, dp, ctypes.c_long, dp, ip, ip, ctypes.c_char_p, ctypes.c_long]
    C = np.ascontiguousarray(C, float)
    params = np.array([*EXTR, margin, RMAX, MIN_RANGE, EL_LO, EL_HI, SCAN_MID, MAX_GAP], float)
    hit, passed, err = np.zeros(len(C), np.int32), np.zeros(len(C), np.int32), ctypes.create_string_buffer(512)
    n = f(scans.bag.encode(), scans.topic.encode(), scans.poses.ctypes.data_as(dp), len(scans.poses), C.ctypes.data_as(dp), len(C), params.ctypes.data_as(dp),
          hit.ctypes.data_as(ip), passed.ctypes.data_as(ip), err, len(err))
    if n < 0:
        raise RuntimeError(f"socialtech_mapping_beams: {err.value.decode()}")
    return hit, passed


def ghost_voxels(hit, passed):
    """bool per voxel from its beam counts: at least MIN_OBS observations and a hit ratio under MIN_RATIO."""
    nobs = hit + passed
    return (nobs >= MIN_OBS) & (hit / np.maximum(nobs, 1) < MIN_RATIO)


def ghost_points(P, Q, count, band=(-np.inf, np.inf)):
    """bool per cloud point: it lies in a ghost voxel. P: points in the PLY frame, Q: the same points levelled and aligned (z = height above the floor); only the points
    with band[0] < Q z <= band[1] are judged (the whole height by default); the others are never ghosts. count(C, margin) -> (hit, passed) counts the beams per candidate."""
    out = np.zeros(len(P), bool)
    sel = (Q[:, 2] > band[0]) & (Q[:, 2] <= band[1])
    if not sel.any():
        return out
    uk, inv = np.unique(np.floor(P[sel] / RATIO_VOX).astype(np.int64), axis=0, return_inverse=True)
    inv = inv.ravel()
    cand = np.column_stack([np.bincount(inv, P[sel][:, i]) for i in range(3)]) / np.bincount(inv)[:, None]   # centroid = where the surface is, not the voxel centre
    hit, passed = count(cand, RATIO_MARGIN)
    out[sel] = ghost_voxels(hit, passed)[inv]
    return out


def main(argv):
    """`--check-engine`: 0 when the C++ library loads, 1 with the way out on stderr. make_map.sh asks it before the long Voxel-SLAM stage."""
    if "--check-engine" in argv:
        err = engine_error()
        if err:
            print(f"ghosts: {err}", file=sys.stderr)
        return 1 if err else 0
    print("usage: ghosts.py --check-engine", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
