"""ctypes front of cpp_search/libcorr_search.so: CppMapField, a drop-in for correlative_search.MapField whose search loop runs in C++ (search() picks it up through `native`).
The library is looked for in $CORR_SEARCH_LIB, then next to cpp_search/corr_search.cpp (cpp_search/build.sh makes it), then by name in $LD_LIBRARY_PATH (a staged board: GRELOC/build, the lib_dir of register_v2). Calls release the GIL: two searches run at once on one field."""
import ctypes as C
import os
from pathlib import Path

import numpy as np

_I64P, _DP, _FP = C.POINTER(C.c_int64), C.POINTER(C.c_double), C.POINTER(C.c_float)
_lib = None


def load(path=None):
    global _lib
    if _lib is None:
        local = Path(__file__).parent / 'cpp_search' / 'libcorr_search.so'
        path = path or os.environ.get('CORR_SEARCH_LIB') or (local if local.exists() else 'libcorr_search.so')
        lib = C.CDLL(str(path))
        lib.cs_field_new.restype = C.c_void_p
        lib.cs_field_new.argtypes = [_DP, C.c_int64, C.c_double, C.c_double, C.c_double, C.c_double, C.c_int, _DP, _I64P, _I64P, _DP]
        lib.cs_field_free.argtypes = [C.c_void_p]
        lib.cs_correlate.argtypes = [C.c_void_p, _DP, C.c_int64, C.c_double, _FP, C.c_int64, _I64P, _I64P]
        lib.cs_search.argtypes = [C.c_void_p, _DP, C.c_int64, _DP, _I64P, C.c_int64, C.c_int, C.c_double, C.c_double, C.c_int, _DP, C.c_int64, _I64P]
        _lib = lib
    return _lib


def _check(rc):
    if rc == 2:
        raise ValueError('source reaches outside the field in z (or the layers this field was built for)')
    if rc:
        raise RuntimeError(f'libcorr_search error {rc}')


def _ptr(a, t):
    return a.ctypes.data_as(C.POINTER(t))


class CppMapField:
    native = True

    def __init__(self, points, res, sigma, reach_xy, reach_z, workers=1, z_band=None, z_cells=None):
        lib = load()
        pts = np.ascontiguousarray(points, np.float64)
        zb = None if z_band is None else np.asarray(z_band, np.float64)
        zc = None if z_cells is None else np.asarray(z_cells, np.int64)
        info, origin = np.zeros(11, np.int64), np.zeros(3)
        self._lib = lib
        self._h = lib.cs_field_new(_ptr(pts, C.c_double), len(pts), res, sigma, reach_xy, reach_z, workers,
                                   None if zb is None else _ptr(zb, C.c_double), None if zc is None else _ptr(zc, C.c_int64), _ptr(info, C.c_int64), _ptr(origin, C.c_double))
        if not self._h:
            raise ValueError('z_band outside the map box')
        self.res, self.workers, self.origin = res, workers, origin
        self.shape, self.valid_lo, self.valid_hi = tuple(int(n) for n in info[0:3]), info[3:6].copy(), info[6:9].copy()
        self.z_first, self.n_layers = int(info[9]), int(info[10])

    def __del__(self):
        if getattr(self, '_h', None):
            self._lib.cs_field_free(self._h)

    def correlate(self, source_points, yaw):
        """(score volume of the valid window, number of source voxels), as MapField.correlate."""
        src = np.ascontiguousarray(source_points, np.float64)
        win, n = np.zeros(3, np.int64), np.zeros(1, np.int64)
        cap = int(np.prod(self.valid_hi - self.valid_lo + 1))
        out = np.empty(cap, np.float32)
        _check(self._lib.cs_correlate(self._h, _ptr(src, C.c_double), len(src), yaw, _ptr(out, C.c_float), cap, _ptr(win, C.c_int64), _ptr(n, C.c_int64)))
        return out[:int(np.prod(win))].reshape(tuple(win)), int(n[0])

    def loop(self, source, yaws, order, per_yaw, nms_xy, nms_yaw_deg, top):
        """The yaw loop of search(): (rows (score, yaw, x, y, z, yaw index, rank in the yaw, source voxels), yaws skipped)."""
        src = np.ascontiguousarray(source, np.float64)
        yaws = np.ascontiguousarray(yaws, np.float64)
        order = np.ascontiguousarray(order, np.int64)
        cap = len(yaws) * per_yaw
        out, info = np.empty((cap, 8)), np.zeros(2, np.int64)
        _check(self._lib.cs_search(self._h, _ptr(src, C.c_double), len(src), _ptr(yaws, C.c_double), _ptr(order, C.c_int64), len(order), per_yaw, nms_xy, nms_yaw_deg, top,
                                   _ptr(out, C.c_double), cap, _ptr(info, C.c_int64)))
        return out[:info[0]], int(info[1])
