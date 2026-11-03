# cpp_search

C++ port of the hot part of `correlative_search.py` (MapField, correlate, `yaw_peaks`, the yaw loop with the skip bar). Same operations; `search()` keeps the final selection and the report. `search_v2.py --corr-backend cpp` uses it; the default stays `python`.

| file | what |
|---|---|
| `corr_search.cpp` | the library, C API for ctypes (`corr_cpp.py`) |
| `pocketfft_hdronly.h` | pocketfft (BSD-3, the FFT scipy.fft is built on), commit in `POCKETFFT_COMMIT`, licence in `POCKETFFT_LICENSE.md` |
| `build.sh` | `libcorr_search.so` (g++ -O3, no `-march`: one binary for every Orin; ~14 s) |

Differences from the numpy path: exact EDT by lower envelope (same distances as scipy's feature transform), per-yaw ties among equal scores broken by cell index, `rot_z` product without BLAS. Measured on the AGX, 17 real inputs x 2: decisions 34/34 equal, primary poses identical, scores within 2e-7 (`test_correlative_search.CppBackendTest`, skipped when the library is not built).
