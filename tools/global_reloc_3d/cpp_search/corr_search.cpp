// C++ port of the hot part of correlative_search.py: MapField, correlate, yaw_peaks and the yaw loop (spread order, skip bar). Same operations, same names.
// Python keeps the final selection and the report (search()). C API for ctypes; every call is independent (no globals), so two searches can run at once on one field.
// Build: build.sh. FFT: pocketfft (header-only, the library scipy.fft is made of).
#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numeric>
#include <vector>

#include "pocketfft_hdronly.h"

namespace {
using cf = std::complex<float>;
using pocketfft::shape_t;
using pocketfft::stride_t;
using i64 = int64_t;
constexpr double INF_D2 = 1e12;     // squared distance of "no feature in this line yet": above any real one, and the integers below stay exact in a double

stride_t contiguous(const shape_t & s, size_t el)
{
  stride_t st(s.size());
  size_t a = el;
  for (int i = static_cast<int>(s.size()) - 1; i >= 0; --i) {st[i] = static_cast<ptrdiff_t>(a); a *= s[i];}
  return st;
}

// Felzenszwalb & Huttenlocher lower envelope of parabolas: d[q] = min_p f[p] + (q-p)^2. Integers only, so exact.
void dt1d(const double * f, int n, double * d, int * v, double * z)
{
  int k = 0;
  v[0] = 0; z[0] = -1e300; z[1] = 1e300;
  for (int q = 1; q < n; ++q) {
    double s;
    for (;;) {
      const int p = v[k];
      s = ((f[q] + static_cast<double>(q) * q) - (f[p] + static_cast<double>(p) * p)) / (2.0 * q - 2.0 * p);
      if (s <= z[k]) {--k;} else {break;}
    }
    ++k; v[k] = q; z[k] = s; z[k + 1] = 1e300;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) {++k;}
    d[q] = static_cast<double>(q - v[k]) * (q - v[k]) + f[v[k]];
  }
}

// squared Euclidean distance (cells) to the nearest occupied cell, in place on g[x][y][z]; floats are exact for the values that matter (< 2^24)
void edt_squared(std::vector<float> & g, const i64 sh[3])
{
  const i64 nmax = std::max({sh[0], sh[1], sh[2]});
  std::vector<double> f(nmax), d(nmax), z(nmax + 1);
  std::vector<int> v(nmax);
  const i64 stride[3] = {sh[1] * sh[2], sh[2], 1};
  for (int ax = 2; ax >= 0; --ax) {
    const int a1 = (ax + 1) % 3, a2 = (ax + 2) % 3;
    for (i64 i = 0; i < sh[a1]; ++i) {
      for (i64 j = 0; j < sh[a2]; ++j) {
        float * base = g.data() + i * stride[a1] + j * stride[a2];
        for (i64 q = 0; q < sh[ax]; ++q) {f[q] = base[q * stride[ax]];}
        dt1d(f.data(), static_cast<int>(sh[ax]), d.data(), v.data(), z.data());
        for (i64 q = 0; q < sh[ax]; ++q) {base[q * stride[ax]] = static_cast<float>(d[q]);}
      }
    }
  }
}

struct Field {
  double res = 0, origin[3] = {0, 0, 0};
  i64 shape[3] = {0, 0, 0}, vlo[3] = {0, 0, 0}, vhi[3] = {0, 0, 0};
  size_t nx = 0, ny = 0, nyh = 0;           // padded FFT grid; nyh = ny/2 + 1
  i64 z_first = 0, nlayers = 0;
  size_t workers = 1;
  std::vector<cf> layers;                  // [layer][nx][nyh]
};

struct Workspace {
  std::vector<float> S, c, vol;
  std::vector<cf> spec_rows, spectrum, acc;
};

i64 floor_mod(i64 a, i64 m) {const i64 r = a % m; return r < 0 ? r + m : r;}

double yaw_distance_deg(double a, double b)
{
  const double d = (a - b) * 180.0 / M_PI;
  double m = std::fmod(d + 180.0, 360.0);
  if (m < 0) {m += 360.0;}                  // Python's % has the sign of the divisor
  return std::fabs(m - 180.0);
}

// 0 ok, 2 the source reaches outside the field in z. vol[(x*ny_w + y)*nz_w + z] (the valid window, C order of the (x, y, z) array), ncells = distinct voxels
int correlate(const Field & F, Workspace & W, const double * src, i64 m, double yaw, i64 win[3], i64 * ncells)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  std::vector<i64> cx(m), cy(m), cz(m);
  for (i64 i = 0; i < m; ++i) {
    const double x = src[3 * i] * c + src[3 * i + 1] * (-s) + src[3 * i + 2] * 0.0;
    const double y = src[3 * i] * s + src[3 * i + 1] * c + src[3 * i + 2] * 0.0;
    const double zz = src[3 * i] * 0.0 + src[3 * i + 1] * 0.0 + src[3 * i + 2] * 1.0;
    cx[i] = static_cast<i64>(std::floor(x / F.res)); cy[i] = static_cast<i64>(std::floor(y / F.res)); cz[i] = static_cast<i64>(std::floor(zz / F.res));
  }
  const i64 bx = *std::min_element(cx.begin(), cx.end()), by = *std::min_element(cy.begin(), cy.end()), bz = *std::min_element(cz.begin(), cz.end());
  const i64 sy = *std::max_element(cy.begin(), cy.end()) - by + 1, sz = *std::max_element(cz.begin(), cz.end()) - bz + 1;
  std::vector<i64> key(m);
  for (i64 i = 0; i < m; ++i) {key[i] = ((cx[i] - bx) * sy + (cy[i] - by)) * sz + (cz[i] - bz);}
  std::sort(key.begin(), key.end());
  key.erase(std::unique(key.begin(), key.end()), key.end());
  const i64 n = static_cast<i64>(key.size());
  *ncells = n;
  std::vector<i64> ux(n), uy(n), uz(n);
  for (i64 i = 0; i < n; ++i) {
    ux[i] = key[i] / (sy * sz) + bx; uy[i] = (key[i] % (sy * sz)) / sz + by; uz[i] = key[i] % sz + bz;
  }
  const i64 k0 = *std::min_element(uz.begin(), uz.end()), k1 = *std::max_element(uz.begin(), uz.end());
  const i64 nk = k1 - k0 + 1, nz0 = F.vhi[2] - F.vlo[2] + 1;
  if (F.vlo[2] + k0 < F.z_first || F.vhi[2] + k1 >= F.z_first + F.nlayers) {return 2;}
  const i64 nx = static_cast<i64>(F.nx), ny = static_cast<i64>(F.ny), nyh = static_cast<i64>(F.nyh);
  // the occupied rows only are transformed along y
  std::vector<i64> rowof(n), rows;
  for (i64 i = 0; i < n; ++i) {rowof[i] = floor_mod(ux[i], nx);}
  rows = rowof;
  std::sort(rows.begin(), rows.end());
  rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
  const i64 nr = static_cast<i64>(rows.size());
  std::vector<i64> ri(n);
  for (i64 i = 0; i < n; ++i) {ri[i] = std::lower_bound(rows.begin(), rows.end(), rowof[i]) - rows.begin();}
  W.S.assign(static_cast<size_t>(nk * nr * ny), 0.f);
  for (i64 i = 0; i < n; ++i) {W.S[static_cast<size_t>(((uz[i] - k0) * nr + ri[i]) * ny + floor_mod(uy[i], ny))] = 1.f;}
  W.spec_rows.resize(static_cast<size_t>(nk * nr * nyh));
  pocketfft::r2c(shape_t{(size_t)nk, (size_t)nr, (size_t)ny}, contiguous({(size_t)nk, (size_t)nr, (size_t)ny}, sizeof(float)),
                 contiguous({(size_t)nk, (size_t)nr, (size_t)nyh}, sizeof(cf)), 2, true, W.S.data(), W.spec_rows.data(), 1.f, F.workers);
  W.spectrum.assign(static_cast<size_t>(nk * nx * nyh), cf(0.f, 0.f));
  for (i64 k = 0; k < nk; ++k) {
    for (i64 r = 0; r < nr; ++r) {
      std::memcpy(&W.spectrum[static_cast<size_t>((k * nx + rows[r]) * nyh)], &W.spec_rows[static_cast<size_t>((k * nr + r) * nyh)], sizeof(cf) * nyh);
    }
  }
  const shape_t sshape{(size_t)nk, (size_t)nx, (size_t)nyh};
  pocketfft::c2c(sshape, contiguous(sshape, sizeof(cf)), contiguous(sshape, sizeof(cf)), shape_t{1}, true, W.spectrum.data(), W.spectrum.data(), 1.f, F.workers);
  for (auto & e : W.spectrum) {e = std::conj(e);}
  // sum over the source's layers, in the order of the Python loop
  const size_t plane = static_cast<size_t>(nx * nyh);
  W.acc.resize(static_cast<size_t>(nz0) * plane);
  // blocks of the spectrum plane: the layers and the source spectrum of one block stay in cache for every z (the whole layer stack, ~11 MB on a real map, was read once per z)
  constexpr size_t BLOCK = 384;
  for (size_t j0 = 0; j0 < plane; j0 += BLOCK) {
    const size_t len = std::min(BLOCK, plane - j0);
    for (i64 i = 0; i < nz0; ++i) {
      const i64 z = F.vlo[2] + i;
      float * __restrict acc = reinterpret_cast<float *>(&W.acc[static_cast<size_t>(i) * plane + j0]);
      for (i64 k = 0; k < nk; ++k) {
        const float * __restrict L = reinterpret_cast<const float *>(&F.layers[static_cast<size_t>(z + k0 - F.z_first + k) * plane + j0]);
        const float * __restrict Sp = reinterpret_cast<const float *>(&W.spectrum[static_cast<size_t>(k) * plane + j0]);
        if (k == 0) {
          for (size_t j = 0; j < len; ++j) {
            const float a = L[2 * j], b = L[2 * j + 1], cc = Sp[2 * j], d = Sp[2 * j + 1];
            acc[2 * j] = a * cc - b * d; acc[2 * j + 1] = a * d + b * cc;
          }
        } else {
          for (size_t j = 0; j < len; ++j) {
            const float a = L[2 * j], b = L[2 * j + 1], cc = Sp[2 * j], d = Sp[2 * j + 1];
            acc[2 * j] += a * cc - b * d; acc[2 * j + 1] += a * d + b * cc;
          }
        }
      }
    }
  }
  W.c.resize(static_cast<size_t>(nz0 * nx * ny));
  const shape_t oshape{(size_t)nz0, (size_t)nx, (size_t)ny};
  pocketfft::c2r(oshape, contiguous({(size_t)nz0, (size_t)nx, (size_t)nyh}, sizeof(cf)), contiguous(oshape, sizeof(float)), shape_t{1, 2}, false,
                 W.acc.data(), W.c.data(), 1.f / static_cast<float>(nx * ny), F.workers);
  win[0] = F.vhi[0] - F.vlo[0] + 1; win[1] = F.vhi[1] - F.vlo[1] + 1; win[2] = nz0;
  W.vol.resize(static_cast<size_t>(win[0] * win[1] * win[2]));
  const float fn = static_cast<float>(n);
  for (i64 z = 0; z < nz0; ++z) {
    for (i64 x = 0; x < win[0]; ++x) {
      const float * row = &W.c[static_cast<size_t>((z * nx + x + F.vlo[0]) * ny + F.vlo[1])];
      for (i64 y = 0; y < win[1]; ++y) {W.vol[static_cast<size_t>((x * win[1] + y) * nz0 + z)] = row[y] / fn;}
    }
  }
  return 0;
}

struct Peak {double score, yaw, xyz[3]; i64 yaw_index, rank, voxels;};

void yaw_peaks(const Field & F, const std::vector<float> & flat, const i64 win[3], int per_yaw, double nms_xy, std::vector<std::array<double, 4>> & out)
{
  const size_t N = flat.size(), k = std::min<size_t>(static_cast<size_t>(per_yaw) * 20, N);
  std::vector<uint32_t> idx(N);
  std::iota(idx.begin(), idx.end(), 0u);
  auto better = [&](uint32_t a, uint32_t b) {return flat[a] > flat[b] || (flat[a] == flat[b] && a < b);};
  std::nth_element(idx.begin(), idx.begin() + (k - 1), idx.end(), better);
  std::sort(idx.begin(), idx.begin() + k, better);
  std::vector<std::array<double, 3>> xyz(k);
  for (size_t j = 0; j < k; ++j) {
    const i64 i = idx[j], z = i % win[2], y = (i / win[2]) % win[1], x = i / (win[2] * win[1]);
    xyz[j] = {F.origin[0] + (x + F.vlo[0]) * F.res, F.origin[1] + (y + F.vlo[1]) * F.res, F.origin[2] + (z + F.vlo[2]) * F.res};
  }
  std::vector<size_t> chosen;
  for (size_t j = 0; j < k; ++j) {
    bool ok = true;
    for (size_t c : chosen) {
      const double dx = xyz[c][0] - xyz[j][0], dy = xyz[c][1] - xyz[j][1];
      if (!(std::sqrt(dx * dx + dy * dy) > nms_xy)) {ok = false; break;}
    }
    if (ok) {
      chosen.push_back(j);
      out.push_back({xyz[j][0], xyz[j][1], xyz[j][2], static_cast<double>(flat[idx[j]])});
      if (static_cast<int>(chosen.size()) == per_yaw) {break;}
    }
  }
}

double top_of_separated(std::vector<Peak> peaks, double nms_xy, double nms_yaw_deg, int top)
{
  const double slack = 1e-6;
  std::stable_sort(peaks.begin(), peaks.end(), [](const Peak & a, const Peak & b) {return a.score > b.score;});
  std::vector<const Peak *> chosen;
  for (const Peak & p : peaks) {
    bool far = true;
    for (const Peak * q : chosen) {
      if (!(std::hypot(p.xyz[0] - q->xyz[0], p.xyz[1] - q->xyz[1]) > 2 * nms_xy + slack || yaw_distance_deg(p.yaw, q->yaw) > 2 * nms_yaw_deg + slack)) {far = false; break;}
    }
    if (far) {
      chosen.push_back(&p);
      if (static_cast<int>(chosen.size()) == top) {return p.score;}
    }
  }
  return -std::numeric_limits<double>::infinity();
}
}  // namespace

extern "C" {

// info (out, 11): shape[3], valid_lo[3], valid_hi[3], z_first, layers.  Returns null on error: info[0] = 1 z_band outside the map box.
void * cs_field_new(const double * pts, i64 n, double res, double sigma, double reach_xy, double reach_z, int workers,
                    const double * zband, const i64 * zcells, i64 * info, double * origin_out)
{
  auto * F = new Field();
  F->res = res; F->workers = static_cast<size_t>(workers);
  double lo[3] = {1e300, 1e300, 1e300}, hi[3] = {-1e300, -1e300, -1e300};
  for (i64 i = 0; i < n; ++i) {for (int a = 0; a < 3; ++a) {lo[a] = std::min(lo[a], pts[3 * i + a]); hi[a] = std::max(hi[a], pts[3 * i + a]);}}
  const double pad[3] = {reach_xy, reach_xy, reach_z};
  for (int a = 0; a < 3; ++a) {
    F->origin[a] = lo[a] - pad[a];
    F->shape[a] = static_cast<i64>(std::ceil((hi[a] + pad[a] - F->origin[a]) / res)) + 1;
    F->vlo[a] = static_cast<i64>(std::floor((lo[a] - F->origin[a]) / res));
    F->vhi[a] = static_cast<i64>(std::ceil((hi[a] - F->origin[a]) / res));
    origin_out[a] = F->origin[a];
  }
  if (zband) {
    F->vlo[2] = std::max(F->vlo[2], static_cast<i64>(std::floor((zband[0] - F->origin[2]) / res)));
    F->vhi[2] = std::min(F->vhi[2], static_cast<i64>(std::ceil((zband[1] - F->origin[2]) / res)));
    if (F->vhi[2] < F->vlo[2]) {info[0] = 1; delete F; return nullptr;}
  }
  const size_t cells = static_cast<size_t>(F->shape[0] * F->shape[1] * F->shape[2]);
  std::vector<float> g(cells, static_cast<float>(INF_D2));
  for (i64 i = 0; i < n; ++i) {
    const i64 x = static_cast<i64>(std::floor((pts[3 * i] - F->origin[0]) / res)), y = static_cast<i64>(std::floor((pts[3 * i + 1] - F->origin[1]) / res)),
      z = static_cast<i64>(std::floor((pts[3 * i + 2] - F->origin[2]) / res));
    g[static_cast<size_t>((x * F->shape[1] + y) * F->shape[2] + z)] = 0.f;
  }
  edt_squared(g, F->shape);
  F->nx = pocketfft::detail::util::good_size_real(static_cast<size_t>(F->shape[0]));
  F->ny = pocketfft::detail::util::good_size_real(static_cast<size_t>(F->shape[1]));
  F->nyh = F->ny / 2 + 1;
  F->z_first = 0;
  i64 z_end = F->shape[2];
  if (zcells) {
    F->z_first = std::max<i64>(0, F->vlo[2] + zcells[0]);
    z_end = std::min(F->shape[2], F->vhi[2] + zcells[1] + 1);
  }
  F->nlayers = z_end - F->z_first;
  const size_t plane = F->nx * F->nyh;
  F->layers.resize(static_cast<size_t>(F->nlayers) * plane);
  std::vector<float> real(F->nx * F->ny);
  for (i64 l = 0; l < F->nlayers; ++l) {
    std::fill(real.begin(), real.end(), 0.f);
    const i64 z = F->z_first + l;
    for (i64 x = 0; x < F->shape[0]; ++x) {
      for (i64 y = 0; y < F->shape[1]; ++y) {
        const double d = std::sqrt(static_cast<double>(g[static_cast<size_t>((x * F->shape[1] + y) * F->shape[2] + z)])) * res;
        const double t = d / sigma;
        real[static_cast<size_t>(x) * F->ny + y] = static_cast<float>(std::exp(-.5 * (t * t)));
      }
    }
    pocketfft::r2c(shape_t{F->nx, F->ny}, contiguous({F->nx, F->ny}, sizeof(float)), contiguous({F->nx, F->nyh}, sizeof(cf)), shape_t{0, 1}, true,
                   real.data(), &F->layers[static_cast<size_t>(l) * plane], 1.f, F->workers);
  }
  for (int a = 0; a < 3; ++a) {info[a] = F->shape[a]; info[3 + a] = F->vlo[a]; info[6 + a] = F->vhi[a];}
  info[9] = F->z_first; info[10] = F->nlayers;
  return F;
}

void cs_field_free(void * f) {delete static_cast<Field *>(f);}

// one yaw's score volume (valid window, (x, y, z) C order) into out (capacity cap floats); win[3] its shape, ncells the source voxels. Returns 0, 2 (z), 3 (cap too small)
int cs_correlate(void * f, const double * src, i64 m, double yaw, float * out, i64 cap, i64 * win, i64 * ncells)
{
  Workspace W;
  const Field & F = *static_cast<Field *>(f);
  const int rc = correlate(F, W, src, m, yaw, win, ncells);
  if (rc) {return rc;}
  if (static_cast<i64>(W.vol.size()) > cap) {return 3;}
  std::memcpy(out, W.vol.data(), W.vol.size() * sizeof(float));
  return 0;
}

// the yaw loop of search(): order = the yaw indexes (spread_order) of yaws[]; out: rows of 8 doubles (score, yaw, x, y, z, yaw index, rank, voxels), cap rows.
// info: peaks written, yaws skipped. Returns 0, 2 (z), 3 (cap too small)
int cs_search(void * f, const double * src, i64 m, const double * yaws, const i64 * order, i64 n_order, int per_yaw, double nms_xy, double nms_yaw_deg, int top,
              double * out, i64 cap, i64 * info)
{
  const Field & F = *static_cast<Field *>(f);
  Workspace W;
  std::vector<Peak> peaks;
  double bar = -std::numeric_limits<double>::infinity();
  i64 skipped = 0;
  for (i64 o = 0; o < n_order; ++o) {
    const i64 i = order[o];
    i64 win[3], n;
    const int rc = correlate(F, W, src, m, yaws[i], win, &n);
    if (rc) {return rc;}
    const double vmax = *std::max_element(W.vol.begin(), W.vol.end());
    if (vmax < bar) {++skipped; continue;}
    std::vector<std::array<double, 4>> got;
    yaw_peaks(F, W.vol, win, per_yaw, nms_xy, got);
    for (size_t j = 0; j < got.size(); ++j) {peaks.push_back({got[j][3], yaws[i], {got[j][0], got[j][1], got[j][2]}, i, static_cast<i64>(j), n});}
    bar = top_of_separated(peaks, nms_xy, nms_yaw_deg, top);
  }
  if (static_cast<i64>(peaks.size()) > cap) {return 3;}
  for (size_t p = 0; p < peaks.size(); ++p) {
    const Peak & k = peaks[p];
    const double row[8] = {k.score, k.yaw, k.xyz[0], k.xyz[1], k.xyz[2], static_cast<double>(k.yaw_index), static_cast<double>(k.rank), static_cast<double>(k.voxels)};
    std::memcpy(out + 8 * p, row, sizeof(row));
  }
  info[0] = static_cast<i64>(peaks.size()); info[1] = skipped;
  return 0;
}

}  // extern "C"
