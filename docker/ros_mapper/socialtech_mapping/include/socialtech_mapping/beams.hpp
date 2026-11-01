// Beam tests (count_beams, range_image, pose_at, custom_points), ported line by line from the numpy reference tools/slam_bench/beams_oracle.py (test-only) and checked against it.
// Same float64 formulas and the same order of operations as the numpy code; build with -ffp-contract=off so no multiply-add is fused.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace socialtech_mapping {

constexpr double kDeg = 180.0 / M_PI;   // numpy's rad2deg: x * (180 / pi)

struct Params {
  std::array<double, 3> extr;   // lidar -> IMU translation, m
  double margin, rmax, min_range, el_lo, el_hi, scan_mid, max_gap;
  int nel() const { return static_cast<int>(el_hi - el_lo); }
};

using Mat3 = std::array<double, 9>;   // row-major
struct Pose { Mat3 R; std::array<double, 3> t; };

// scipy Rotation.from_quat(q).as_matrix(): normalise, then the usual formula. q = x y z w.
inline Mat3 quat_to_matrix(double x, double y, double z, double w) {
  const double n = std::sqrt(x * x + y * y + z * z + w * w);
  x /= n; y /= n; z /= n; w /= n;
  const double x2 = x * x, y2 = y * y, z2 = z * z, w2 = w * w;
  const double xy = x * y, zw = z * w, xz = x * z, yw = y * w, yz = y * z, xw = x * w;
  return {x2 - y2 - z2 + w2, 2 * (xy - zw), 2 * (xz + yw),
          2 * (xy + zw), -x2 + y2 - z2 + w2, 2 * (yz - xw),
          2 * (xz - yw), 2 * (yz + xw), -x2 - y2 + z2 + w2};
}

// pose_at: poses rows of t x y z qx qy qz qw (time-sorted); linear interpolation, none outside the file or across a gap > max_gap.
inline std::optional<Pose> pose_at(const double* poses, long n, double t, double max_gap) {
  long lo = 0, hi = n;   // np.searchsorted(pt, t) (side left): first index with pt[j] >= t
  while (lo < hi) {
    const long mid = (lo + hi) / 2;
    if (poses[mid * 8] < t) lo = mid + 1; else hi = mid;
  }
  const long j = lo;
  if (j == 0 || j == n || poses[j * 8] - poses[(j - 1) * 8] > max_gap) return std::nullopt;
  const double* a = poses + (j - 1) * 8;
  const double* b = poses + j * 8;
  const double f = (t - a[0]) / std::max(b[0] - a[0], 1e-9);
  double dot = a[4] * b[4] + a[5] * b[5] + a[6] * b[6] + a[7] * b[7];
  const double s = dot >= 0 ? 1.0 : -1.0;
  double q[4];
  for (int i = 0; i < 4; ++i) q[i] = (1 - f) * a[4 + i] + f * (s * b[4 + i]);
  Pose p{quat_to_matrix(q[0], q[1], q[2], q[3]), {}};
  for (int i = 0; i < 3; ++i) p.t[i] = (1 - f) * a[1 + i] + f * b[1 + i];
  return p;
}

// custom_points: header stamp (s) and xyz (float32 triplets) from the CDR bytes of a livox_ros_driver2/CustomMsg.
inline double decode_custom(const uint8_t* b, size_t len, std::vector<float>& xyz) {
  auto u32 = [&](size_t o) { if (o + 4 > len) throw std::runtime_error("CustomMsg truncated"); uint32_t v; std::memcpy(&v, b + o, 4); return v; };
  if (len < 16 || b[1] != 1) throw std::runtime_error("big-endian or short CDR is not supported");
  const int32_t sec = static_cast<int32_t>(u32(4));
  const uint32_t nsec = u32(8), nstr = u32(12);
  size_t off = 16 + nstr;
  off += (8 - (off - 4) % 8) % 8;   // timebase (uint64) aligned to 8 counting from after the 4-byte CDR header
  off += 16;                        // timebase, point_num, lidar_id, rsvd[3]
  const uint32_t n = u32(off);
  off += 4;
  xyz.clear();
  xyz.reserve(3 * n);
  for (uint32_t i = 0; i < n; ++i) {   // 20-byte elements (19 + 1 of padding); the last may come without its padding byte
    const size_t o = off + 20 * size_t(i);
    if (o + 16 > len) throw std::runtime_error("CustomMsg truncated");
    float p[3];
    std::memcpy(p, b + o + 4, 12);
    if (std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2])) xyz.insert(xyz.end(), p, p + 3);
  }
  return sec + nsec * 1e-9;
}

// range_image: nearest return per 1 deg azimuth/elevation bin (inf = nothing seen), flat [azimuth * nel + elevation]. Returns nearer than min_range are dropped.
inline void range_image(const std::vector<float>& xyz, const Params& P, std::vector<double>& img) {
  const int nel = P.nel();
  img.assign(360 * nel, std::numeric_limits<double>::infinity());
  for (size_t i = 0; i + 2 < xyz.size(); i += 3) {
    const double x = xyz[i], y = xyz[i + 1], z = xyz[i + 2];
    const double r = std::sqrt(x * x + y * y + z * z);
    if (!(r > P.min_range)) continue;
    const double az = std::fmod(std::atan2(y, x) * kDeg + 180.0, 360.0);
    const double el = std::asin(z / r) * kDeg;
    if (!(el >= P.el_lo && el < P.el_hi)) continue;
    double& c = img[std::min(static_cast<int>(az), 359) * nel + static_cast<int>(el - P.el_lo)];
    c = std::min(c, r);
  }
}

// One scan of count_beams for one candidate c (PLY frame): +1 hit when a beam ends within margin + 1 % of its range, +1 passed when it ends farther.
inline void test_candidate(const double* c, const Pose& p, const Params& P, const std::vector<double>& img, int32_t& hit, int32_t& passed) {
  const double d0 = c[0] - p.t[0], d1 = c[1] - p.t[1], d2 = c[2] - p.t[2];
  const Mat3& R = p.R;   // (C - t) @ R - extr
  const double x = d0 * R[0] + d1 * R[3] + d2 * R[6] - P.extr[0];
  const double y = d0 * R[1] + d1 * R[4] + d2 * R[7] - P.extr[1];
  const double z = d0 * R[2] + d1 * R[5] + d2 * R[8] - P.extr[2];
  const double rc = std::sqrt(x * x + y * y + z * z);
  if (!(rc > P.min_range)) return;
  const double ec = std::asin(std::clamp(z / std::max(rc, 1e-9), -1.0, 1.0)) * kDeg;
  if (!(ec > P.el_lo && ec < P.el_hi)) return;
  const double ac = std::fmod(std::atan2(y, x) * kDeg + 180.0, 360.0);
  const int nel = P.nel();
  const double rb = img[std::min(static_cast<int>(ac), 359) * nel + std::clamp(static_cast<int>(ec - P.el_lo), 0, nel - 1)];
  if (!std::isfinite(rb)) return;
  const double mg = P.margin + 0.01 * rc;
  if (std::abs(rc - rb) <= mg) ++hit;
  if (rc < rb - mg) ++passed;
}

// Candidates bucketed on a 2-D grid (PLY x, y) so a scan visits only those within rmax of the sensor (exact 3-D test, <= like cKDTree.query_ball_point).
// Stored in bucket order (slot k = candidate index(k)), so a scan reads the coordinates and its counters sequentially.
class CandidateGrid {
 public:
  CandidateGrid(const double* C, long n, double cell) : cell_(cell) {
    if (n == 0) return;
    double lo[2] = {C[0], C[1]}, hi[2] = {C[0], C[1]};
    for (long i = 0; i < n; ++i)
      for (int k = 0; k < 2; ++k) { lo[k] = std::min(lo[k], C[3 * i + k]); hi[k] = std::max(hi[k], C[3 * i + k]); }
    x0_ = lo[0]; y0_ = lo[1];
    nx_ = static_cast<long>((hi[0] - lo[0]) / cell) + 1;
    ny_ = static_cast<long>((hi[1] - lo[1]) / cell) + 1;
    start_.assign(nx_ * ny_ + 1, 0);
    std::vector<long> key(n);
    for (long i = 0; i < n; ++i) { key[i] = cellof(C[3 * i], C[3 * i + 1]); ++start_[key[i] + 1]; }
    for (long k = 0; k < nx_ * ny_; ++k) start_[k + 1] += start_[k];
    order_.resize(n);
    std::vector<long> fill(start_.begin(), start_.end() - 1);
    for (long i = 0; i < n; ++i) order_[fill[key[i]]++] = i;
    xyz_.resize(3 * n);
    for (long k = 0; k < n; ++k) std::copy(C + 3 * order_[k], C + 3 * order_[k] + 3, xyz_.begin() + 3 * k);
  }
  long size() const { return static_cast<long>(order_.size()); }
  long index(long k) const { return order_[k]; }
  // Calls f(k, xyz) for each slot k whose candidate is within r of q.
  template <class F> void within(const double* q, double r, F&& f) const {
    if (order_.empty()) return;
    const long ix0 = std::max(0L, static_cast<long>(std::floor((q[0] - r - x0_) / cell_)));
    const long ix1 = std::min(nx_ - 1, static_cast<long>(std::floor((q[0] + r - x0_) / cell_)));
    const long iy0 = std::max(0L, static_cast<long>(std::floor((q[1] - r - y0_) / cell_)));
    const long iy1 = std::min(ny_ - 1, static_cast<long>(std::floor((q[1] + r - y0_) / cell_)));
    const double r2 = r * r;
    for (long ix = ix0; ix <= ix1; ++ix)
      for (long iy = iy0; iy <= iy1; ++iy)
        for (long k = start_[ix * ny_ + iy]; k < start_[ix * ny_ + iy + 1]; ++k) {
          const double* c = xyz_.data() + 3 * k;
          const double a = c[0] - q[0], b = c[1] - q[1], d = c[2] - q[2];
          if (a * a + b * b + d * d <= r2) f(k, c);
        }
  }

 private:
  long cellof(double x, double y) const { return static_cast<long>((x - x0_) / cell_) * ny_ + static_cast<long>((y - y0_) / cell_); }
  double cell_, x0_ = 0, y0_ = 0;
  long nx_ = 0, ny_ = 0;
  std::vector<long> start_, order_;
  std::vector<double> xyz_;
};

// count_beams for one scan already in the sensor frame; hit / passed are indexed by grid slot.
inline void count_scan(const CandidateGrid& grid, const std::vector<float>& xyz, const Pose& p, const Params& P,
                       std::vector<double>& img, int32_t* hit, int32_t* passed) {
  range_image(xyz, P, img);
  double s[3];   // sensor position: R @ extr + t
  for (int i = 0; i < 3; ++i) s[i] = p.R[3 * i] * P.extr[0] + p.R[3 * i + 1] * P.extr[1] + p.R[3 * i + 2] * P.extr[2] + p.t[i];
  grid.within(s, P.rmax, [&](long k, const double* c) { test_candidate(c, p, P, img, hit[k], passed[k]); });
}

}  // namespace socialtech_mapping
