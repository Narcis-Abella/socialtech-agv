// The estimator without ROS: Gauss-Newton scan-to-map tracking with a Kalman prior. Port of notes/nav_real_scripts/amcl_replay/gn_localizer.py + gn_loc/gn_loc_node.py (GNCore);
// the Python files are the reference the port is checked against (same bag, same numbers).
#pragma once
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <tuple>
#include <vector>

namespace gn_loc
{

struct Pose
{
  double x = 0, y = 0, yaw = 0;
};

inline double wrap(double a) {return std::remainder(a, 2 * M_PI);}

inline Pose compose(const Pose & a, const Pose & b)
{
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, a.yaw + b.yaw};
}

inline Pose inverse(const Pose & a)
{
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {-(c * a.x + s * a.y), -(-s * a.x + c * a.y), -a.yaw};
}

struct Params
{
  double sigma = 0.10;        // m, per beam
  double dmax = 0.20;         // m, beams farther than this from any wall do not vote
  double robust_c = 0.08;     // m, final Welsch scale: a beam d from the wall votes with weight exp(-(d/c)^2) (0 = plain least squares). Beams 8-20 cm off a wall are furniture, people, map
                              // error: with full quadratic weight inside the 20 cm gate they, entering and leaving it, made the single-scan fit of a parked robot scatter 0.7 deg / 2.4 cm (lidar
                              // noise alone: 0.06 deg). 0.05 scatters least but 3 of 45 odometry-jump variants then recover late or never; 0.08 keeps 45/45 (see notes/L2b_2026-10-31.md)
  int n_beams = 360;          // beams used per scan
  double q_pos = 0.02;        // m / sqrt(s), random walk of map->odom
  double q_yaw = 0.7 * M_PI / 180.0;   // rad / sqrt(s)
  double min_inliers = 0.25;  // ratio below which the scan counts as bad
  double tight = 0.05;        // m, a beam this close to a wall agrees with the map (consistency)
  double min_tight = 0.50;    // consistency below which the scan counts as bad: the inlier ratio (dmax 20 cm) stays high on a false optimum or on a blocked sensor's ring
  double accept_tight = 0.60; // consistency every scan of a recovery candidate must reach (stricter than min_tight: a wrong candidate is the dangerous outcome)
  double min_range_cv = 0.05; // std/mean of the used ranges below which a scan has no pose information (a blocked sensor's ring); real scans: >= 0.157 over 19 988 scans
  double lost_s = 3.0;        // s of bad scans until LOST
  bool init_search = true;    // exhaustive search around /initialpose on the first scan
  double rmin = 0.5, rmax = 12.0;
  // LOST: every recover_period s, the coarse-to-fine search around the last good map->odom o odometry, sigma growing with the time in LOST; a candidate is accepted after recover_k
  // consecutive good scans. After reloc_after_s in LOST, Result::need_reloc asks for the global relocalizer (its /initialpose ends LOST like any initial pose).
  double recover_period = 1.0;
  double recover_sxy0 = 0.2, recover_syaw0 = 5.0 * M_PI / 180.0;
  double recover_grow_xy = 0.05, recover_grow_yaw = 1.5 * M_PI / 180.0;   // per s in LOST
  double recover_sxy_max = 1.5, recover_syaw_max = 30.0 * M_PI / 180.0;
  int recover_k = 10;
  double reloc_after_s = 15.0;
  // Parked averaging: the map->odom random walk (q_pos, q_yaw) only grows while the odometry moves (net change over the last still_window_s above still_xy or still_yaw).
  // Parked, every scan only adds information and the estimate averages the fits since the stop instead of following each. Real bags (144 still windows): with the walk
  // growing in time, parked map->base wandered 1.4 deg median vs FAST-LIO's 0.6. (Not applying parked fits at all, as AMCL does, froze the error of the stop instant:
  // up to 12 cm for whole parked periods, and it lost two recoveries in the L5 variants.) Thresholds from the same bags: FAST-LIO's net change over 1 s while parked is
  // p50/p95/p99 0.7/2.5/3.9 cm and 0.08/0.38/0.61 deg; at 5 cm / 1 deg 0.34 % of parked scans count as moving and 1.1 % of wheel-moving ones as still (the first second
  // after moving off). still_window_s 0 = never still (the old behaviour).
  double still_window_s = 1.0, still_xy = 0.05, still_yaw = 1.0 * M_PI / 180.0;
};

// Distance (m) to the nearest occupied cell (value 100), bilinear at cell centres, and the exact gradient of that interpolation (central differences of the cells gave
// 0 on a one-cell wall and ramped to 1 a cell away: beams that agree with the map carried little weight, see Field.GradientIsTheDerivativeOfTheInterpolatedDistance).
class Field
{
public:
  Field(const std::vector<int8_t> & data, int w, int h, double res, double ox, double oy)
  : w_(w), h_(h), res_(res), ox_(ox), oy_(oy)
  {
    const double inf = 1e20;
    std::vector<double> sq(static_cast<size_t>(w) * h);
    for (size_t i = 0; i < sq.size(); ++i) {sq[i] = data[i] == 100 ? 0.0 : inf;}
    std::vector<double> f(std::max(w, h)), d(std::max(w, h)), z(std::max(w, h) + 1);
    std::vector<int> v(std::max(w, h));
    auto edt1d = [&](int n) {   // Felzenszwalb-Huttenlocher: exact squared distance along one line, f -> d
        int k = 0; v[0] = 0; z[0] = -inf; z[1] = inf;
        for (int q = 1; q < n; ++q) {
          double s;
          while (true) {
            s = ((f[q] + double(q) * q) - (f[v[k]] + double(v[k]) * v[k])) / (2.0 * q - 2.0 * v[k]);
            if (s <= z[k] && k > 0) {--k;} else {break;}
          }
          if (s <= z[k]) {v[k] = q; z[k] = -inf; z[k + 1] = inf;} else {++k; v[k] = q; z[k] = s; z[k + 1] = inf;}
        }
        k = 0;
        for (int q = 0; q < n; ++q) {
          while (z[k + 1] < q) {++k;}
          d[q] = double(q - v[k]) * (q - v[k]) + f[v[k]];
        }
      };
    for (int x = 0; x < w; ++x) {
      for (int y = 0; y < h; ++y) {f[y] = sq[size_t(y) * w + x];}
      edt1d(h);
      for (int y = 0; y < h; ++y) {sq[size_t(y) * w + x] = d[y];}
    }
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {f[x] = sq[size_t(y) * w + x];}
      edt1d(w);
      for (int x = 0; x < w; ++x) {sq[size_t(y) * w + x] = d[x];}
    }
    D_.resize(sq.size());
    for (size_t i = 0; i < sq.size(); ++i) {D_[i] = std::sqrt(sq[i]) * res;}
  }

  struct Sample
  {
    double d, gx, gy;
    bool ok;
  };

  Sample eval(double px, double py) const
  {
    const double fx = (px - ox_) / res_ - 0.5, fy = (py - oy_) / res_ - 0.5;
    if (!(fx >= 0 && fy >= 0 && fx < w_ - 1 && fy < h_ - 1)) {return {9.0, 9.0, 9.0, false};}
    const int x0 = int(fx), y0 = int(fy);
    const double ax = fx - x0, ay = fy - y0;
    const size_t i = size_t(y0) * w_ + x0;
    const double a = D_[i], b = D_[i + 1], c = D_[i + w_], d = D_[i + w_ + 1];
    return {a * (1 - ax) * (1 - ay) + b * ax * (1 - ay) + c * (1 - ax) * ay + d * ax * ay,
      ((b - a) * (1 - ay) + (d - c) * ay) / res_, ((c - a) * (1 - ax) + (d - b) * ax) / res_, true};
  }

  double distanceAt(int x, int y) const {return D_[size_t(y) * w_ + x];}

private:
  int w_, h_;
  double res_, ox_, oy_;
  std::vector<double> D_;
};

struct Beams
{
  std::vector<double> x, y;
};

// valid ranges (finite, rmin < r < rmax), evenly thinned to n_beams, as points in the base frame
inline Beams beams(const std::vector<float> & ranges, double a0, double inc, const Params & p)
{
  std::vector<double> r, a;
  for (size_t i = 0; i < ranges.size(); ++i) {
    const double v = ranges[i];
    if (std::isfinite(v) && v > p.rmin && v < p.rmax) {r.push_back(v); a.push_back(a0 + inc * double(i));}
  }
  std::vector<double> rr, aa;
  if (int(r.size()) > p.n_beams) {
    const double step = double(r.size() - 1) / (p.n_beams - 1);
    for (int i = 0; i < p.n_beams; ++i) {const size_t k = size_t(i * step); rr.push_back(r[k]); aa.push_back(a[k]);}
  } else {rr = r; aa = a;}
  Beams b;
  for (size_t i = 0; i < rr.size(); ++i) {b.x.push_back(rr[i] * std::cos(aa[i])); b.y.push_back(rr[i] * std::sin(aa[i]));}
  return b;
}

struct Fit
{
  Pose pose;
  Eigen::Matrix3d P;
  int inliers;
};

// 3-5 Gauss-Newton steps on the distance field, (H + P^-1) delta = -(g + P^-1 (p - pose)); P is the prior covariance of `pose`
inline Fit gnUpdate(const Field & F, const Params & prm, const Beams & b, const Pose & pose, const Eigen::Matrix3d & P, int iters = 5)
{
  const Eigen::Matrix3d Pinv = P.inverse();
  Pose p = pose;
  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  int n = 0;
  for (int it = 0; it < iters; ++it) {
    const double c = std::cos(p.yaw), s = std::sin(p.yaw);
    // Welsch scale annealed from dmax (halving per iteration) to robust_c: wide first so an offset start is pulled in, narrow at the end so clutter does not move the result
    const double wc = prm.robust_c > 0 ? std::max(prm.robust_c, prm.dmax / double(1 << it)) : 0.0;
    Eigen::Matrix3d Hn = Eigen::Matrix3d::Zero();
    Eigen::Vector3d g = Eigen::Vector3d::Zero();
    n = 0;
    for (size_t i = 0; i < b.x.size(); ++i) {
      const double qx = p.x + c * b.x[i] - s * b.y[i], qy = p.y + s * b.x[i] + c * b.y[i];
      const auto m = F.eval(qx, qy);
      if (!m.ok || !(m.d < prm.dmax)) {continue;}
      const Eigen::Vector3d J(m.gx, m.gy, m.gx * (-(qy - p.y)) + m.gy * (qx - p.x));
      const double w = wc > 0 ? std::exp(-(m.d / wc) * (m.d / wc)) : 1.0;
      Hn += w * J * J.transpose(); g += w * J * m.d; ++n;
    }
    if (n < 20) {return {pose, P, n};}
    const double s2 = prm.sigma * prm.sigma;
    H = Hn / s2; g /= s2;
    const Eigen::Vector3d prior = Pinv * Eigen::Vector3d(p.x - pose.x, p.y - pose.y, wrap(p.yaw - pose.yaw));
    const Eigen::Vector3d step = (H + Pinv).ldlt().solve(-(g + prior));
    p.x += step[0]; p.y += step[1]; p.yaw += step[2];
    if (wc <= prm.robust_c && std::abs(step[0]) < 1e-4 && std::abs(step[1]) < 1e-4 && std::abs(step[2]) < 1e-4) {break;}   // converged at the final scale
  }
  return {p, (H + Pinv).inverse(), n};
}

// fraction of the beams whose end lies within `tight` of a wall at `p`
inline double consistency(const Field & F, const Beams & b, const Pose & p, double tight)
{
  const double c = std::cos(p.yaw), s = std::sin(p.yaw);
  size_t n = 0;
  for (size_t i = 0; i < b.x.size(); ++i) {
    const auto m = F.eval(p.x + c * b.x[i] - s * b.y[i], p.y + s * b.x[i] + c * b.y[i]);
    n += m.ok && m.d < tight;
  }
  return double(n) / std::max<size_t>(b.x.size(), 1);
}

// std / mean of the beam ranges
inline double rangeCv(const Beams & b)
{
  double s = 0, s2 = 0;
  for (size_t i = 0; i < b.x.size(); ++i) {const double r = std::hypot(b.x[i], b.y[i]); s += r; s2 += r * r;}
  const double n = double(std::max<size_t>(b.x.size(), 1)), m = s / n;
  return m > 0 ? std::sqrt(std::max(s2 / n - m * m, 0.0)) / m : 0.0;
}

// sum over every `stride`-th beam of exp(-d^2 / 2 sigma^2), d = distance of the beam end to the nearest wall at pose `cand`
inline double score(const Field & F, const Beams & b, const Pose & cand, double sigma, size_t stride)
{
  const double c = std::cos(cand.yaw), s = std::sin(cand.yaw);
  double sc = 0;
  for (size_t i = 0; i < b.x.size(); i += stride) {
    const auto m = F.eval(cand.x + c * b.x[i] - s * b.y[i], cand.y + s * b.x[i] + c * b.y[i]);
    sc += std::exp(-m.d * m.d / (2 * sigma * sigma));
  }
  return sc;
}

// best pose within +-3 sigma (at least 0.3 m / 8 deg, yaw at most +-180 deg) of `pose`; the caller refines it with gnUpdate. Coarse to fine: a 15 cm / 4.5 deg grid on
// every 4th beam with a kernel as wide as the grid, then the 5 cm / 1.5 deg grid on every beam around the 10 best distinct coarse cells. One 5 cm grid over the whole
// prior grows with its cube (1 m / 30 deg: 1.8 M poses, ~18 s on the AGX), which blocked the node long enough for the lifecycle manager to declare it dead.
inline Pose exhaustive(const Field & F, const Params & prm, const Beams & b, const Pose & pose, double sxy, double syaw)
{
  const double rx = std::max(3 * sxy, 0.3), ry = std::min(std::max(3 * syaw, 8.0 * M_PI / 180.0), M_PI);
  const double cstep = 0.15, ctstep = 4.5 * M_PI / 180.0, fstep = 0.05, ftstep = 1.5 * M_PI / 180.0;
  const int kTop = 10;
  std::vector<std::pair<double, Pose>> coarse;
  for (double dt = -ry; dt <= ry + 1e-9; dt += ctstep) {
    for (double dx = -rx; dx <= rx + 1e-9; dx += cstep) {
      for (double dy = -rx; dy <= rx + 1e-9; dy += cstep) {
        const Pose cand{pose.x + dx, pose.y + dy, pose.yaw + dt};
        coarse.emplace_back(score(F, b, cand, std::max(prm.sigma, 0.25), 4), cand);
      }
    }
  }
  std::sort(coarse.begin(), coarse.end(), [](const auto & a, const auto & c) {return a.first > c.first;});
  std::vector<Pose> seeds;   // non-maximum suppression: the neighbours of a peak score almost as high and would fill the list
  for (const auto & [sc, p] : coarse) {
    if (int(seeds.size()) == kTop) {break;}
    const bool near = std::any_of(seeds.begin(), seeds.end(), [&](const Pose & q) {
          return std::abs(p.x - q.x) <= cstep + 1e-9 && std::abs(p.y - q.y) <= cstep + 1e-9 && std::abs(wrap(p.yaw - q.yaw)) <= ctstep + 1e-9;});
    if (!near) {seeds.push_back(p);}
  }
  double best = -1; Pose out = pose;
  for (const Pose & q : seeds) {
    for (double dt = -ctstep; dt <= ctstep + 1e-9; dt += ftstep) {
      for (double dx = -cstep; dx <= cstep + 1e-9; dx += fstep) {
        for (double dy = -cstep; dy <= cstep + 1e-9; dy += fstep) {
          const Pose cand{q.x + dx, q.y + dy, q.yaw + dt};
          const double sc = score(F, b, cand, prm.sigma, 1);
          if (sc > best) {best = sc; out = cand;}
        }
      }
    }
  }
  return out;
}

enum class State { WaitingPose, Tracking, Lost };

inline const char * name(State s) {return s == State::WaitingPose ? "WAITING_POSE" : s == State::Tracking ? "TRACKING" : "LOST";}

struct Result
{
  std::optional<Pose> Tmo;   // map->odom, empty until the first scan after an initial pose
  Eigen::Matrix3d P = Eigen::Matrix3d::Identity();
  double ratio = 0, tight = 0;   // inlier ratio (dmax) and consistency (tight) of the scan's fit
  State state = State::WaitingPose;
  bool need_reloc = false;
};

class Core
{
public:
  Core(std::shared_ptr<const Field> field, const Params & p)
  : F_(std::move(field)), prm_(p) {}

  void setInitial(const Pose & pose, double sxy, double syaw)
  {
    pending_ = {pose, std::max(sxy, 0.02), std::max(syaw, 1.0 * M_PI / 180.0)};
    state_ = State::Tracking; Tmo_.reset(); anchor_.reset(); tBad_.reset(); cand_.reset();
  }

  Result update(double t, const Pose & o, const std::vector<float> & ranges, double a0, double inc)
  {
    if (state_ == State::WaitingPose) {return {std::nullopt, P_, 0.0, 0.0, state_, false};}
    const Beams b = beams(ranges, a0, inc, prm_);
    if (b.x.size() < 20) {return {Tmo_, P_, 0.0, 0.0, state_, needReloc(t)};}
    const double dt = tPrev_ ? std::max(t - *tPrev_, 0.0) : 0.0;
    tPrev_ = t;
    const double dtq = moving(t, o) ? dt : 0.0;   // time that grows the map->odom random walk
    const bool informative = rangeCv(b) >= prm_.min_range_cv;
    if (!pending_ && state_ == State::Lost) {
      if (!informative) {cand_.reset(); return {Tmo_, P_, 0.0, 0.0, state_, needReloc(t)};}
      return recover(t, dt, o, b);
    }
    if (!pending_ && !informative) {   // no pose information: dead reckoning, counts as a bad scan
      P_ += Q(dtq);
      if (!tBad_) {tBad_ = t;}
      if (t - *tBad_ >= prm_.lost_s) {enterLost(t);}
      return {Tmo_, P_, 0.0, 0.0, state_, needReloc(t)};
    }
    Pose pose;
    if (pending_) {
      const auto [p0, sxy, syaw] = *pending_;
      pending_.reset();
      P_ = Eigen::Vector3d(sxy * sxy, sxy * sxy, syaw * syaw).asDiagonal();
      pose = prm_.init_search ? exhaustive(*F_, prm_, b, p0, sxy, syaw) : p0;
    } else {
      P_ += Q(dtq);
      pose = compose(*Tmo_, o);
    }
    const Fit f = gnUpdate(*F_, prm_, b, pose, P_);
    const double ratio = double(f.inliers) / std::max<size_t>(b.x.size(), 1), tight = consistency(*F_, b, f.pose, prm_.tight);
    const bool first = !Tmo_;
    if (first || good(ratio, tight)) {   // a bad scan's fit is not applied (a blocked sensor's ring fitted to a nearby wall dragged the pose metres): dead reckoning, P keeps growing
      P_ = f.P; Tmo_ = compose(f.pose, inverse(o));
    }
    if (!anchor_) {anchor_ = Tmo_;}   // the initial pose is the operator's (or the relocalizer's): the fallback until a scan agrees with the map
    if (good(ratio, tight)) {
      tBad_.reset(); anchor_ = Tmo_;
    } else {
      if (!tBad_) {tBad_ = t;}
      if (t - *tBad_ >= prm_.lost_s) {enterLost(t);}
    }
    return {Tmo_, P_, ratio, tight, state_, needReloc(t)};
  }

  State state() const {return state_;}

private:
  std::shared_ptr<const Field> F_;
  Params prm_;
  State state_ = State::WaitingPose;
  std::optional<Pose> Tmo_;
  Eigen::Matrix3d P_ = Eigen::Matrix3d::Identity();
  std::optional<double> tPrev_, tBad_, tSearch_;
  std::optional<std::tuple<Pose, double, double>> pending_;
  std::optional<Pose> anchor_, cand_;   // map->odom at the last good scan; recovery candidate (map->odom) being verified
  Eigen::Matrix3d candP_ = Eigen::Matrix3d::Identity();
  double tLost_ = 0;
  int nGood_ = 0;
  std::deque<std::pair<double, Pose>> hist_;   // odometry over the last still_window_s

  // net odometry change between now and still_window_s ago above still_xy / still_yaw; true until the window is filled
  bool moving(double t, const Pose & o)
  {
    if (prm_.still_window_s <= 0) {return true;}
    hist_.emplace_back(t, o);
    while (hist_.size() > 1 && hist_[1].first <= t - prm_.still_window_s) {hist_.pop_front();}
    if (hist_.front().first > t - prm_.still_window_s) {return true;}
    const Pose & a = hist_.front().second;
    return std::hypot(o.x - a.x, o.y - a.y) > prm_.still_xy || std::abs(wrap(o.yaw - a.yaw)) > prm_.still_yaw;
  }

  bool good(double ratio, double tight) const {return ratio >= prm_.min_inliers && tight >= prm_.min_tight;}
  bool accept(double ratio, double tight) const {return ratio >= prm_.min_inliers && tight >= prm_.accept_tight;}
  // fall back to the last map->odom the map agreed with (dead reckoning on the odometry) and start the recovery searches
  void enterLost(double t) {state_ = State::Lost; tLost_ = t; Tmo_ = anchor_; tSearch_.reset(); cand_.reset();}
  bool needReloc(double t) const {return state_ == State::Lost && t - tLost_ >= prm_.reloc_after_s;}
  Eigen::Matrix3d Q(double dt) const
  {
    return Eigen::Vector3d(prm_.q_pos * prm_.q_pos * dt, prm_.q_pos * prm_.q_pos * dt, prm_.q_yaw * prm_.q_yaw * dt).asDiagonal().toDenseMatrix();
  }

  // LOST: Tmo_ stays the anchor (published as is) until a candidate passes recover_k consecutive good scans
  Result recover(double t, double dt, const Pose & o, const Beams & b)
  {
    double ratio = 0, tight = 0;
    if (cand_) {
      const Fit f = gnUpdate(*F_, prm_, b, compose(*cand_, o), candP_ + Q(dt));
      ratio = double(f.inliers) / std::max<size_t>(b.x.size(), 1); tight = consistency(*F_, b, f.pose, prm_.tight);
      if (!accept(ratio, tight)) {
        cand_.reset();
      } else {
        cand_ = compose(f.pose, inverse(o)); candP_ = f.P;
        if (++nGood_ >= prm_.recover_k) {
          state_ = State::Tracking; Tmo_ = anchor_ = cand_; P_ = candP_; tBad_.reset(); cand_.reset();
        }
      }
    } else if (!tSearch_ || t - *tSearch_ >= prm_.recover_period) {
      tSearch_ = t;
      const double tl = t - tLost_;
      const double sxy = std::min(prm_.recover_sxy0 + prm_.recover_grow_xy * tl, prm_.recover_sxy_max);
      const double syaw = std::min(prm_.recover_syaw0 + prm_.recover_grow_yaw * tl, prm_.recover_syaw_max);
      const Eigen::Matrix3d P0 = Eigen::Vector3d(sxy * sxy, sxy * sxy, syaw * syaw).asDiagonal();
      const Fit f = gnUpdate(*F_, prm_, b, exhaustive(*F_, prm_, b, compose(*Tmo_, o), sxy, syaw), P0);
      ratio = double(f.inliers) / std::max<size_t>(b.x.size(), 1); tight = consistency(*F_, b, f.pose, prm_.tight);
      if (accept(ratio, tight)) {cand_ = compose(f.pose, inverse(o)); candP_ = f.P; nGood_ = 1;}
    }
    return {Tmo_, P_, ratio, tight, state_, needReloc(t)};
  }
};

}  // namespace gn_loc
