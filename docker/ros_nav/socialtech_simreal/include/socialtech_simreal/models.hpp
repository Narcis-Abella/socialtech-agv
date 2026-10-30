// Realism models of the Tracer simulation, without ROS. Numbers: notes/sim_realism/calibration.json of the SocialTech workspace (2026-10-30), measured on real bags:
//   BaseModel      command -> actual motion: linear dead time 60 ms + lag 100 ms, gain 1.015; angular dead time 120 ms + lag 50 ms, gain 0.97 moving; in place a deadband (static friction) of
//                  0.03-0.18 rad/s that depends on the session (floor, casters): w_actual = sign(u) max(0, |u| - d) when |v cmd| < 0.03 m/s
//   WheelOdomModel what /odom reports: linear 0.965 x actual, angular 0.92-0.96 in place and 0.967-1.003 moving, quantised (0.003 m/s, 0.018 rad/s), 30 ms late
//   ScanModel      the /scan of pointcloud_to_laserscan on the Mid-360 cloud: gaussian core 10-25 mm by range, dropout 2-27 % by range, and surface jumps on a persistent subset of the beams
//   ImuModel       ICM-40609-D from a 12 h Allan variance: white noise by axis and a Gauss-Markov bias
#pragma once
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <random>
#include <utility>
#include <vector>

namespace simreal
{

struct Rng
{
  explicit Rng(int seed = -1)
  : g(seed >= 0 ? static_cast<unsigned>(seed) : std::random_device{}()) {}
  double normal() {return n(g);}
  double uniform() {return u(g);}
  double uniform(double a, double b) {return a + (b - a) * u(g);}
  std::mt19937 g;
  std::normal_distribution<double> n{0.0, 1.0};
  std::uniform_real_distribution<double> u{0.0, 1.0};
};

// y(x) piecewise linear, clamped beyond the ends
struct Table
{
  std::vector<double> x, y;
  double at(double v) const
  {
    if (v <= x.front()) {return y.front();}
    if (v >= x.back()) {return y.back();}
    size_t i = 1;
    while (x[i] < v) {++i;}
    const double a = (v - x[i - 1]) / (x[i] - x[i - 1]);
    return y[i - 1] + a * (y[i] - y[i - 1]);
  }
};

class Lowpass
{
public:
  double step(double in, double tau, double dt)
  {
    if (!init_) {y_ = in; init_ = true;}
    y_ += (tau <= dt * 0.5 ? 1.0 : dt / (tau + dt)) * (in - y_);
    return y_;
  }
  double value() const {return y_;}

private:
  double y_ = 0;
  bool init_ = false;
};

// pure dead time in whole samples
class DeadTime
{
public:
  explicit DeadTime(int n = 0) {reset(n);}
  void reset(int n) {q_.assign(static_cast<size_t>(std::max(n, 0)), 0.0);}
  double step(double in)
  {
    if (q_.empty()) {return in;}
    q_.push_back(in);
    const double out = q_.front();
    q_.pop_front();
    return out;
  }

private:
  std::deque<double> q_;
};

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
struct BaseParams
{
  double dt = 0.01;
  double td_v = 0.06, tau_v = 0.10, gain_v = 1.015;
  double td_w = 0.12, tau_w = 0.05, gain_w = 0.97;
  double deadband = 0.10;                    // rad/s, in place only; draw it with BaseParams::draw_deadband
  double inplace_v = 0.03, ramp_v = 0.03;    // |v cmd| below inplace_v: full deadband; above inplace_v + ramp_v: none
  double kick = 0.0, kick_tau = 1.0;         // yaw-rate disturbance after an in-place turn (rad/s, opposite to it); WEAK evidence (7 sequences): off by default
  double kick_min_turn_s = 0.5;

  void draw_deadband(Rng & r, double lo = 0.03, double hi = 0.18) {deadband = r.uniform(lo, hi);}
};

class BaseModel
{
public:
  explicit BaseModel(const BaseParams & p)
  : p_(p), dv_(static_cast<int>(std::lround(p.td_v / p.dt))), dw_(static_cast<int>(std::lround(p.td_w / p.dt))), dvw_(static_cast<int>(std::lround(p.td_w / p.dt))) {}

  // one step of dt: commanded (v, w) -> the velocities the base really performs
  std::pair<double, double> step(double v_cmd, double w_cmd)
  {
    const double v_d = dv_.step(v_cmd), w_d = dw_.step(w_cmd), v_for_w = dvw_.step(v_cmd);   // the angular channel sees the linear command with its own delay
    const double v_out = p_.gain_v * lv_.step(v_d, p_.tau_v, p_.dt);
    const double f = std::clamp(1.0 - (std::fabs(v_for_w) - p_.inplace_v) / p_.ramp_v, 0.0, 1.0);   // 1 in place, 0 when moving
    const double d = p_.deadband * f;
    const double u = std::copysign(std::max(0.0, std::fabs(w_d) - d), w_d);
    double w_out = p_.gain_w * lw_.step(u, p_.tau_w, p_.dt);
    // caster kick: remember the in-place turn, and once the base moves, add a decaying yaw rate opposite to it
    const bool in_place = std::fabs(v_for_w) < p_.inplace_v && std::fabs(w_d) > 0.2;
    if (in_place) {turn_t_ += p_.dt; last_sign_ = std::copysign(1.0, w_d);}
    else if (turn_t_ > 0.0 && std::fabs(v_for_w) > 0.05) {
      if (turn_t_ >= p_.kick_min_turn_s) {kick_amp_ = -last_sign_ * p_.kick; kick_t_ = 0.0;}
      turn_t_ = 0.0;
    }
    if (kick_amp_ != 0.0 && std::fabs(v_for_w) > 0.05) {
      w_out += kick_amp_ * std::exp(-kick_t_ / p_.kick_tau);
      kick_t_ += p_.dt;
    }
    return {v_out, w_out};
  }

private:
  BaseParams p_;
  DeadTime dv_, dw_, dvw_;
  Lowpass lv_, lw_;
  double turn_t_ = 0, last_sign_ = 1, kick_amp_ = 0, kick_t_ = 0;
};

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
struct WheelParams
{
  double dt = 0.02;                      // input rate (Gazebo odometry at 50 Hz)
  double k_v = 0.965;                    // reported / actual, linear
  double k_w_moving = 0.985;             // reported / actual, angular, moving; draw_scales picks it per run
  double k_w_inplace = 0.94;             // reported / actual, angular, |v| < 0.03
  double q_v = 0.003, q_w = 0.018;       // quantisation steps
  double n_v = 0.0002, n_w = 0.001;      // white noise at rest
  double delay = 0.03;                   // reported later than the actual motion

  void draw_scales(Rng & r)
  {
    k_v = r.uniform(0.960, 0.970); k_w_moving = r.uniform(0.967, 1.003); k_w_inplace = r.uniform(0.92, 0.96);
  }
};

class WheelOdomModel
{
public:
  explicit WheelOdomModel(const WheelParams & p, int seed = -1)
  : p_(p), rng_(seed), dv_(static_cast<int>(std::lround(p.delay / p.dt))), dw_(static_cast<int>(std::lround(p.delay / p.dt))) {}

  std::pair<double, double> step(double v_true, double w_true)
  {
    const double in_place = std::fabs(v_true) < 0.03 ? 1.0 : 0.0;
    const double kw = in_place * p_.k_w_inplace + (1.0 - in_place) * p_.k_w_moving;
    double v = dv_.step(p_.k_v * v_true + p_.n_v * rng_.normal()), w = dw_.step(kw * w_true + p_.n_w * rng_.normal());
    v = std::round(v / p_.q_v) * p_.q_v;
    w = std::round(w / p_.q_w) * p_.q_w;
    return {v, w};
  }

private:
  WheelParams p_;
  Rng rng_;
  DeadTime dv_, dw_;
};

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
struct ScanParams
{
  // range bins of the real stationary stretches: [0.5,1) [1,2) [2,3) [3,5) [5,8) [8,12) [12,20); tables at the geometric middle of each
  Table core_sigma{{0.75, 1.5, 2.5, 4.0, 6.5, 10.0, 16.0}, {0.0163, 0.0134, 0.0099, 0.0156, 0.0159, 0.0248, 0.0201}};
  Table dropout{{0.75, 1.5, 2.5, 4.0, 6.5, 10.0, 16.0}, {0.038, 0.020, 0.021, 0.059, 0.085, 0.151, 0.275}};
  Table outlier_rate{{0.75, 1.5, 2.5, 4.0, 6.5, 10.0, 16.0}, {0.160, 0.209, 0.214, 0.219, 0.269, 0.253, 0.351}};   // mean over beams of the share of frames
  Table outlier_median{{0.75, 1.5, 2.5, 4.0, 6.5, 10.0, 16.0}, {0.282, 0.493, 0.947, 2.101, 1.837, 0.733, 1.560}};   // m
  Table outlier_p90{{0.75, 1.5, 2.5, 4.0, 6.5, 10.0, 16.0}, {1.251, 1.579, 2.099, 3.382, 4.748, 5.902, 9.819}};      // m
  Table nearer{{0.75, 1.5, 2.5, 4.0, 6.5, 10.0, 16.0}, {0.29, 0.47, 0.61, 0.70, 0.66, 0.42, 0.73}};                  // share of outliers closer than the median
  double rough_q = 0.7;          // frame probability of an outlier on a rough beam; the share of rough beams is outlier_rate / rough_q
  double outlier_scale = 1.0;    // 0 = no surface jumps (gaussian core and dropout only)
  double noise_scale = 1.0;
};

class ScanModel
{
public:
  explicit ScanModel(const ScanParams & p, int seed = -1)
  : p_(p), rng_(seed) {}

  void apply(std::vector<float> & ranges, double range_min, double range_max)
  {
    if (cls_.size() != ranges.size()) {cls_.assign(ranges.size(), -1);}
    for (size_t i = 0; i < ranges.size(); ++i) {
      const double r = ranges[i];
      if (!std::isfinite(r) || r <= 0.0) {continue;}
      if (rng_.uniform() < p_.dropout.at(r)) {ranges[i] = std::numeric_limits<float>::infinity(); continue;}
      if (cls_[i] < 0) {cls_[i] = rng_.uniform() < std::min(1.0, p_.outlier_rate.at(r) / p_.rough_q) ? 1 : 0;}   // persistent roughness of this beam
      double out = r + p_.noise_scale * p_.core_sigma.at(r) * rng_.normal();
      if (p_.outlier_scale > 0.0 && cls_[i] == 1 && rng_.uniform() < p_.rough_q * p_.outlier_scale) {
        const double med = p_.outlier_median.at(r), p90 = std::max(p_.outlier_p90.at(r), med * 1.01);
        const double mag = std::exp(std::log(med) + (std::log(p90) - std::log(med)) / 1.2816 * rng_.normal());
        out = rng_.uniform() < p_.nearer.at(r) ? r - mag : r + mag;
      }
      if (out < range_min) {out = range_min;}
      ranges[i] = out > range_max ? std::numeric_limits<float>::infinity() : static_cast<float>(out);
    }
  }

private:
  ScanParams p_;
  Rng rng_;
  std::vector<signed char> cls_;
};

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
struct ImuParams
{
  double fs = 200.0;
  double gyro_density[3] = {8.3e-5, 6.25e-5, 1.07e-4};        // rad/s/sqrt(Hz)
  double gyro_bias[3] = {3.3e-5, 2.3e-5, 3.0e-5};             // rad/s, bias instability
  double gyro_tau[3] = {29.0, 116.0, 97.0};                   // s
  double acc_density[3] = {8.7e-4, 4.9e-4, 4.1e-4};           // m/s2/sqrt(Hz), middle of the fit range
  double acc_bias[3] = {1.3e-4, 1.3e-4, 2.9e-4};              // m/s2
  double acc_tau[3] = {232.0, 138.0, 29.0};                   // s
};

class ImuModel
{
public:
  explicit ImuModel(const ImuParams & p, int seed = -1)
  : p_(p), rng_(seed) {}

  // dt: sample period of the input; adds noise in place
  void step(double gyro[3], double acc[3], double dt)
  {
    for (int i = 0; i < 3; ++i) {
      gb_[i] = gm(gb_[i], p_.gyro_bias[i], p_.gyro_tau[i], dt);
      ab_[i] = gm(ab_[i], p_.acc_bias[i], p_.acc_tau[i], dt);
      gyro[i] += gb_[i] + p_.gyro_density[i] / std::sqrt(dt) * rng_.normal();   // a sample of a white process of density N and sample period dt has std N / sqrt(dt)
      acc[i] += ab_[i] + p_.acc_density[i] / std::sqrt(dt) * rng_.normal();
    }
  }

private:
  double gm(double b, double sigma, double tau, double dt)
  {
    const double a = std::exp(-dt / tau);
    return b * a + sigma * std::sqrt(1.0 - a * a) * rng_.normal();
  }
  ImuParams p_;
  Rng rng_;
  double gb_[3] = {0, 0, 0}, ab_[3] = {0, 0, 0};
};

}  // namespace simreal
