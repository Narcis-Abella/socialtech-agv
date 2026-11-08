#include <gtest/gtest.h>

#include <cmath>
#include <numeric>

#include "socialtech_simreal/models.hpp"

using namespace simreal;

namespace
{
double mean(const std::vector<double> & v) {return std::accumulate(v.begin(), v.end(), 0.0) / double(v.size());}
double stdev(const std::vector<double> & v)
{
  const double m = mean(v); double s = 0;
  for (double x : v) {s += (x - m) * (x - m);}
  return std::sqrt(s / double(v.size()));
}
}  // namespace

TEST(Base, LinearStepHasDeadTimeLagAndGain)
{
  BaseParams p; BaseModel m(p); std::vector<double> y;
  for (int i = 0; i < 300; ++i) {y.push_back(m.step(0.6, 0.0).first);}
  const int nd = int(std::lround(p.td_v / p.dt));
  for (int i = 0; i < nd; ++i) {EXPECT_NEAR(y[i], 0.0, 1e-12) << i;}               // dead time: nothing moves yet
  EXPECT_NEAR(y[nd + 10], p.gain_v * 0.6 * (1 - std::pow(1 - p.dt / (p.tau_v + p.dt), 11)), 0.02);   // the lag
  EXPECT_NEAR(y.back(), 1.015 * 0.6, 0.002);                                         // steady state: measured gain
}

TEST(Base, InPlaceDeadbandAndNoDeadbandWhileMoving)
{
  BaseParams p; p.deadband = 0.18;
  { BaseModel m(p); double w = 0; for (int i = 0; i < 300; ++i) {w = m.step(0.0, 0.10).second;} EXPECT_NEAR(w, 0.0, 1e-9); }            // below the deadband: no rotation in place
  { BaseModel m(p); double w = 0; for (int i = 0; i < 300; ++i) {w = m.step(0.0, 0.50).second;} EXPECT_NEAR(w, 0.97 * (0.50 - 0.18), 1e-3); }   // above: w - d
  { BaseModel m(p); double w = 0; for (int i = 0; i < 300; ++i) {w = m.step(0.0, -0.50).second;} EXPECT_NEAR(w, -0.97 * (0.50 - 0.18), 1e-3); } // symmetric
  { BaseModel m(p); double w = 0; for (int i = 0; i < 300; ++i) {w = m.step(0.30, 0.10).second;} EXPECT_NEAR(w, 0.97 * 0.10, 1e-3); }       // moving: the same command goes through (gain 0.97)
}

TEST(Base, CasterKickOpposesTheLastInPlaceTurnAndIsOffByDefault)
{
  auto run = [](double kick) {
      BaseParams p; p.deadband = 0.0; p.kick = kick; BaseModel m(p); std::vector<double> after;
      for (int i = 0; i < 150; ++i) {m.step(0.0, 0.6);}                       // 1.5 s turning left in place
      // 0.5-1.0 s after the command: the kick (tau 1 s) is still about -0.03 rad/s
      for (int i = 0; i < 400; ++i) {const double w = m.step(0.3, 0.0).second; if (i >= 50 && i < 100) {after.push_back(w);}}
      return mean(after);
    };
  EXPECT_NEAR(run(0.0), 0.0, 1e-3);
  EXPECT_LT(run(0.05), -0.02);
}

TEST(Wheel, ScaleQuantisationAndInPlaceRatio)
{
  WheelParams p; p.n_v = 0; p.n_w = 0; WheelOdomModel m(p, 1);
  double v = 0, w = 0; for (int i = 0; i < 20; ++i) {std::tie(v, w) = m.step(0.30, 0.0);}
  EXPECT_NEAR(v, 0.965 * 0.30, 0.0015 + 1e-9);                                    // within half a quantisation step
  EXPECT_NEAR(std::fmod(std::fabs(v) / 0.003 + 1e-9, 1.0), 0.0, 1e-6);            // multiple of 0.003 m/s
  WheelOdomModel m2(p, 1);
  for (int i = 0; i < 20; ++i) {std::tie(v, w) = m2.step(0.0, 0.40);}
  EXPECT_NEAR(w, 0.94 * 0.40, 0.009 + 1e-9);                                      // in place: 0.94, quantised at 0.018 rad/s
  EXPECT_NEAR(std::fmod(std::fabs(w) / 0.018 + 1e-9, 1.0), 0.0, 1e-6);
}

TEST(Scan, GaussianCoreAndDropoutMatchTheRealBins)
{
  for (double r : {2.5, 16.0}) {
    ScanParams p; p.outlier_scale = 0.0; ScanModel m(p, 7); std::vector<double> dev; long n = 0, drop = 0;
    for (int f = 0; f < 400; ++f) {
      std::vector<float> s(720, float(r)); m.apply(s, 0.1, 60.0);
      for (float x : s) {++n; if (std::isinf(x)) {++drop;} else {dev.push_back(x - r);}}
    }
    EXPECT_NEAR(double(drop) / double(n), p.dropout.at(r), 0.01) << r;
    EXPECT_NEAR(stdev(dev), p.core_sigma.at(r), 0.0015) << r;
  }
}

TEST(Scan, SurfaceJumpsHaveTheMeasuredRateAndMagnitude)
{
  ScanParams p; ScanModel m(p, 11); const double r = 2.5; long n = 0, out = 0; std::vector<double> mag; long nearer = 0;
  for (int f = 0; f < 600; ++f) {
    std::vector<float> s(720, float(r)); m.apply(s, 0.1, 60.0);
    for (float x : s) {
      if (std::isinf(x)) {continue;}
      ++n;
      if (std::fabs(x - r) >= 0.10) {++out; mag.push_back(std::fabs(x - r)); if (x < r) {++nearer;}}
    }
  }
  EXPECT_NEAR(double(out) / double(n), p.outlier_rate.at(r), 0.03);               // measured: 21 % of the frames of a beam at 2-3 m
  std::sort(mag.begin(), mag.end());
  EXPECT_NEAR(mag[mag.size() / 2], p.outlier_median.at(r), 0.20 * p.outlier_median.at(r) + 0.1);   // median of the jump (0.95 m measured)
  EXPECT_NEAR(double(nearer) / double(out), p.nearer.at(r), 0.06);
}

TEST(Scan, ABeamPushedBelowRangeMinReturnsNothingInsteadOfAFakeObstacleAtRangeMin)
{
  // a 'nearer' jump is sampled with a magnitude that does not depend on the range: at 0.6 m it can be several metres and r - mag is negative
  ScanParams p; p.noise_scale = 200.0; ScanModel m(p, 5); const double rmin = 0.5; long at_rmin = 0, none = 0;
  for (int f = 0; f < 100; ++f) {
    std::vector<float> s(720, 0.6f); m.apply(s, rmin, 60.0);
    for (float x : s) {if (x == float(rmin)) {++at_rmin;} if (std::isinf(x)) {++none;}}
  }
  EXPECT_EQ(at_rmin, 0);                                            // real chain: a return closer than range_min does not exist
  EXPECT_GT(none, 0);
}

TEST(Scan, NoOutliersWhenSwitchedOffAndInvalidBeamsUntouched)
{
  ScanParams p; p.outlier_scale = 0.0; ScanModel m(p, 3); std::vector<float> s(720, 3.0f); s[5] = std::numeric_limits<float>::infinity(); s[6] = 0.0f;
  for (int f = 0; f < 50; ++f) {auto t = s; m.apply(t, 0.1, 60.0); EXPECT_TRUE(std::isinf(t[5])); EXPECT_EQ(t[6], 0.0f); for (float x : t) {if (std::isfinite(x) && x > 0) {EXPECT_LT(std::fabs(x - 3.0f), 0.3f);}}}
}

TEST(Imu, WhiteNoisePerSampleIsDensityOverSqrtDt)
{
  ImuParams p; ImuModel m(p, 5); const double dt = 0.005; std::vector<double> gx, az;
  for (int i = 0; i < 200000; ++i) {double g[3] = {0, 0, 0}, a[3] = {0, 0, 0}; m.step(g, a, dt); gx.push_back(g[0]); az.push_back(a[2]);}
  EXPECT_NEAR(stdev(gx), p.gyro_density[0] / std::sqrt(dt), 0.05 * p.gyro_density[0] / std::sqrt(dt));
  EXPECT_NEAR(stdev(az), p.acc_density[2] / std::sqrt(dt), 0.15 * p.acc_density[2] / std::sqrt(dt));   // the Gauss-Markov bias (2.9e-4) adds to the white part (5.8e-3)
}
