// MapRegistrar on a synthetic room (floor, four walls, two boxes): a scan taken with the body tilted by a known roll/pitch must give that tilt back, a scan of a
// different room must fail the gates.
#include <gtest/gtest.h>

#include <random>

#include "level_corrector/registration.hpp"

using namespace level_corrector;

static std::vector<Eigen::Vector3f> room(unsigned seed, double dens, bool boxes = true) {
  std::mt19937 g(seed); std::uniform_real_distribution<double> u(0, 1);
  std::vector<Eigen::Vector3f> p;
  auto plane = [&](Vec3 o, Vec3 a, Vec3 b) {   // o + s a + t b, s,t in [0,1]
    const int n = int(dens * a.norm() * b.norm());
    for (int i = 0; i < n; i++) { const double s = u(g), t = u(g); p.push_back((o + s * a + t * b).cast<float>()); }
  };
  plane({-5, -4, 0}, {10, 0, 0}, {0, 8, 0});                 // floor
  plane({-5, -4, 0}, {10, 0, 0}, {0, 0, 2.5}); plane({-5, 4, 0}, {10, 0, 0}, {0, 0, 2.5});   // long walls
  plane({-5, -4, 0}, {0, 8, 0}, {0, 0, 2.5}); plane({5, -4, 0}, {0, 8, 0}, {0, 0, 2.5});     // short walls
  if (boxes) {
    plane({1, 1, 0}, {1, 0, 0}, {0, 0, 1.2}); plane({1, 1, 0}, {0, 1, 0}, {0, 0, 1.2}); plane({1, 1, 1.2}, {1, 1, 0}, {0, 0, 0});
    plane({-3, -2, 0}, {0.8, 0, 0}, {0, 0, 0.9}); plane({-3, -2, 0}, {0, 0.8, 0}, {0, 0, 0.9});
  }
  return p;
}
// what a sensor at pose T (map <- body) sees of the map points within 8 m, in the body frame
static std::vector<Eigen::Vector3f> view(const std::vector<Eigen::Vector3f>& m, const Eigen::Isometry3d& T) {
  std::vector<Eigen::Vector3f> s;
  for (const auto& q : m) { const Vec3 b = T.inverse() * q.cast<double>(); if (b.norm() > 0.8 && b.norm() < 8) s.push_back(b.cast<float>()); }
  return s;
}
static Eigen::Isometry3d pose(double x, double y, double z, double yaw_deg, double roll_deg, double pitch_deg) {
  const double d = M_PI / 180;
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = (Eigen::AngleAxisd(yaw_deg * d, Vec3::UnitZ()) * Eigen::AngleAxisd(pitch_deg * d, Vec3::UnitY()) * Eigen::AngleAxisd(roll_deg * d, Vec3::UnitX())).toRotationMatrix();
  T.translation() = Vec3(x, y, z);
  return T;
}
static double deg_between(const Mat3& a, const Mat3& b) { return Eigen::AngleAxisd(a * b.transpose()).angle() * 180 / M_PI; }

class Reg : public ::testing::Test {
 protected:
  MapRegistrar reg{room(1, 400)};   // built once per test: tiny map
};

TEST_F(Reg, RecoversAKnownTiltFromAnUntiltedSeed) {
  const auto truth = pose(-1.0, 0.5, 0.5, 20, 1.0, -0.7);
  const auto seed = pose(-1.02, 0.47, 0.5, 20.5, 0, 0);   // planar pose slightly off, attitude level: what the chain + gn would give
  const auto r = reg.align(view(room(2, 250), truth), seed);
  EXPECT_EQ(gate_reason(r.fit, Gates()), nullptr) << "inliers " << r.fit.inliers << " rms " << r.fit.rms;
  EXPECT_LT(deg_between(r.T.linear().transpose(), truth.linear().transpose()), 0.15);   // the map's up in the body
  EXPECT_LT((r.T.translation() - truth.translation()).norm(), 0.05);
}
TEST_F(Reg, ReportsTheMotionAgainstTheSeed) {
  const auto truth = pose(0.0, 0.0, 0.5, 10, 0.5, 0.5);
  const auto seed = pose(0.1, -0.1, 0.5, 12, 0.5, 0.5);
  const auto r = reg.align(view(room(3, 250), truth), seed);
  EXPECT_NEAR(r.fit.dxy, std::hypot(0.1, 0.1), 0.05);
  EXPECT_NEAR(r.fit.dyaw_deg, -2.0, 0.3);
}
TEST_F(Reg, AWrongPlaceFailsTheGates) {
  const auto truth = pose(0, 0, 0.5, 0, 0, 0);
  std::mt19937 g(9); std::uniform_real_distribution<double> u(-6, 6);
  std::vector<Eigen::Vector3f> junk; for (int i = 0; i < 4000; i++) junk.emplace_back(u(g), u(g), 0.5 * u(g));   // random volume: no surfaces
  const auto r = reg.align(junk, truth);
  EXPECT_NE(gate_reason(r.fit, Gates()), nullptr);
}
TEST_F(Reg, TooFewPointsIsNotAnError) {
  const auto r = reg.align({}, pose(0, 0, 0.5, 0, 0, 0));
  EXPECT_STREQ(gate_reason(r.fit, Gates()), "inliers");
}
