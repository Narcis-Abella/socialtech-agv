// Tests of the pure core. Golden matrices come from the validated Python reference (notes/orientacion_scripts/maptrack.py: tilt_rot, lowpass).
#include <gtest/gtest.h>

#include "level_corrector/core.hpp"

using namespace level_corrector;

static Mat3 M(std::initializer_list<double> v) {
  Mat3 m; int i = 0;
  for (double x : v) m(i / 3, i % 3) = x, i++;
  return m;
}
static Mat3 rv(double rx_deg, double ry_deg) {
  const double d = M_PI / 180;
  const Vec3 v(rx_deg * d, ry_deg * d, 0);   // one rotation vector, as scipy's Rotation.from_rotvec in the reference
  return v.norm() < 1e-15 ? Mat3::Identity() : Eigen::AngleAxisd(v.norm(), v.normalized()).toRotationMatrix();
}
static double deg_between(const Mat3& a, const Mat3& b) { return Eigen::AngleAxisd(a * b.transpose()).angle() * 180 / M_PI; }

TEST(TiltRot, MatchesPython) {
  Vec3 u(0.01, -0.02, 1.0); u.normalize();
  const Mat3 want = M({0.9999500187421909, 9.996251561816717e-05, -0.009997500937109547, 9.996251561816717e-05, 0.9998000749687638, 0.019995001874219095,
                       0.009997500937109547, -0.019995001874219095, 0.9997500937109547});
  EXPECT_LT((tilt_rot(u, Vec3::UnitZ()) - want).cwiseAbs().maxCoeff(), 1e-12);
}
TEST(TiltRot, SameVectorIsIdentityAndOppositeDoesNotBlowUp) {
  EXPECT_LT((tilt_rot(Vec3::UnitZ(), Vec3::UnitZ()) - Mat3::Identity()).norm(), 1e-12);
  EXPECT_TRUE(tilt_rot(Vec3::UnitZ(), -Vec3::UnitZ()).allFinite());
}
TEST(Lowpass, MatchesPythonFreeAndSlewLimited) {
  const Mat3 C = rv(0.3, -0.2), Cm = rv(0.9, 0.4);
  const Mat3 free = M({0.9999987320997823, -5.680344779914681e-06, -0.0015924090434222336, -5.680344779914681e-06, 0.9999745513752837, -0.007134183172146254,
                       0.0015924090434222332, 0.007134183172146255, 0.999973283475066});
  const Mat3 slew = M({0.9999974540445904, -7.299997314805565e-06, -0.002256513028375596, -7.299997314805566e-06, 0.9999790687768543, -0.006470081520770453,
                       0.002256513028375597, 0.006470081520770453, 0.9999765228214447});
  EXPECT_LT((lowpass(C, Cm, 2.0, 10.0, 5.0) - free).cwiseAbs().maxCoeff(), 1e-9);
  EXPECT_LT((lowpass(C, Cm, 2.0, 10.0, 0.05) - slew).cwiseAbs().maxCoeff(), 1e-9);
}
TEST(Gates, EachLimitRejectsWithItsOwnReason) {
  Gates g; Fit ok{1000, 1050, 0.02, 0.1, 1.0};   // inliers, source points, rms, dxy, dyaw
  EXPECT_EQ(gate_reason(ok, g), nullptr);
  EXPECT_STREQ(gate_reason({299, 300, 0.02, 0.1, 1.0}, g), "inliers");     // absolute floor: too few constraints even if all of them fit
  EXPECT_STREQ(gate_reason({700, 1000, 0.02, 0.1, 1.0}, g), "ratio");      // 70 % of the scan fits: seen in the bad fits (68-78 %), good ones are 98 %
  EXPECT_STREQ(gate_reason({1000, 1050, 0.031, 0.1, 1.0}, g), "rms");
  EXPECT_STREQ(gate_reason({1000, 1050, 0.02, 0.31, 1.0}, g), "dxy");
  EXPECT_STREQ(gate_reason({1000, 1050, 0.02, 0.1, -3.1}, g), "dyaw");
  EXPECT_STREQ(gate_reason({1000, 1050, 0.02, NAN, 1.0}, g), "dxy");       // a NaN must not pass a "<=" test written as "!(x > max)"
  EXPECT_STREQ(gate_reason({0, 0, INFINITY, INFINITY, 0}, g), "inliers");  // an empty scan: no division by zero in the ratio
}
TEST(Filter, StartsAtIdentityAndTakesTheFirstMeasurementWhole) {
  Filter f(10.0, 0.05, 2.0);
  EXPECT_FALSE(f.primed());
  EXPECT_LT((f.C() - Mat3::Identity()).norm(), 1e-12);
  EXPECT_EQ(f.update(rv(0.8, -0.4), 0.0), Filter::Primed);
  EXPECT_LT(deg_between(f.C(), rv(0.8, -0.4)), 1e-9);
}
TEST(Filter, FollowsSlowlyAndNeverFasterThanTheSlew) {
  Filter f(10.0, 0.05, 2.0); f.update(rv(0, 0), 0.0);
  EXPECT_EQ(f.update(rv(1.0, 0), 2.0), Filter::Accepted);
  EXPECT_NEAR(deg_between(f.C(), rv(0, 0)), 0.05 * 2.0, 1e-6);   // slew-limited: 0.1 deg in 2 s, the low-pass alone would give 0.18
}
TEST(Filter, RejectsAJumpAndKeepsC) {
  Filter f(10.0, 0.05, 2.0); f.update(rv(0, 0), 0.0);
  const Mat3 before = f.C();
  EXPECT_EQ(f.update(rv(2.5, 0), 2.0), Filter::Jump);
  EXPECT_LT((f.C() - before).norm(), 1e-12);
}
TEST(Filter, ResetGoesBackToTheIdentityAndTheNextMeasurementIsTakenWholeEvenFarFromTheOldC) {
  Filter f(10.0, 0.05, 2.0); f.update(rv(0.6, 0.5), 0.0);
  f.reset();   // FAST-LIO restarted: the old C belonged to the old camera_init
  EXPECT_FALSE(f.primed());
  EXPECT_LT((f.C() - Mat3::Identity()).norm(), 1e-12);
  EXPECT_EQ(f.update(rv(4.0, 1.0), 2.0), Filter::Primed);
  EXPECT_LT(deg_between(f.C(), rv(4.0, 1.0)), 1e-9);
}
TEST(Filter, ResetBeforeTheFirstMeasurementIsHarmless) {
  Filter f(10.0, 0.05, 2.0); f.reset();
  EXPECT_EQ(f.update(rv(0.5, 0.5), 1.0), Filter::Primed);
}
