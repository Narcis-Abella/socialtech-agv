#include <gtest/gtest.h>

#include <chrono>
#include <random>

#include "gn_loc/gn_core.hpp"

using namespace gn_loc;

namespace
{
constexpr int N = 200;
constexpr double RES = 0.025;

// a 4 m square room with a wall segment inside, 2.5 cm cells (same as the Python selftest); blob: a 1.4 m checkerboard of occupied cells at x 1.6-3.0, y 2.9-4.3
// (cluttered furniture as the 2D map shows it: every point inside is within 2 cm of an occupied cell)
std::vector<int8_t> room(bool blob = false)
{
  std::vector<int8_t> g(N * N, 0);
  auto at = [&](int y, int x) -> int8_t & {return g[size_t(y) * N + x];};
  for (int i = 20; i < 180; ++i) {at(20, i) = 100; at(i, 20) = 100; at(180, i) = 100; at(i, 180) = 100;}
  for (int y = 90; y < 92; ++y) {for (int x = 60; x < 140; ++x) {at(y, x) = 100;}}
  if (blob) {for (int y = 116; y < 172; ++y) {for (int x = 64; x < 120; ++x) {if ((x + y) % 2 == 0) {at(y, x) = 100;}}}}
  return g;
}

std::vector<float> scan(const std::vector<int8_t> & g, const Pose & t, std::mt19937 & rng)
{
  std::normal_distribution<double> noise(0, 0.01);
  std::vector<float> r(720, std::numeric_limits<float>::infinity());
  for (int i = 0; i < 720; ++i) {
    const double ang = t.yaw + (-M_PI + 2 * M_PI * i / 720), dx = std::cos(ang), dy = std::sin(ang);
    for (double k = 0.2; k < 6.0; k += 0.01) {
      const int ix = int((t.x + k * dx) / RES), iy = int((t.y + k * dy) / RES);
      if (ix >= 0 && ix < N && iy >= 0 && iy < N && g[size_t(iy) * N + ix] == 100) {r[i] = float(k + noise(rng)); break;}
    }
  }
  return r;
}
}  // namespace

TEST(Field, DistanceIsExact)
{
  const auto g = room();
  Field F(g, N, N, RES, 0.0, 0.0);
  for (int y = 0; y < N; y += 7) {
    for (int x = 0; x < N; x += 7) {
      double best = 1e9;
      for (int j = 0; j < N; ++j) {for (int i = 0; i < N; ++i) {if (g[size_t(j) * N + i] == 100) {best = std::min(best, std::hypot(i - x, j - y));}}}
      ASSERT_NEAR(F.distanceAt(x, y), best * RES, 1e-9) << x << "," << y;
    }
  }
}

TEST(Core, TracksADrivenRobotAndGoesLost)
{
  const auto g = room();
  std::mt19937 rng(0);
  Params p; p.init_search = false;
  Core core(std::make_shared<Field>(g, N, N, RES, 0.0, 0.0), p);
  core.setInitial({1.5, 1.0, 0.0}, 0.05, 3.0 * M_PI / 180.0);
  Result r; Pose o;
  for (int k = 0; k < 12; ++k) {
    const Pose t{1.5 + 0.04 * k, 1.0, 0.02 * k};   // odometry frame starts at the initial pose, with 3 cm / 1 deg of drift
    o = {t.x - 1.5 + 0.03 * (k / 11.0), t.y - 1.0, t.yaw + (M_PI / 180.0) * (k / 11.0)};
    r = core.update(0.1 * k, o, scan(g, t, rng), -M_PI, 2 * M_PI / 720);
    const Pose est = compose(*r.Tmo, o);
    EXPECT_LT(std::hypot(est.x - t.x, est.y - t.y), 0.06);
    EXPECT_LT(std::abs(wrap(est.yaw - t.yaw)), 3.0 * M_PI / 180.0);
  }
  EXPECT_EQ(r.state, State::Tracking);
  EXPECT_GT(r.ratio, 0.5);
  const std::vector<float> junk(720, 5.5f);
  for (int k = 0; k < 60; ++k) {r = core.update(2.0 + 0.1 * k, o, junk, -M_PI, 2 * M_PI / 720);}
  EXPECT_EQ(r.state, State::Lost);
  const Pose keep = *r.Tmo;
  r = core.update(9.0, o, scan(g, {1.9, 1.0, 0.2}, rng), -M_PI, 2 * M_PI / 720);
  EXPECT_EQ(r.Tmo->x, keep.x);   // LOST publishes the last good map->odom until a recovery candidate has passed recover_k good scans
}

TEST(Core, ExhaustiveSearchRecoversAnOffset)
{
  const auto g = room();
  std::mt19937 rng(1);
  Core core(std::make_shared<Field>(g, N, N, RES, 0.0, 0.0), Params{});
  const Pose t{2.0, 1.5, 0.3};
  core.setInitial({2.4, 1.2, 0.0}, 0.3, 15.0 * M_PI / 180.0);   // 0.5 m and 17 deg off
  const auto r = core.update(0.0, {0, 0, 0}, scan(g, t, rng), -M_PI, 2 * M_PI / 720);
  EXPECT_LT(std::hypot(r.Tmo->x - t.x, r.Tmo->y - t.y), 0.05);
  EXPECT_LT(std::abs(wrap(r.Tmo->yaw - t.yaw)), 2.0 * M_PI / 180.0);
}

// regression: a 1 m / 30 deg prior made the single fine grid take ~18 s on the AGX (node declared dead by the lifecycle manager); coarse to fine must capture
// 1 m / 30 deg and 2 m / 40 deg offsets with priors of the same size, in well under the 4 s bond timeout
TEST(Core, ExhaustiveSearchCapturesLargeOffsetsQuickly)
{
  const auto g = room();
  std::mt19937 rng(2);
  struct Case {Pose truth, init; double sxy, syaw_deg;};
  const Case cases[] = {
    {{2.0, 1.5, 0.3}, {2.8, 2.1, 0.3 + 30.0 * M_PI / 180.0}, 1.0, 30.0},
    {{1.2, 3.4, -0.5}, {2.8, 2.2, -0.5 - 40.0 * M_PI / 180.0}, 2.0, 40.0},
  };
  for (const auto & c : cases) {
    Core core(std::make_shared<Field>(g, N, N, RES, 0.0, 0.0), Params{});
    core.setInitial(c.init, c.sxy, c.syaw_deg * M_PI / 180.0);
    const auto t0 = std::chrono::steady_clock::now();
    const auto r = core.update(0.0, {0, 0, 0}, scan(g, c.truth, rng), -M_PI, 2 * M_PI / 720);
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    EXPECT_LT(std::hypot(r.Tmo->x - c.truth.x, r.Tmo->y - c.truth.y), 0.05) << c.sxy;
    EXPECT_LT(std::abs(wrap(r.Tmo->yaw - c.truth.yaw)), 2.0 * M_PI / 180.0) << c.sxy;
    EXPECT_LT(secs, 2.0) << c.sxy;
  }
}

namespace
{
// the robot shuttles along y = `y` (1.4: clear of the walls) between x = 1.0 and 3.4 m at 0.4 m/s (12 s period); odom frame = map frame at the start; scans every 0.1 s
Pose truthAt(double t, double y = 1.4)
{
  const double ph = std::fmod(t, 12.0);
  return {ph < 6.0 ? 1.0 + 0.4 * ph : 1.0 + 0.4 * (12.0 - ph), y, 0.0};
}

struct Drive
{
  Core core;
  std::mt19937 rng{3};
  Result r;
  double err = 0, yerr = 0, worst = 0, y;
  bool blob;
  explicit Drive(const Params & p, double y_ = 1.4, bool blob_ = false)
  : core(std::make_shared<Field>(room(blob_), N, N, RES, 0.0, 0.0), p), y(y_), blob(blob_) {core.setInitial(truthAt(0, y), 0.05, 3.0 * M_PI / 180.0);}
  // blocked: every range 0.6 m (someone against the LiDAR); jump: applied to the odometry on the left (odom' = jump o odom), a lasting jump of the robot pose the odometry reports
  float ring = 0.6f;   // range of every beam of a blocked scan
  void run(double t0, double t1, bool blocked, const Pose & jump = {})
  {
    const auto g = room(blob);
    for (double t = t0; t < t1 - 1e-9; t += 0.1) {
      const Pose truth = truthAt(t, y), o = compose(jump, truth);
      r = core.update(t, o, blocked ? std::vector<float>(720, ring) : scan(g, truth, rng), -M_PI, 2 * M_PI / 720);
      if (r.Tmo) {const Pose est = compose(*r.Tmo, o); err = std::hypot(est.x - truth.x, est.y - truth.y); yerr = std::abs(wrap(est.yaw - truth.yaw)); worst = std::max(worst, err);}
    }
  }
};
}  // namespace

// someone stands against the LiDAR for 10 s: GN goes LOST and must come back to the true pose on its own once the scans are good again (it used to stay LOST until a new /initialpose)
TEST(Core, RecoversFromABlockedSensorWithoutAnInitialPose)
{
  Drive d{Params{}};
  d.run(0.0, 5.0, false);
  ASSERT_EQ(d.r.state, State::Tracking); ASSERT_LT(d.err, 0.03);
  d.run(5.0, 15.0, true);
  EXPECT_EQ(d.r.state, State::Lost);
  d.run(15.0, 25.0, false);
  EXPECT_EQ(d.r.state, State::Tracking);
  EXPECT_LT(d.err, 0.05);
  EXPECT_LT(d.yerr, 1.0 * M_PI / 180.0);
  EXPECT_FALSE(d.r.need_reloc);
}

// blocked 1.1 m from a wall (y = 1.1, wall at 0.5): the 0.6 m ring of the blocked sensor touches the wall, ~27 % of its beams fall within dmax and GN used to fit it, dragging the pose
// (real bags: metres off, then a silent false lock). A bad scan must not move the pose (dead reckoning on the odometry), and recovery must not accept a ring as a pose.
TEST(Core, ABlockedSensorNextToAWallNeitherDragsThePoseNorRecoversOnTheRing)
{
  Drive d{Params{}, 1.1};
  d.run(0.0, 5.0, false);
  ASSERT_EQ(d.r.state, State::Tracking); ASSERT_LT(d.err, 0.03);
  d.worst = 0;
  bool tracked_ring = false;
  for (double t = 5.0; t < 15.0 - 1e-9; t += 0.1) {d.run(t, t + 0.1, true); tracked_ring |= t > 9.0 && d.r.state == State::Tracking;}
  EXPECT_LT(d.worst, 0.03);   // perfect odometry: dead reckoning keeps the pose
  EXPECT_FALSE(tracked_ring);
  d.run(15.0, 25.0, false);
  EXPECT_EQ(d.r.state, State::Tracking);
  EXPECT_LT(d.err, 0.05);
}

// real bag (r20261028_140131, blocked-sensor variant): the recovery search accepted a pose 1.9 m away inside a cluttered area of the map (40-49 % of the cells within 0.6 m occupied),
// where a 0.6 m ring of the blocked sensor scored a consistency of 0.6-0.85. A ring has no pose information (every range equal): such a scan must never fit, search or accept.
// Here the path runs at y = 2.6, 0.3 m above the inner wall and 0.3 m below a cluttered blob; the recovery window grows over the blob while the sensor is blocked.
TEST(Core, ABlockedSensorIsNotLocalisedInsideClutter)
{
  Drive d{Params{}, 2.6, true};
  d.run(0.0, 5.0, false);
  ASSERT_EQ(d.r.state, State::Tracking); ASSERT_LT(d.err, 0.03);
  d.worst = 0;
  bool tracked_ring = false;
  for (double t = 5.0; t < 25.0 - 1e-9; t += 0.1) {d.run(t, t + 0.1, true); tracked_ring |= t > 9.0 && d.r.state == State::Tracking;}
  EXPECT_LT(d.worst, 0.03);
  EXPECT_FALSE(tracked_ring);
  d.run(25.0, 35.0, false);
  EXPECT_EQ(d.r.state, State::Tracking);
  EXPECT_LT(d.err, 0.05);
}

// the odometry jumps (kidnap: the robot keeps to its path, the odometry says it moved): GN must never report TRACKING on a pose more than 0.3 m off once lost_s has passed;
// it finds the true pose again or stays LOST. Known limit, not covered: a jump onto an alias of the map. In this room (0, +1.6 m) lands where 60 % of the beams lie within 5 cm of a wall
// (true lock: ~100 %; real bags: true locks go down to 0.6), so no absolute consistency threshold separates it; only a global check (the relocalizer) can.
TEST(Core, NeverTracksAFalseLockAfterAKidnap)
{
  const Pose jumps[] = {{1.2, 0.0, 10.0 * M_PI / 180.0}, {2.0, 0.5, 40.0 * M_PI / 180.0}, {-0.8, 0.9, -25.0 * M_PI / 180.0}, {0.0, 0.0, 90.0 * M_PI / 180.0}};
  for (const Pose & jump : jumps) {
    Drive d{Params{}};
    d.run(0.0, 5.0, false);
    ASSERT_EQ(d.r.state, State::Tracking);
    double worst = 0;
    for (double t = 5.0; t < 40.0 - 1e-9; t += 0.1) {
      d.run(t, t + 0.1, false, jump);
      if (d.r.state == State::Tracking && t > 5.0 + Params{}.lost_s + 0.5) {worst = std::max(worst, d.err);}
    }
    EXPECT_LT(worst, 0.3) << "jump " << jump.x << " " << jump.y << " " << jump.yaw;
    EXPECT_TRUE(d.r.state == State::Lost || d.err < 0.05) << "jump " << jump.x << " " << jump.y << " " << jump.yaw << " err " << d.err;
  }
}

// scans that never fit the map: after reloc_after_s in LOST the core asks for the global relocalizer, and stops asking once a new initial pose (the relocalizer's) is tracked
TEST(Core, AsksForTheRelocalizerWhenRecoveryFails)
{
  Params p; p.reloc_after_s = 10.0;
  Drive d{p};
  d.run(0.0, 3.0, false);
  d.run(3.0, 9.0, true);
  ASSERT_EQ(d.r.state, State::Lost);
  EXPECT_FALSE(d.r.need_reloc);
  d.run(9.0, 20.0, true);
  EXPECT_TRUE(d.r.need_reloc);
  const Pose t20 = truthAt(20.0);
  d.core.setInitial({t20.x + 0.1, t20.y - 0.05, 0.03}, 0.1, 5.0 * M_PI / 180.0);
  d.run(20.0, 21.0, false);
  EXPECT_EQ(d.r.state, State::Tracking);
  EXPECT_FALSE(d.r.need_reloc);
  EXPECT_LT(d.err, 0.05);
}

// The distance gradient must be the derivative of the interpolated distance it goes with. np.gradient-style central differences of a one-cell wall give 0 on the wall cell
// and 1 only a cell away, so a beam within ~half a cell of the wall (a well-matched one) had a gradient ramping from 0: real maps (5 cm cells, lab_test_01): 79 % of the
// inlier beams of a parked scan had |grad| < 0.5, so the beams that agree with the map carried little weight and the off-wall ones dominated the fit.
TEST(Field, GradientIsTheDerivativeOfTheInterpolatedDistance)
{
  const auto g = room();
  Field F(g, N, N, RES, 0.0, 0.0);
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> u(0.35, 0.65);   // around the wall x = 20 cells, y = 90-91 cells
  double worst = 0;
  for (int k = 0; k < 400; ++k) {
    const double px = (20 + u(rng) * 3 - 1.2) * RES, py = (80 + u(rng) * 30) * RES, e = 1e-6;
    const auto m = F.eval(px, py);
    const double nx = (F.eval(px + e, py).d - F.eval(px - e, py).d) / (2 * e), ny = (F.eval(px, py + e).d - F.eval(px, py - e).d) / (2 * e);
    worst = std::max(worst, std::hypot(m.gx - nx, m.gy - ny));
  }
  EXPECT_LT(worst, 0.02);
}

// A single-scan fit must not be moved by beams that end 8-18 cm from a wall (furniture, people, a door ajar: not in the map). 15 % of the beams of a scan from the true
// pose are shortened by that much, a different 15 % each scan; the fit of 60 such scans (loose prior, as the first scan after a stop) scatters by the numbers below.
// (Real bag, parked 130445: the fit of one parked scan scattered 0.79 deg / 2.5 cm; with 1 cm iid range noise it would be 0.06 deg / 0.2 cm: the scatter was clutter beams
// entering and leaving the 20 cm gate with full quadratic weight, and then the Kalman prior averages what is correlated in time, not white.)
TEST(Fit, ClutterBeamsNearTheWallsBarelyMoveASingleScanFit)
{
  const auto g = room();
  Field F(g, N, N, RES, 0.0, 0.0);
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> u(0, 1), cut(0.08, 0.18);
  Params p;
  const Pose truth{1.5, 1.0, 0.0};
  const Eigen::Matrix3d P0 = Eigen::Vector3d(0.25 * 0.25, 0.25 * 0.25, 0.1 * 0.1).asDiagonal();
  double ex = 0, ey = 0, eyaw = 0;
  const int n = 60;
  for (int k = 0; k < n; ++k) {
    auto r = scan(g, truth, rng);
    for (auto & v : r) {if (std::isfinite(v) && u(rng) < 0.15) {v -= float(cut(rng));}}
    const Fit f = gnUpdate(F, p, beams(r, -M_PI, 2 * M_PI / 720, p), truth, P0, 8);
    ex += std::pow(f.pose.x - truth.x, 2); ey += std::pow(f.pose.y - truth.y, 2); eyaw += std::pow(wrap(f.pose.yaw - truth.yaw), 2);
  }
  const double rms_xy = std::sqrt((ex + ey) / n), rms_yaw = std::sqrt(eyaw / n);
  EXPECT_LT(rms_xy, 0.007);
  EXPECT_LT(rms_yaw, 0.15 * M_PI / 180.0);
}

// The narrow robust kernel must not shrink the capture range: a fit that starts 28 cm / 5 deg off (a jump of the robot or the odometry, a stop-instant error) still has to
// reach the walls. With the Welsch scale fixed at 5 cm, beams 10-20 cm off a wall weigh e^-4 .. e^-16 and cannot pull (real variants: one 30 cm / 5 deg jump never recovered,
// 20 cm off for the rest of the run), so the scale starts at dmax and halves per iteration down to robust_c.
TEST(Fit, TheRobustKernelStillPullsInAnOffsetStart)
{
  const auto g = room();
  Field F(g, N, N, RES, 0.0, 0.0);
  std::mt19937 rng(5);
  Params p;
  const Pose truth{1.5, 1.0, 0.0};
  const Eigen::Matrix3d P0 = Eigen::Vector3d(0.3 * 0.3, 0.3 * 0.3, 0.1 * 0.1).asDiagonal();
  double worst = 0;
  for (const Pose & off : {Pose{0.28, 0, 0}, Pose{0, -0.28, 0}, Pose{0.2, 0.2, 5.0 * M_PI / 180}, Pose{-0.2, 0.2, -5.0 * M_PI / 180}}) {
    const Fit f = gnUpdate(F, p, beams(scan(g, truth, rng), -M_PI, 2 * M_PI / 720, p), {truth.x + off.x, truth.y + off.y, truth.yaw + off.yaw}, P0, 5);
    worst = std::max(worst, std::hypot(f.pose.x - truth.x, f.pose.y - truth.y));
  }
  EXPECT_LT(worst, 0.01);
}

// parked robot (odometry constant): after 30 s of clean scans, 10 s of scans as seen from 3 cm / 1 deg away (a person or moved clutter biasing the fit of a stopped robot).
// Real bags: parked GN map->base wandered 1.4 deg median (3.7 p95) over 144 still windows while FAST-LIO held 0.6 deg. Without parked averaging: 2.88 cm / 1.09 deg (follows
// all of it); with it P shrinks (sx 14 -> 1.1 mm over the 30 s) and the follow halves. It stays above the 25 % of a linear filter (the thin-wall gradient and clutter-weight fixes of notes/L2b_2026-10-31.md
// reduce the real-bag scatter, not this synthetic follow). Then the odometry wanders 1 deg while still (FAST-LIO's
// parked yaw) and the robot drives off: the first moving fits must take the wander out.
TEST(Core, AParkedRobotDoesNotFollowABiasedFit)
{
  const auto g = room();
  std::mt19937 rng(0);
  Params p; p.init_search = false;
  Core core(std::make_shared<Field>(g, N, N, RES, 0.0, 0.0), p);
  const Pose truth{1.5, 1.0, 0.0}, biased{1.53, 1.0, 1.0 * M_PI / 180.0};
  core.setInitial(truth, 0.05, 3.0 * M_PI / 180.0);
  Result r;
  for (int k = 0; k < 300; ++k) {r = core.update(0.1 * k, {0, 0, 0}, scan(g, truth, rng), -M_PI, 2 * M_PI / 720);}
  ASSERT_EQ(r.state, State::Tracking);
  double worst_xy = 0, worst_yaw = 0;
  for (int k = 300; k < 400; ++k) {
    r = core.update(0.1 * k, {0, 0, 0}, scan(g, biased, rng), -M_PI, 2 * M_PI / 720);
    const Pose est = compose(*r.Tmo, {0, 0, 0});
    worst_xy = std::max(worst_xy, std::hypot(est.x - truth.x, est.y - truth.y));
    worst_yaw = std::max(worst_yaw, std::abs(wrap(est.yaw - truth.yaw)));
  }
  EXPECT_LT(worst_xy, 0.6 * 0.03);
  EXPECT_LT(worst_yaw, 0.7 * M_PI / 180.0);
  EXPECT_EQ(r.state, State::Tracking);
  for (int k = 400; k < 500; ++k) {   // 10 s still while the odometry yaw drifts 1 deg (the robot does not move)
    r = core.update(0.1 * k, {0, 0, (k - 400) / 100.0 * M_PI / 180.0}, scan(g, truth, rng), -M_PI, 2 * M_PI / 720);
  }
  const double w = M_PI / 180.0;
  for (int k = 0; k < 20; ++k) {   // drives off at 0.2 m/s: odometry = truth rotated by the 1 deg wander
    const Pose t{1.5 + 0.02 * k, 1.0, 0.0};
    const Pose o = compose({0, 0, w}, {t.x - 1.5, t.y - 1.0, t.yaw});
    r = core.update(50.0 + 0.1 * k, o, scan(g, t, rng), -M_PI, 2 * M_PI / 720);
    if (k >= 15) {
      const Pose est = compose(*r.Tmo, o);
      EXPECT_LT(std::hypot(est.x - t.x, est.y - t.y), 0.02) << k;
      EXPECT_LT(std::abs(wrap(est.yaw - t.yaw)), 0.3 * M_PI / 180.0) << k;
    }
  }
}
