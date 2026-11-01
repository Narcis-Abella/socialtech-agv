#include <gtest/gtest.h>

#include <cstdio>

#include "socialtech_livox_sim/caster.hpp"

using namespace livoxsim;

namespace
{
// a 4 x 4 m square room of walls (x = +-2, y = +-2) and a floor at z = 0, two triangles per face, written as the exporter does
std::string write_room()
{
  const std::string path = ::testing::TempDir() + "room.tris";
  const float q[6][4][3] = {
    {{2, -2, 0}, {2, 2, 0}, {2, 2, 3}, {2, -2, 3}}, {{-2, -2, 0}, {-2, 2, 0}, {-2, 2, 3}, {-2, -2, 3}},
    {{-2, 2, 0}, {2, 2, 0}, {2, 2, 3}, {-2, 2, 3}}, {{-2, -2, 0}, {2, -2, 0}, {2, -2, 3}, {-2, -2, 3}},
    {{-2, -2, 0}, {2, -2, 0}, {2, 2, 0}, {-2, 2, 0}}, {{-2, -2, 3}, {2, -2, 3}, {2, 2, 3}, {-2, 2, 3}}};
  FILE * f = std::fopen(path.c_str(), "wb"); const uint32_t n = 12; std::fwrite("TRIS", 1, 4, f); std::fwrite(&n, 4, 1, f);
  for (auto & face : q) {const int t[2][3] = {{0, 1, 2}, {0, 2, 3}}; for (auto & tri : t) {for (int v : tri) {std::fwrite(face[v], 4, 3, f);}}}
  std::fclose(f);
  return path;
}
}  // namespace

TEST(Scene, RaysHitTheWallsAtTheRightDistanceAndMissWhenOutOfRange)
{
  Scene s(write_room()); const float o[3] = {0, 0, 1};
  const float px[3] = {1, 0, 0}, diag[3] = {0.70710678f, 0.70710678f, 0}, down[3] = {0, 0, -1};
  EXPECT_NEAR(s.cast(o, px, 0.1f, 40.f), 2.0f, 1e-4);
  EXPECT_NEAR(s.cast(o, diag, 0.1f, 40.f), 2.0f * std::sqrt(2.0f), 1e-3);
  EXPECT_NEAR(s.cast(o, down, 0.1f, 40.f), 1.0f, 1e-4);
  EXPECT_LT(s.cast(o, px, 0.1f, 1.5f), 0.f);             // beyond range_max
  const float near[3] = {1.95f, 0, 1};
  EXPECT_LT(s.cast(near, px, 0.2f, 40.f), 0.f);          // wall closer than range_min: no return
}

TEST(PoseTrack, InterpolatesTheShortestArcAndExtrapolatesNowhere)
{
  PoseTrack t; t.push({0.0, 0, 0, 3.0}); t.push({1.0, 2, 0, -3.0});
  const Pose m = t.at(0.5);
  EXPECT_NEAR(m.x, 1.0, 1e-12);
  EXPECT_NEAR(std::fabs(std::remainder(m.yaw - M_PI, 2 * M_PI)), 0.0, 0.2);   // through +-pi, not through 0
  EXPECT_NEAR(t.at(-1.0).x, 0.0, 1e-12);                                      // before the oldest: held
}

TEST(Model, CircleOnTheSlotClockAndFanTurnWithTheLineIndex)
{
  Model m; m.f_a = 181.0; m.f_z = 10.0; m.c_ze[0] = 69.0; m.c_ze[4] = 26.0;       // zenith = 69 + 26 sin z on line 0; the rest of the shape is zero
  double az, ze;
  m.direction(5000, az, ze);                                                      // t = 25 ms: z = pi/2, a = 2 pi * 4.525
  EXPECT_NEAR(ze * 180 / M_PI, 95.0, 1e-9);
  EXPECT_NEAR(std::remainder(az * 180 / M_PI - 0.525 * 360.0, 360.0), 0.0, 1e-6);
  m.direction(5003, az, ze);                                                      // line 3: +3 deg azimuth, zenith centre is c_ze[3] = 0 here
  EXPECT_NEAR(std::remainder(az * 180 / M_PI - (0.525 * 360.0 + 3.0 + 3 * 181.0 * 360.0 * kSlot), 360.0), 0.0, 1e-6);
  m.direction(0, az, ze);                                                         // z = 0: sin = 0, cos = 1 (cos terms are zero here)
  EXPECT_NEAR(ze * 180 / M_PI, 69.0, 1e-9);
}
