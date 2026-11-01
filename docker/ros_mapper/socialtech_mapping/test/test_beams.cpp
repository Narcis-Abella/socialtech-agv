#include <gtest/gtest.h>

#include "socialtech_mapping/beams.hpp"

using namespace socialtech_mapping;

namespace {
const Params kP{{0, 0, 0}, 0.05, 15.0, 0.5, -7.0, 52.0, 0.05, 0.5};
const Pose kIdentity{{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}};

// Bytes of a CustomMsg as CDR: header (stamp, frame_id), timebase, point_num, lidar_id, rsvd[3], points (20-byte elements, the last without padding).
std::vector<uint8_t> custom_msg(int32_t sec, uint32_t nsec, const std::string& frame, const std::vector<std::array<float, 3>>& pts) {
  std::vector<uint8_t> b = {0, 1, 0, 0};
  auto put = [&](const void* p, size_t n) { b.insert(b.end(), static_cast<const uint8_t*>(p), static_cast<const uint8_t*>(p) + n); };
  auto pad = [&](size_t a) { while ((b.size() - 4) % a) b.push_back(0); };
  put(&sec, 4); put(&nsec, 4);
  const uint32_t ns = frame.size() + 1;
  put(&ns, 4); put(frame.c_str(), ns);
  pad(8);
  const uint64_t timebase = 7; put(&timebase, 8);
  const uint32_t n = pts.size(); put(&n, 4);
  b.insert(b.end(), {0, 0, 0, 0});   // lidar_id, rsvd[3]
  put(&n, 4);                        // sequence length
  for (size_t i = 0; i < pts.size(); ++i) {
    const uint32_t t = 0; put(&t, 4); put(pts[i].data(), 12);
    b.insert(b.end(), {9, 0, 1});
    if (i + 1 < pts.size()) b.push_back(0);
  }
  return b;
}
}  // namespace

TEST(Decode, CustomMsgStampPointsAndNonFiniteDropped) {
  std::vector<float> xyz;
  for (std::string frame : {"livox_frame", "ab"}) {   // two frame_id lengths: the 8-byte alignment of timebase differs
    const auto b = custom_msg(100, 500000000, frame, {{{1, 2, 3}}, {{NAN, 0, 0}}, {{4, 5, 6}}});
    EXPECT_DOUBLE_EQ(decode_custom(b.data(), b.size(), xyz), 100.5);
    EXPECT_EQ(xyz, (std::vector<float>{1, 2, 3, 4, 5, 6}));
  }
  auto bad = custom_msg(1, 0, "x", {{{1, 2, 3}}});
  EXPECT_THROW(decode_custom(bad.data(), bad.size() - 6, xyz), std::runtime_error);
}

TEST(Pose, InterpolatesAndRefusesOutsideAndAcrossGaps) {
  const double s = std::sqrt(0.5);   // 0 and 90 deg about z, then a gap of 1 s
  const double poses[] = {0.0, 0, 0, 0, 0, 0, 0, 1, 0.1, 1, 0, 0, 0, 0, s, s, 1.1, 2, 0, 0, 0, 0, s, s};
  auto p = pose_at(poses, 3, 0.05, 0.5);
  ASSERT_TRUE(p);
  EXPECT_NEAR(p->t[0], 0.5, 1e-12);
  const double yaw = std::atan2(p->R[3], p->R[0]);
  EXPECT_NEAR(yaw, std::atan2(s / 2, 0.5 + s / 2) * 2, 1e-12);   // nlerp, as beams_oracle.py
  EXPECT_FALSE(pose_at(poses, 3, -0.01, 0.5));
  EXPECT_FALSE(pose_at(poses, 3, 0.6, 0.5));
  EXPECT_FALSE(pose_at(poses, 3, 1.2, 0.5));
}

TEST(Count, WallHitsInFrontPassesBeforeAndNothingBehindOrOutOfView) {
  // a wall at x = 4: one return per azimuth/elevation bin centre in front of the sensor
  std::vector<float> scan;
  for (double az = -30.5; az < 30; az += 1)
    for (double el = -6.5; el < 51; el += 1) {
      const double a = az / kDeg, e = el / kDeg, r = 4 / (std::cos(e) * std::cos(a));
      scan.insert(scan.end(), {float(r * std::cos(e) * std::cos(a)), float(r * std::cos(e) * std::sin(a)), float(r * std::sin(e))});
    }
  const std::vector<double> C = {4.0, 0.2, 0.3,    // on the wall: hit
                                 2.0, 0.2, 0.3,    // in front of it: passed
                                 6.0, 0.2, 0.3,    // behind it: neither
                                 2.0, 0.2, -1.0,   // under the field of view: neither
                                 0.2, 0.0, 0.0,    // in the blind zone: neither
                                 -3.0, 0.0, 0.0,   // where no beam went: neither
                                 20.0, 0.0, 0.0};  // beyond rmax: not visited
  const CandidateGrid grid(C.data(), 7, 1.0);
  std::vector<int32_t> hs(7, 0), ps(7, 0), hit(7), passed(7);
  std::vector<double> img;
  count_scan(grid, scan, kIdentity, kP, img, hs.data(), ps.data());
  for (long k = 0; k < 7; ++k) { hit[grid.index(k)] = hs[k]; passed[grid.index(k)] = ps[k]; }
  EXPECT_EQ(hit, (std::vector<int32_t>{1, 0, 0, 0, 0, 0, 0}));
  EXPECT_EQ(passed, (std::vector<int32_t>{0, 1, 0, 0, 0, 0, 0}));
}

TEST(Count, PoseMovesCandidatesIntoTheSensorFrame) {
  // the sensor sits at (10, 5) turned 180 deg: a wall 4 m ahead of it in its own frame is at world x = 6
  std::vector<float> scan;
  for (double el = -6.5; el < 51; el += 1) scan.insert(scan.end(), {float(4.0), 0.01f, float(4.0 * std::tan(el / kDeg))});
  const Pose p{{-1, 0, 0, 0, -1, 0, 0, 0, 1}, {10, 5, 0}};
  const std::vector<double> C = {6.0, 4.99, 0.0, 8.0, 4.99, 0.0};
  const CandidateGrid grid(C.data(), 2, 1.0);
  std::vector<int32_t> hs(2, 0), ps(2, 0), hit(2), passed(2);
  std::vector<double> img;
  count_scan(grid, scan, p, kP, img, hs.data(), ps.data());
  for (long k = 0; k < 2; ++k) { hit[grid.index(k)] = hs[k]; passed[grid.index(k)] = ps[k]; }
  EXPECT_EQ(hit, (std::vector<int32_t>{1, 0}));
  EXPECT_EQ(passed, (std::vector<int32_t>{0, 1}));
}

TEST(Grid, VisitsExactlyThePointsWithinTheRadius) {
  std::vector<double> C;
  for (int i = 0; i < 2000; ++i) C.insert(C.end(), {std::fmod(i * 7.31, 40.0) - 20, std::fmod(i * 3.17, 30.0) - 15, std::fmod(i * 0.37, 3.0)});
  const CandidateGrid grid(C.data(), 2000, 1.0);
  const double q[3] = {1.3, -2.2, 0.5};
  std::vector<int> got(2000, 0);
  grid.within(q, 9.0, [&](long k, const double*) { ++got[grid.index(k)]; });
  for (int i = 0; i < 2000; ++i) {
    const double a = C[3 * i] - q[0], b = C[3 * i + 1] - q[1], d = C[3 * i + 2] - q[2];
    EXPECT_EQ(got[i], a * a + b * b + d * d <= 81.0 ? 1 : 0) << i;
  }
}
