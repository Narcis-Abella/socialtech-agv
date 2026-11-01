#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <fstream>
#include <sstream>

#include "scan_median/binner.hpp"

using scan_median::Binner;
using scan_median::Params;

namespace
{
std::vector<float> run(Params p, const std::vector<std::array<double, 3>> & pts)
{
  Binner b(p); b.begin();
  for (const auto & q : pts) {b.add(q[0], q[1], q[2]);}
  std::vector<float> out; b.finish(out); return out;
}
}  // namespace

TEST(Binner, MatchesPythonReferenceOnRandomPoints)   // golden.txt: make_golden.py, scan_study.to_scan
{
  std::ifstream f(GOLDEN_PATH); ASSERT_TRUE(f.good()) << GOLDEN_PATH;
  size_t n; f >> n; std::vector<std::array<double, 3>> pts(n);
  for (auto & p : pts) {f >> p[0] >> p[1] >> p[2];}
  std::string word; int cases = 0;
  while (f >> word) {
    ASSERT_EQ(word, "case");
    Params p; double rmin; f >> p.quantile >> rmin; p.range_min = rmin;
    const auto got = run(p, pts);
    ASSERT_EQ(got.size(), 720u);
    int finite = 0;
    for (size_t i = 0; i < 720; ++i) {
      std::string tok; f >> tok;
      const double want = tok == "inf" ? INFINITY : std::stod(tok);
      if (std::isinf(want)) {EXPECT_TRUE(std::isinf(got[i])) << "q " << p.quantile << " bin " << i;} else {EXPECT_NEAR(got[i], want, 1e-5) << "q " << p.quantile << " bin " << i; ++finite;}
    }
    EXPECT_GT(finite, 300);   // the case is not trivially empty
    ++cases;
  }
  EXPECT_EQ(cases, 3);
}

TEST(Binner, FiltersAndQuantiles)
{
  Params p; p.range_min = 0.5;
  // heights, range band, non-finite
  EXPECT_TRUE(std::isinf(run(p, {{3, 0, 0.05}})[360]));    // below the band
  EXPECT_TRUE(std::isinf(run(p, {{3, 0, 2.0}})[360]));     // above the band
  EXPECT_TRUE(std::isinf(run(p, {{0.3, 0, 0.5}})[360]));   // closer than range_min
  EXPECT_TRUE(std::isinf(run(p, {{NAN, 0, 0.5}})[360]));
  // a bin with 3, 4, 5, 6, 7 m: min / median / 0.25 / max
  const std::vector<std::array<double, 3>> col{{7, 0, 0.5}, {3, 0, 0.5}, {5, 0, 0.5}, {4, 0, 0.5}, {6, 0, 0.5}};
  const size_t k = size_t((0 - p.angle_min) / p.angle_increment);
  p.quantile = 0.0; EXPECT_FLOAT_EQ(run(p, col)[k], 3.0f);
  p.quantile = 0.5; EXPECT_FLOAT_EQ(run(p, col)[k], 5.0f);
  p.quantile = 0.25; EXPECT_FLOAT_EQ(run(p, col)[k], 4.0f);
  p.quantile = 1.0; EXPECT_FLOAT_EQ(run(p, col)[k], 7.0f);
  p.quantile = 0.5; p.use_inf = false; EXPECT_FLOAT_EQ(run(p, {})[0], float(p.range_max + p.inf_epsilon));
  // even count: the lower median
  p.use_inf = true; EXPECT_FLOAT_EQ(run(p, {{3, 0, 0.5}, {4, 0, 0.5}})[k], 3.0f);
}
