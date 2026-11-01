// Points (x, y, z in the target frame, z = height above the floor) -> LaserScan ranges. Same filters and bin layout as pointcloud_to_laserscan
// (height band, planar range, angle, bin = floor((angle - angle_min) / angle_increment)); the difference is what a bin keeps: the `quantile` of its ranges
// (0 = the closest point = pointcloud_to_laserscan, 0.5 = the median, the lower one for an even count), as notes/nav_real_scripts/scan_study.py to_scan(how=...).
#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace scan_median
{

struct Params
{
  double min_height = 0.10, max_height = 1.80;
  double range_min = 0.1, range_max = 60.0;
  double angle_min = -3.14159, angle_max = 3.14159, angle_increment = 0.00873;
  double quantile = 0.5;
  bool use_inf = true;
  double inf_epsilon = 1.0;
};

class Binner
{
public:
  explicit Binner(const Params & p)
  : p_(validated(p)), bins_(size_t(std::ceil((p_.angle_max - p_.angle_min) / p_.angle_increment))) {}

  void begin() {for (auto & b : bins_) {b.clear();}}

  void add(double x, double y, double z)
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {return;}
    if (z > p_.max_height || z < p_.min_height) {return;}
    const double r = std::hypot(x, y);
    if (r < p_.range_min || r > p_.range_max) {return;}
    const double a = std::atan2(y, x);
    if (a < p_.angle_min || a > p_.angle_max) {return;}
    const size_t i = size_t((a - p_.angle_min) / p_.angle_increment);
    if (i < bins_.size()) {bins_[i].push_back(float(r));}
  }

  void finish(std::vector<float> & out)
  {
    const float empty = p_.use_inf ? std::numeric_limits<float>::infinity() : float(p_.range_max + p_.inf_epsilon);
    out.assign(bins_.size(), empty);
    for (size_t i = 0; i < bins_.size(); ++i) {
      auto & b = bins_[i];
      if (b.empty()) {continue;}
      const size_t k = size_t(std::floor(p_.quantile * double(b.size() - 1) + 1e-9));
      std::nth_element(b.begin(), b.begin() + long(k), b.end());
      out[i] = b[k];
    }
  }

  size_t size() const {return bins_.size();}

private:
  /** A launch typo must not become undefined behaviour in the /scan the localizer depends on: bad bins are an error, a quantile outside [0, 1] is the nearest end. */
  static Params validated(Params p)
  {
    if (!(p.angle_increment > 0.0) || !(p.angle_max > p.angle_min)) {
      throw std::invalid_argument("scan_median: angle_increment must be > 0 and angle_max > angle_min");
    }
    p.quantile = std::min(1.0, std::max(0.0, p.quantile));
    return p;
  }

  Params p_;
  std::vector<std::vector<float>> bins_;
};

}  // namespace scan_median
