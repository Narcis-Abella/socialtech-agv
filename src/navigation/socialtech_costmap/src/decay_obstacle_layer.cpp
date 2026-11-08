// ObstacleLayer plus two optional behaviours, both off by default (then it is the stock ObstacleLayer):
//  - observation_expiry S > 0: a lethal cell that no marking observation has hit for S seconds is freed. Inside the sensor's view nothing changes: the base class still
//    clears it with the ray as soon as it is free (and a cell that keeps being hit keeps its stamp). Only cells nobody observes any more age out.
//  - height_band_clearing true (needs a PointCloud2 source: a LaserScan has no height and degenerates to the stock behaviour): a 2D ray does NOT clear a marked cell if the 3D ray
//    passed over it, i.e. above the highest point that ever hit that cell (+ band_margin). The stock layer clears whatever a ray passes over, so a low object that leaves the lidar's
//    field of view (a Mid-360 at 0.57 m sees nothing below 0.57 - d tan 7 deg: a 25 cm box is out of it nearer than ~2.6 m) is freed by the rays that go over it and hit the wall
//    behind. Only the upper side is protected: a ray below the marks cannot have gone through an object standing on the floor, so it is either evidence or a z error (a transient z
//    or attitude error of the localizer would otherwise leave marks no ray can free). A cell with no recorded height (never marked, or marked by a point the layer did not see) is
//    cleared as in the stock layer.
// How a re-mark is detected without copying the base class's range filters: before the base update every lethal cell is demoted to a sentinel; the base writes LETHAL again on
// the cells it marks and FREE on the ones a ray clears, so a cell still holding the sentinel afterwards was not observed this cycle (it is restored, or freed when expired).
// It relies on ObstacleLayer of Nav2 1.3.13 (checked in its source): Costmap2D::mutex_t is a recursive_mutex and updateBounds locks it, rays clear before the marking loop writes
// LETHAL, rolling_window_ is protected, and raytraceFreespace is virtual (its body is ported below, only the cell action differs). Re-check those after a Nav2 upgrade.
#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "nav2_costmap_2d/observation.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace socialtech_costmap
{

// What each cell on a 2D ray does: free it, unless the 3D ray is above the highest point that marked it (+ margin) at this distance along the ray.
struct BandClear
{
  unsigned char * costmap;
  float * zmax;
  unsigned int size_x, x0, y0;
  double length_cells, oz, wz, margin;

  void operator()(unsigned int offset)
  {
    if (!std::isnan(zmax[offset])) {
      const double along = length_cells > 0.0 ?
        std::hypot(static_cast<double>(offset % size_x) - x0, static_cast<double>(offset / size_x) - y0) / length_cells : 1.0;
      const double z = oz + (wz - oz) * std::min(along, 1.0);
      if (z > zmax[offset] + margin) {
        return;                                              // the ray went over what stood here: no evidence that it is gone
      }
    }
    costmap[offset] = nav2_costmap_2d::FREE_SPACE;
    zmax[offset] = std::numeric_limits<float>::quiet_NaN();
  }
};

class DecayObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  void onInitialize() override
  {
    nav2_costmap_2d::ObstacleLayer::onInitialize();
    declareParameter("observation_expiry", rclcpp::ParameterValue(0.0));
    declareParameter("height_band_clearing", rclcpp::ParameterValue(false));
    declareParameter("band_margin", rclcpp::ParameterValue(0.05));
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error{"DecayObstacleLayer: failed to lock node"};
    }
    node->get_parameter(name_ + "." + "observation_expiry", expiry_);
    node->get_parameter(name_ + "." + "height_band_clearing", band_);
    node->get_parameter(name_ + "." + "band_margin", margin_);
    if (expiry_ < 0.0) {
      RCLCPP_WARN(logger_, "%s: observation_expiry %.1f is negative: the layer behaves as the stock ObstacleLayer", name_.c_str(), expiry_);
    }
    // shortcut: a rolling window shifts the grid under the stamps and the bands; only the (static-size) global costmap uses them, so refuse it elsewhere
    if ((expiry_ > 0.0 || band_) && rolling_window_) {
      throw std::runtime_error{"DecayObstacleLayer: observation_expiry and height_band_clearing need a non-rolling costmap"};
    }
    resetState();
  }

  void matchSize() override
  {
    nav2_costmap_2d::ObstacleLayer::matchSize();
    resetState();
  }

  void reset() override
  {
    nav2_costmap_2d::ObstacleLayer::reset();
    resetState();                                            // clear_entirely_* frees the cells: what stood there is forgotten too
  }

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override
  {
    if ((expiry_ <= 0.0 && !band_) || !enabled_) {
      nav2_costmap_2d::ObstacleLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
      return;
    }
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());   // recursive: the base class locks it again
    const unsigned int n = getSizeInCellsX() * getSizeInCellsY();
    for (unsigned int i = 0; i < n; ++i) {
      if (costmap_[i] == nav2_costmap_2d::LETHAL_OBSTACLE) {
        costmap_[i] = kNotObserved;
      }
    }
    nav2_costmap_2d::ObstacleLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
    if (band_) {
      markBands();
    }
    const double now = clock_->now().seconds();
    unsigned int expired = 0;
    for (unsigned int i = 0; i < n; ++i) {
      if (costmap_[i] == nav2_costmap_2d::LETHAL_OBSTACLE) {
        stamp_[i] = now;                                    // observed again this cycle
      } else if (costmap_[i] == kNotObserved) {
        if (expiry_ > 0.0 && now - stamp_[i] > expiry_) {
          costmap_[i] = nav2_costmap_2d::FREE_SPACE;        // expired
          zmax_[i] = std::numeric_limits<float>::quiet_NaN();
          ++expired;
          double wx, wy;
          mapToWorld(i % getSizeInCellsX(), i / getSizeInCellsX(), wx, wy);
          touch(wx, wy, min_x, min_y, max_x, max_y);        // the master grid has to re-read this cell
        } else {
          costmap_[i] = nav2_costmap_2d::LETHAL_OBSTACLE;   // still remembered
        }
      }
    }
    if (expired > 0) {
      RCLCPP_INFO_THROTTLE(logger_, *clock_, 5000, "%s: %u cells not observed for %.0f s expired", name_.c_str(), expired, expiry_);
    }
  }

protected:
  // Port of ObstacleLayer::raytraceFreespace (Nav2 1.3.13): the same endpoint clamping and ranges, but the cell action looks at the ray's height and each point's z is kept.
  void raytraceFreespace(
    const nav2_costmap_2d::Observation & clearing_observation, double * min_x, double * min_y,
    double * max_x, double * max_y) override
  {
    if (!band_) {
      nav2_costmap_2d::ObstacleLayer::raytraceFreespace(clearing_observation, min_x, min_y, max_x, max_y);
      return;
    }
    const double ox = clearing_observation.origin_.x;
    const double oy = clearing_observation.origin_.y;
    const double oz = clearing_observation.origin_.z;
    const sensor_msgs::msg::PointCloud2 & cloud = *(clearing_observation.cloud_);
    unsigned int x0, y0;
    if (!worldToMap(ox, oy, x0, y0)) {
      RCLCPP_WARN(
        logger_, "Sensor origin at (%.2f, %.2f) is out of map bounds (%.2f, %.2f) to (%.2f, %.2f). The costmap cannot raytrace for it.",
        ox, oy, origin_x_, origin_y_, origin_x_ + getSizeInMetersX(), origin_y_ + getSizeInMetersY());
      return;
    }
    const double origin_x = origin_x_, origin_y = origin_y_;
    const double map_end_x = origin_x + size_x_ * resolution_;
    const double map_end_y = origin_y + size_y_ * resolution_;
    touch(ox, oy, min_x, min_y, max_x, max_y);
    const unsigned int cell_raytrace_max_range = cellDistance(clearing_observation.raytrace_max_range_);
    const unsigned int cell_raytrace_min_range = cellDistance(clearing_observation.raytrace_min_range_);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      double wx = *iter_x;
      double wy = *iter_y;
      const double wz = *iter_z;
      const double a = wx - ox;
      const double b = wy - oy;
      if (wx < origin_x) {
        const double t = (origin_x - ox) / a;
        wx = origin_x;
        wy = oy + b * t;
      }
      if (wy < origin_y) {
        const double t = (origin_y - oy) / b;
        wx = ox + a * t;
        wy = origin_y;
      }
      if (wx > map_end_x) {
        const double t = (map_end_x - ox) / a;
        wx = map_end_x - .001;
        wy = oy + b * t;
      }
      if (wy > map_end_y) {
        const double t = (map_end_y - oy) / b;
        wx = ox + a * t;
        wy = map_end_y - .001;
      }
      unsigned int x1, y1;
      if (!worldToMap(wx, wy, x1, y1)) {
        continue;
      }
      const double full = std::hypot(a, b);                  // a clamped endpoint is on the same 3D ray: its height follows the fraction of the way
      const double kept = std::hypot(wx - ox, wy - oy);
      const double end_z = full > 0.0 ? oz + (wz - oz) * std::min(kept / full, 1.0) : wz;
      BandClear action{costmap_, zmax_.data(), size_x_, x0, y0,
        std::hypot(static_cast<double>(x1) - x0, static_cast<double>(y1) - y0), oz, end_z, margin_};
      raytraceLine(action, x0, y0, x1, y1, cell_raytrace_max_range, cell_raytrace_min_range);
      updateRaytraceBounds(
        ox, oy, wx, wy, clearing_observation.raytrace_max_range_, clearing_observation.raytrace_min_range_, min_x, min_y, max_x, max_y);
    }
  }

private:
  static constexpr unsigned char kNotObserved = 253;       // never written by ObstacleLayer, which only uses FREE, LETHAL and NO_INFORMATION

  void resetState()
  {
    const size_t n = static_cast<size_t>(getSizeInCellsX()) * getSizeInCellsY();
    stamp_.assign(n, 0.0);
    zmax_.assign(n, std::numeric_limits<float>::quiet_NaN());
  }

  // After the base update the cells holding LETHAL were marked this cycle: raise their top with the heights of the points that fell in them (same height filter as the base).
  // shortcut: the marking observations are read again, so a cloud that arrived between the two reads raises the tops of cells it did not mark yet; the next cycle marks them
  void markBands()
  {
    std::vector<nav2_costmap_2d::Observation> observations;
    getMarkingObservations(observations);
    for (const auto & obs : observations) {
      const sensor_msgs::msg::PointCloud2 & cloud = *(obs.cloud_);
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        const float pz = *iter_z;
        if (pz < min_obstacle_height_ || pz > max_obstacle_height_) {
          continue;
        }
        unsigned int mx, my;
        if (!worldToMap(*iter_x, *iter_y, mx, my)) {
          continue;
        }
        const unsigned int i = getIndex(mx, my);
        if (costmap_[i] != nav2_costmap_2d::LETHAL_OBSTACLE) {
          continue;
        }
        zmax_[i] = std::isnan(zmax_[i]) ? pz : std::max(zmax_[i], pz);
      }
    }
  }

  double expiry_{0.0};
  bool band_{false};
  double margin_{0.05};
  std::vector<double> stamp_;                              // sim or wall time of the last observation of each cell
  std::vector<float> zmax_;                                // highest point (global frame) that marked the cell; NaN: none recorded
};

}  // namespace socialtech_costmap

PLUGINLIB_EXPORT_CLASS(socialtech_costmap::DecayObstacleLayer, nav2_costmap_2d::Layer)
