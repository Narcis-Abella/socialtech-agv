// ObstacleLayer plus expiry: a lethal cell that no marking observation has hit for `observation_expiry` seconds is freed.
// Inside the sensor's view nothing changes: the base class still clears it with the ray as soon as it is free (and a cell that keeps being hit keeps its stamp).
// Only cells nobody observes any more (out of range, occluded, behind the robot) age out. observation_expiry 0 (default) = the stock ObstacleLayer.
// How a re-mark is detected without copying the base class's range/height filters: before the base update every lethal cell is demoted to a sentinel; the
// base writes LETHAL again on the cells it marks and FREE on the ones a ray clears, so a cell still holding the sentinel afterwards was not observed this cycle.
// It relies on ObstacleLayer of Nav2 1.3.13 (checked in its source): Costmap2D::mutex_t is a recursive_mutex and updateBounds locks it, rays clear before the
// marking loop writes LETHAL, and rolling_window_ is protected. Re-check those three after a Nav2 upgrade.
#include <algorithm>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace socialtech_costmap
{

class DecayObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  void onInitialize() override
  {
    nav2_costmap_2d::ObstacleLayer::onInitialize();
    declareParameter("observation_expiry", rclcpp::ParameterValue(0.0));
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error{"DecayObstacleLayer: failed to lock node"};
    }
    node->get_parameter(name_ + "." + "observation_expiry", expiry_);
    if (expiry_ < 0.0) {
      RCLCPP_WARN(logger_, "%s: observation_expiry %.1f is negative: the layer behaves as the stock ObstacleLayer", name_.c_str(), expiry_);
    }
    // shortcut: a rolling window shifts the grid under the stamps; only the (static-size) global costmap uses expiry, so refuse it elsewhere
    if (expiry_ > 0.0 && rolling_window_) {
      throw std::runtime_error{"DecayObstacleLayer: observation_expiry needs a non-rolling costmap"};
    }
    stamp_.assign(getSizeInCellsX() * getSizeInCellsY(), 0.0);
  }

  void matchSize() override
  {
    nav2_costmap_2d::ObstacleLayer::matchSize();
    stamp_.assign(getSizeInCellsX() * getSizeInCellsY(), 0.0);
  }

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override
  {
    if (expiry_ <= 0.0 || !enabled_) {
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
    const double now = clock_->now().seconds();
    unsigned int expired = 0;
    for (unsigned int i = 0; i < n; ++i) {
      if (costmap_[i] == nav2_costmap_2d::LETHAL_OBSTACLE) {
        stamp_[i] = now;                                    // observed again this cycle
      } else if (costmap_[i] == kNotObserved) {
        if (now - stamp_[i] > expiry_) {
          costmap_[i] = nav2_costmap_2d::FREE_SPACE;        // expired
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

private:
  static constexpr unsigned char kNotObserved = 253;       // never written by ObstacleLayer, which only uses FREE, LETHAL and NO_INFORMATION
  double expiry_{0.0};
  std::vector<double> stamp_;                              // sim or wall time of the last observation of each cell
};

}  // namespace socialtech_costmap

PLUGINLIB_EXPORT_CLASS(socialtech_costmap::DecayObstacleLayer, nav2_costmap_2d::Layer)
