// FAST-LIO2 /Odometry (camera_init -> body) -> odom -> base_footprint TF, in the floor-levelled odom frame. reference: tools/nav_bench/planar_odom.py, checked by test_planar_odom_node.py
// (same maths: planar_odom.footprint); also publishes the static odom -> camera_init level transform, so no static_transform_publisher is needed.
// params: sensor_height (m), level ([qx, qy, qz, qw] of odom <- camera_init), correction_topic ("" = off: odom -> camera_init is the static level, as always).
// With a topic (level_corrector's ~/correction, QuaternionStamped: a rotation C of odom) the level becomes C * level: odom -> camera_init is then dynamic (stamped with /Odometry; a static one
// would be a second publisher of the same child) and base_footprint uses it too, so the floor and the heights it gives stay flat while FAST-LIO2's gravity estimate wanders.
// A C that is not finite, not a unit quaternion or turns more than max_correction_deg is ignored (the last good one stays; none yet = the plain level).
#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

using Mat = std::array<double, 9>;  // row-major

static Mat quat_to_mat(double x, double y, double z, double w) {
  const double n = std::sqrt(x * x + y * y + z * z + w * w);
  x /= n; y /= n; z /= n; w /= n;
  return {1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
          2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
          2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)};
}

using Quat = std::array<double, 4>;  // x, y, z, w

static Quat quat_mul(const Quat &a, const Quat &b) {  // Hamilton product a * b
  return {a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1], a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
          a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3], a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]};
}

class PlanarOdom : public rclcpp::Node {
 public:
  PlanarOdom() : Node("planar_odom"), br_(this), static_br_(this) {
    h_ = declare_parameter("sensor_height", 0.575);
    auto lv = declare_parameter("level", std::vector<double>{0.0, 0.0, 0.0, 1.0});
    if (lv.size() != 4) throw std::runtime_error("level must be [qx, qy, qz, qw]");
    const double n = std::sqrt(lv[0] * lv[0] + lv[1] * lv[1] + lv[2] * lv[2] + lv[3] * lv[3]);
    base_ = {lv[0] / n, lv[1] / n, lv[2] / n, lv[3] / n};
    set_level(base_);
    max_corr_rad_ = declare_parameter("max_correction_deg", 5.0) * M_PI / 180;
    const auto topic = declare_parameter("correction_topic", std::string());
    if (topic.empty()) {  // today's behaviour: one static transform
      geometry_msgs::msg::TransformStamped s;
      s.header.stamp = now();
      s.header.frame_id = "odom";
      s.child_frame_id = "camera_init";
      s.transform.rotation.x = base_[0]; s.transform.rotation.y = base_[1]; s.transform.rotation.z = base_[2]; s.transform.rotation.w = base_[3];
      static_br_.sendTransform(s);
    } else {
      dynamic_level_ = true;
      corr_sub_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(topic, 10, [this](geometry_msgs::msg::QuaternionStamped::ConstSharedPtr m) { on_correction(*m); });
    }
    sub_ = create_subscription<nav_msgs::msg::Odometry>("/Odometry", 100, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { cb(*m); });
  }

 private:
  void set_level(const Quat &q) {
    level_ = q;
    Rl_ = quat_to_mat(q[0], q[1], q[2], q[3]);
  }
  void on_correction(const geometry_msgs::msg::QuaternionStamped &m) {
    const auto &q = m.quaternion;
    const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!std::isfinite(n) || std::abs(n - 1.0) > 0.01 || 2 * std::acos(std::min(1.0, std::abs(q.w / n))) > max_corr_rad_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "ignoring a level correction that is not a small unit quaternion (norm %.3f, w %.3f)", n, q.w);
      return;
    }
    set_level(quat_mul({q.x / n, q.y / n, q.z / n, q.w / n}, base_));
  }
  void cb(const nav_msgs::msg::Odometry &m) {
    const auto &p = m.pose.pose.position;
    const auto &q = m.pose.pose.orientation;
    const Mat Rq = quat_to_mat(q.x, q.y, q.z, q.w);
    // po = Rl p ; yaw of Ro = Rl Rq from its first column: atan2(Ro[1][0], Ro[0][0])
    const double ox = Rl_[0] * p.x + Rl_[1] * p.y + Rl_[2] * p.z, oy = Rl_[3] * p.x + Rl_[4] * p.y + Rl_[5] * p.z,
                 oz = Rl_[6] * p.x + Rl_[7] * p.y + Rl_[8] * p.z;
    const double r00 = Rl_[0] * Rq[0] + Rl_[1] * Rq[3] + Rl_[2] * Rq[6], r10 = Rl_[3] * Rq[0] + Rl_[4] * Rq[3] + Rl_[5] * Rq[6];
    const double yaw = std::atan2(r10, r00);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = m.header.stamp;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_footprint";
    t.transform.translation.x = ox;
    t.transform.translation.y = oy;
    t.transform.translation.z = oz - h_;
    t.transform.rotation.z = std::sin(yaw / 2);
    t.transform.rotation.w = std::cos(yaw / 2);
    if (!dynamic_level_) {
      br_.sendTransform(t);
      return;
    }
    geometry_msgs::msg::TransformStamped l;
    l.header = t.header;
    l.child_frame_id = "camera_init";
    l.transform.rotation.x = level_[0]; l.transform.rotation.y = level_[1]; l.transform.rotation.z = level_[2]; l.transform.rotation.w = level_[3];
    br_.sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{t, l});
  }
  double h_, max_corr_rad_;
  bool dynamic_level_ = false;
  Quat base_, level_;
  Mat Rl_;
  tf2_ros::TransformBroadcaster br_;
  tf2_ros::StaticTransformBroadcaster static_br_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr corr_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanarOdom>());
  rclcpp::shutdown();
}
