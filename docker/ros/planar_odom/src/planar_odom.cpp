// FAST-LIO2 /Odometry (camera_init -> body) -> odom -> base_footprint TF, in the floor-levelled odom frame. reference: tools/nav_common/planar_odom.py, checked by test_planar_odom_node.py
// (same maths: planar_odom.footprint); also publishes the static odom -> camera_init level transform, so no static_transform_publisher is needed.
// Also the static base_footprint -> lidar_level: levelled, at the height of the lidar, what /scan is published in (a scan on the floor sits on the costmaps' height filter).
// params: sensor_height (m, of the body), level ([qx, qy, qz, qw] of odom <- camera_init), lidar_in_body ([x, y, z] m: FAST-LIO2's extrinsic_T; the body is the IMU, and base_footprint goes under the lidar,
// which is at the centre of rotation: under the IMU it would orbit it, ~2.5 cm, on every turn in place. The offset is applied in the levelled plane: at the +-5 deg of tilt the error is below 4 mm)
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
    lidar_ = declare_parameter("lidar_in_body", std::vector<double>{0.0, 0.0, 0.0});
    if (lidar_.size() != 3) throw std::runtime_error("lidar_in_body must be [x, y, z]");
    const double n = std::sqrt(lv[0] * lv[0] + lv[1] * lv[1] + lv[2] * lv[2] + lv[3] * lv[3]);
    base_ = {lv[0] / n, lv[1] / n, lv[2] / n, lv[3] / n};
    set_level(base_);
    max_corr_rad_ = declare_parameter("max_correction_deg", 5.0) * M_PI / 180;
    corr_timeout_s_ = declare_parameter("correction_timeout_s", 15.0);
    const auto topic = declare_parameter("correction_topic", std::string());
    geometry_msgs::msg::TransformStamped l;
    l.header.stamp = now();
    l.header.frame_id = "base_footprint";
    l.child_frame_id = "lidar_level";
    l.transform.translation.z = h_ + lidar_[2];
    l.transform.rotation.w = 1.0;
    std::vector<geometry_msgs::msg::TransformStamped> statics{l};
    if (topic.empty()) {  // today's behaviour: the level is one static transform
      geometry_msgs::msg::TransformStamped s;
      s.header.stamp = l.header.stamp;
      s.header.frame_id = "odom";
      s.child_frame_id = "camera_init";
      s.transform.rotation.x = base_[0]; s.transform.rotation.y = base_[1]; s.transform.rotation.z = base_[2]; s.transform.rotation.w = base_[3];
      statics.push_back(s);
    } else {
      dynamic_level_ = true;
      corr_sub_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(topic, 10, [this](geometry_msgs::msg::QuaternionStamped::ConstSharedPtr m) { on_correction(*m); });
    }
    static_br_.sendTransform(statics);
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
    last_corr_ = now();
    have_corr_ = true;
  }
  void cb(const nav_msgs::msg::Odometry &m) {
    if (dynamic_level_ && have_corr_ && corr_timeout_s_ > 0 && (now() - last_corr_).seconds() > corr_timeout_s_) {
      set_level(base_);
      have_corr_ = false;
      RCLCPP_WARN(get_logger(), "no level correction for %.0f s: back to the plain level", corr_timeout_s_);
    }
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
    t.transform.translation.x = ox + std::cos(yaw) * lidar_[0] - std::sin(yaw) * lidar_[1];
    t.transform.translation.y = oy + std::sin(yaw) * lidar_[0] + std::cos(yaw) * lidar_[1];
    t.transform.translation.z = oz - h_;
    t.transform.rotation.z = std::sin(yaw / 2);
    t.transform.rotation.w = std::cos(yaw / 2);
    if (!dynamic_level_) {
      br_.sendTransform(t);
      return;
    }
    geometry_msgs::msg::TransformStamped c;
    c.header = t.header;
    c.child_frame_id = "camera_init";
    c.transform.rotation.x = level_[0]; c.transform.rotation.y = level_[1]; c.transform.rotation.z = level_[2]; c.transform.rotation.w = level_[3];
    br_.sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{t, c});
  }
  double max_corr_rad_, corr_timeout_s_ = 15.0;
  bool have_corr_ = false;
  rclcpp::Time last_corr_{0, 0, RCL_ROS_TIME};
  bool dynamic_level_ = false;
  Quat base_, level_;
  double h_;
  std::vector<double> lidar_;
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
