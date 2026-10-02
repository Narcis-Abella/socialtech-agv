// FAST-LIO2 /Odometry (camera_init -> body) -> odom -> base_footprint TF, in the floor-levelled odom frame. reference: tools/nav_bench/planar_odom.py, checked by test_planar_odom_node.py
// (same maths: planar_odom.footprint); also publishes the static odom -> camera_init level transform, so no static_transform_publisher is needed.
// params: sensor_height (m), level ([qx, qy, qz, qw] of odom <- camera_init)
#include <array>
#include <cmath>
#include <vector>

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

class PlanarOdom : public rclcpp::Node {
 public:
  PlanarOdom() : Node("planar_odom"), br_(this), static_br_(this) {
    h_ = declare_parameter("sensor_height", 0.575);
    auto lv = declare_parameter("level", std::vector<double>{0.0, 0.0, 0.0, 1.0});
    if (lv.size() != 4) throw std::runtime_error("level must be [qx, qy, qz, qw]");
    Rl_ = quat_to_mat(lv[0], lv[1], lv[2], lv[3]);
    geometry_msgs::msg::TransformStamped s;
    s.header.stamp = now();
    s.header.frame_id = "odom";
    s.child_frame_id = "camera_init";
    const double n = std::sqrt(lv[0] * lv[0] + lv[1] * lv[1] + lv[2] * lv[2] + lv[3] * lv[3]);
    s.transform.rotation.x = lv[0] / n; s.transform.rotation.y = lv[1] / n; s.transform.rotation.z = lv[2] / n; s.transform.rotation.w = lv[3] / n;
    static_br_.sendTransform(s);
    sub_ = create_subscription<nav_msgs::msg::Odometry>("/Odometry", 100, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { cb(*m); });
  }

 private:
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
    br_.sendTransform(t);
  }
  double h_;
  Mat Rl_;
  tf2_ros::TransformBroadcaster br_;
  tf2_ros::StaticTransformBroadcaster static_br_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanarOdom>());
  rclcpp::shutdown();
}
