// scan_median: drop-in for pointcloud_to_laserscan (same parameters, remaps cloud_in / scan, frame handling): the cloud is moved to target_frame at its stamp,
// then binned by scan_median::Binner. `quantile` 0 reproduces pointcloud_to_laserscan; 0.5 (default) keeps the median of each bin. A plain node, as the original.
#include <cmath>
#include <memory>

#include <Eigen/Geometry>

#include "message_filters/subscriber.h"
#include "rclcpp/rclcpp.hpp"
#include "scan_median/binner.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/time.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

using sensor_msgs::msg::LaserScan;
using sensor_msgs::msg::PointCloud2;

class ScanMedianNode : public rclcpp::Node
{
public:
  ScanMedianNode()
  : rclcpp::Node("scan_median")
  {
    target_ = declare_parameter("target_frame", "base_footprint");
    tol_ = declare_parameter("transform_tolerance", 0.2);
    scan_time_ = declare_parameter("scan_time", 0.1);
    const int queue = int(declare_parameter("queue_size", 10));
    scan_median::Params p;
    p.min_height = declare_parameter("min_height", p.min_height);
    p.max_height = declare_parameter("max_height", p.max_height);
    p.range_min = declare_parameter("range_min", p.range_min);
    p.range_max = declare_parameter("range_max", p.range_max);
    p.angle_min = declare_parameter("angle_min", p.angle_min);
    p.angle_max = declare_parameter("angle_max", p.angle_max);
    p.angle_increment = declare_parameter("angle_increment", p.angle_increment);
    p.quantile = declare_parameter("quantile", p.quantile);
    p.use_inf = declare_parameter("use_inf", p.use_inf);
    p.inf_epsilon = declare_parameter("inf_epsilon", p.inf_epsilon);
    p_ = p; binner_ = std::make_unique<scan_median::Binner>(p);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_buffer_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(), get_node_timers_interface()));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pub_ = create_publisher<LaserScan>("scan", rclcpp::SensorDataQoS());
    sub_ = std::make_unique<message_filters::Subscriber<PointCloud2>>(this, "cloud_in", rmw_qos_profile_sensor_data);
    filter_ = std::make_unique<tf2_ros::MessageFilter<PointCloud2>>(
      *sub_, *tf_buffer_, target_, queue, get_node_logging_interface(), get_node_clock_interface(), tf2::durationFromSec(tol_));
    filter_->registerCallback(std::bind(&ScanMedianNode::onCloud, this, std::placeholders::_1));
  }

private:
  void onCloud(const PointCloud2::ConstSharedPtr & m)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(target_, m->header.frame_id, m->header.stamp);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "no %s -> %s at the cloud stamp: %s", m->header.frame_id.c_str(), target_.c_str(), e.what());
      return;
    }
    const Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
    const Eigen::Matrix3d R = q.normalized().toRotationMatrix();
    const Eigen::Vector3d t(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
    binner_->begin();
    sensor_msgs::PointCloud2ConstIterator<float> ix(*m, "x"), iy(*m, "y"), iz(*m, "z");
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      const Eigen::Vector3d w = R * Eigen::Vector3d(*ix, *iy, *iz) + t;
      binner_->add(w.x(), w.y(), w.z());
    }
    LaserScan s;
    s.header.stamp = m->header.stamp; s.header.frame_id = target_;
    s.angle_min = float(p_.angle_min); s.angle_max = float(p_.angle_max); s.angle_increment = float(p_.angle_increment);
    s.time_increment = 0.0; s.scan_time = float(scan_time_); s.range_min = float(p_.range_min); s.range_max = float(p_.range_max);
    binner_->finish(s.ranges);
    pub_->publish(s);
  }

  std::string target_;
  double tol_, scan_time_;
  scan_median::Params p_;
  std::unique_ptr<scan_median::Binner> binner_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<LaserScan>::SharedPtr pub_;
  std::unique_ptr<message_filters::Subscriber<PointCloud2>> sub_;
  std::unique_ptr<tf2_ros::MessageFilter<PointCloud2>> filter_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanMedianNode>());
  rclcpp::shutdown();
  return 0;
}
