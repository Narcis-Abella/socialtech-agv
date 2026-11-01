// gn_loc: a lifecycle node (as nav2_amcl) that computes map->odom by Gauss-Newton scan-to-map tracking, standing where nav2_amcl stands.
// configure: parameters, TF, subscriptions (map, initialpose, scan through the MessageFilter). activate: publishers and the status timer. Scans are ignored while not active.
// In: scan (LaserScan, in the base frame), map (OccupancyGrid, latched), initialpose (PoseWithCovarianceStamped; its covariance is the prior), TF odom->base.
// Out: TF map->odom (scan stamp + tf_tolerance), amcl_pose (so listeners of AMCL's pose keep working), gn_loc/status (String, 1 Hz),
//      gn_loc/need_reloc (Bool, latched, on change): true after reloc_after_s in LOST, for whoever starts the global relocalizer; its /initialpose ends LOST.
// Scans wait in a tf2_ros::MessageFilter until odom->base exists at their stamp (as in nav2_amcl); nothing is dropped for being ahead of the odometry.
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>

#include "bondcpp/bond.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gn_loc/gn_core.hpp"
#include "message_filters/subscriber.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/time.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;
using sensor_msgs::msg::LaserScan;

namespace
{
double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}
}  // namespace

class GnLocNode : public rclcpp_lifecycle::LifecycleNode
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:
  GnLocNode()
  : rclcpp_lifecycle::LifecycleNode("gn_loc")
  {
    declare_parameter("base_frame", "base_footprint");
    declare_parameter("odom_frame", "odom");
    declare_parameter("global_frame", "map");
    declare_parameter("tf_tolerance", 0.2);
    declare_parameter("publish_pose", true);
    declare_parameter("sigma", 0.10);
    declare_parameter("dmax", 0.20);
    declare_parameter("robust_c", 0.08);
    declare_parameter("n_beams", 360);
    declare_parameter("q_pos", 0.02);
    declare_parameter("q_yaw_deg", 0.7);
    declare_parameter("min_inliers", 0.25);
    declare_parameter("lost_s", 3.0);
    declare_parameter("tight", 0.05);
    declare_parameter("min_tight", 0.50);
    declare_parameter("accept_tight", 0.60);
    declare_parameter("recover_period", 1.0);
    declare_parameter("recover_sxy0", 0.2);
    declare_parameter("recover_syaw0_deg", 5.0);
    declare_parameter("recover_grow_xy", 0.05);
    declare_parameter("recover_grow_yaw_deg", 1.5);
    declare_parameter("recover_sxy_max", 1.5);
    declare_parameter("recover_syaw_max_deg", 30.0);
    declare_parameter("recover_k", 10);
    declare_parameter("reloc_after_s", 15.0);
    declare_parameter("init_search", true);
    declare_parameter("still_window_s", 1.0);   // map->odom random walk only while the odometry moves (gn_core.hpp Params)
    declare_parameter("still_xy", 0.05);
    declare_parameter("still_yaw_deg", 1.0);
    declare_parameter("scan_topic", "scan");
    declare_parameter("bond_heartbeat_period", 0.1);   // s; 0 = no bond (as nav2_util::LifecycleNode)
  }

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    base_ = get_parameter("base_frame").as_string();
    odom_ = get_parameter("odom_frame").as_string();
    global_ = get_parameter("global_frame").as_string();
    tol_ = get_parameter("tf_tolerance").as_double();
    publish_pose_ = get_parameter("publish_pose").as_bool();
    prm_ = gn_loc::Params{};
    prm_.sigma = get_parameter("sigma").as_double();
    prm_.dmax = get_parameter("dmax").as_double();
    prm_.robust_c = get_parameter("robust_c").as_double();
    prm_.n_beams = int(get_parameter("n_beams").as_int());
    prm_.q_pos = get_parameter("q_pos").as_double();
    prm_.q_yaw = get_parameter("q_yaw_deg").as_double() * M_PI / 180.0;
    prm_.min_inliers = get_parameter("min_inliers").as_double();
    prm_.lost_s = get_parameter("lost_s").as_double();
    const double deg = M_PI / 180.0;
    prm_.tight = get_parameter("tight").as_double();
    prm_.min_tight = get_parameter("min_tight").as_double();
    prm_.accept_tight = get_parameter("accept_tight").as_double();
    prm_.recover_period = get_parameter("recover_period").as_double();
    prm_.recover_sxy0 = get_parameter("recover_sxy0").as_double();
    prm_.recover_syaw0 = get_parameter("recover_syaw0_deg").as_double() * deg;
    prm_.recover_grow_xy = get_parameter("recover_grow_xy").as_double();
    prm_.recover_grow_yaw = get_parameter("recover_grow_yaw_deg").as_double() * deg;
    prm_.recover_sxy_max = get_parameter("recover_sxy_max").as_double();
    prm_.recover_syaw_max = get_parameter("recover_syaw_max_deg").as_double() * deg;
    prm_.recover_k = int(get_parameter("recover_k").as_int());
    prm_.reloc_after_s = get_parameter("reloc_after_s").as_double();
    prm_.init_search = get_parameter("init_search").as_bool();
    prm_.still_window_s = get_parameter("still_window_s").as_double();
    prm_.still_xy = get_parameter("still_xy").as_double();
    prm_.still_yaw = get_parameter("still_yaw_deg").as_double() * deg;
    const auto scan_topic = get_parameter("scan_topic").as_string();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_buffer_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(), get_node_timers_interface()));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    br_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", rclcpp::QoS(1).transient_local().reliable());
    status_pub_ = create_publisher<std_msgs::msg::String>("gn_loc/status", 1);
    reloc_pub_ = create_publisher<std_msgs::msg::Bool>("gn_loc/need_reloc", rclcpp::QoS(1).transient_local().reliable());
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local().reliable(), [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {onMap(*m);});
    init_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 10, [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) {onInit(*m);});
    scan_sub_ = std::make_unique<message_filters::Subscriber<LaserScan, rclcpp_lifecycle::LifecycleNode>>(
      shared_from_this(), scan_topic, rmw_qos_profile_sensor_data);
    scan_filter_ = std::make_unique<tf2_ros::MessageFilter<LaserScan>>(
      *scan_sub_, *tf_buffer_, odom_, 10, get_node_logging_interface(), get_node_clock_interface(), tf2::durationFromSec(1.0));
    scan_filter_->registerCallback(std::bind(&GnLocNode::onScan, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "configured: waiting for map and initialpose");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    pose_pub_->on_activate();
    status_pub_->on_activate();
    reloc_pub_->on_activate();
    publishReloc(false);
    timer_ = create_wall_timer(1s, [this]() {status();});
    active_ = true;
    createBond();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    active_ = false;
    bond_.reset();
    timer_.reset();
    pose_pub_->on_deactivate();
    status_pub_->on_deactivate();
    reloc_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    scan_filter_.reset(); scan_sub_.reset(); init_sub_.reset(); map_sub_.reset();
    pose_pub_.reset(); status_pub_.reset(); reloc_pub_.reset(); br_.reset(); tf_listener_.reset(); tf_buffer_.reset();
    std::lock_guard<std::mutex> lk(mtx_);
    core_.reset(); pending_init_.reset(); ratio_ = tight_ = 0; n_ = 0;
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & s) override {return on_cleanup(s);}

private:
  void createBond()   // the lifecycle_manager of Nav2 holds a bond with every managed node and takes the node as dead without it
  {
    const double period = get_parameter("bond_heartbeat_period").as_double();
    if (period <= 0.0) {return;}
    bond_ = std::make_unique<bond::Bond>("bond", get_name(), shared_from_this());
    bond_->setHeartbeatPeriod(period);
    bond_->setHeartbeatTimeout(4.0);
    bond_->start();
  }

  void onMap(const nav_msgs::msg::OccupancyGrid & m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    const auto & i = m.info;
    auto field = std::make_shared<const gn_loc::Field>(m.data, int(i.width), int(i.height), i.resolution, i.origin.position.x, i.origin.position.y);
    core_ = std::make_unique<gn_loc::Core>(field, prm_);
    RCLCPP_INFO(get_logger(), "map %ux%u at %.3f m: distance field built", i.width, i.height, i.resolution);
    if (pending_init_) {const auto p = *pending_init_; pending_init_.reset(); setInitial(p);}
  }

  void onInit(const geometry_msgs::msg::PoseWithCovarianceStamped & m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!core_) {pending_init_ = m; return;}
    setInitial(m);
  }

  void setInitial(const geometry_msgs::msg::PoseWithCovarianceStamped & m)   // mtx_ held
  {
    const double yaw = yawOf(m.pose.pose.orientation);
    if (!m.header.frame_id.empty() && m.header.frame_id != global_) {   // nav2_amcl transforms it; here it is used as it comes
      RCLCPP_WARN(get_logger(), "initial pose given in frame '%s', not '%s': used as a '%s' pose", m.header.frame_id.c_str(), global_.c_str(), global_.c_str());
    }
    const auto & c = m.pose.covariance;
    core_->setInitial({m.pose.pose.position.x, m.pose.pose.position.y, yaw}, std::sqrt(std::max(c[0], c[7])), std::sqrt(c[35]));
    RCLCPP_INFO(get_logger(), "initial pose %.3f %.3f %.1f deg", m.pose.pose.position.x, m.pose.pose.position.y, yaw * 180 / M_PI);
  }

  void onScan(const LaserScan::ConstSharedPtr & m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!active_ || !core_ || core_->state() == gn_loc::State::WaitingPose) {return;}
    if (m->header.frame_id != base_) {    // the beams are fitted as seen from the base frame (odom->base is what is looked up)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "scan frame '%s' is not the base frame '%s': the fit assumes they are the same", m->header.frame_id.c_str(), base_.c_str());
    }
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(odom_, base_, m->header.stamp);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "no odom->base at the scan stamp: %s", e.what());
      return;
    }
    const gn_loc::Pose o{tf.transform.translation.x, tf.transform.translation.y, yawOf(tf.transform.rotation)};
    const double t = rclcpp::Time(m->header.stamp).seconds();
    const auto r = core_->update(t, o, m->ranges, m->angle_min, m->angle_increment);
    ratio_ = r.ratio; tight_ = r.tight;
    if (r.need_reloc != reloc_sent_) {publishReloc(r.need_reloc);}
    if (!r.Tmo) {return;}
    geometry_msgs::msg::TransformStamped out;
    out.header.stamp = (rclcpp::Time(m->header.stamp) + rclcpp::Duration::from_seconds(tol_));
    out.header.frame_id = global_; out.child_frame_id = odom_;
    out.transform.translation.x = r.Tmo->x; out.transform.translation.y = r.Tmo->y;
    out.transform.rotation.z = std::sin(r.Tmo->yaw / 2); out.transform.rotation.w = std::cos(r.Tmo->yaw / 2);
    br_->sendTransform(out);
    if (publish_pose_) {
      const gn_loc::Pose p = gn_loc::compose(*r.Tmo, o);
      geometry_msgs::msg::PoseWithCovarianceStamped pm;
      pm.header.stamp = m->header.stamp; pm.header.frame_id = global_;
      pm.pose.pose.position.x = p.x; pm.pose.pose.position.y = p.y;
      pm.pose.pose.orientation.z = std::sin(p.yaw / 2); pm.pose.pose.orientation.w = std::cos(p.yaw / 2);
      auto & c = pm.pose.covariance;
      c[0] = r.P(0, 0); c[1] = r.P(0, 1); c[6] = r.P(1, 0); c[7] = r.P(1, 1); c[5] = r.P(0, 2); c[30] = r.P(2, 0); c[35] = r.P(2, 2);
      pose_pub_->publish(pm);
    }
    ++n_;
  }

  void publishReloc(bool v)
  {
    reloc_sent_ = v;
    std_msgs::msg::Bool m; m.data = v;
    reloc_pub_->publish(m);
    if (v) {RCLCPP_WARN(get_logger(), "LOST for %.0f s: asking for the global relocalizer (gn_loc/need_reloc)", prm_.reloc_after_s);}
  }

  void status()
  {
    // the bond heartbeat shares this executor: never wait for a scan being processed (the initial search can take a while), skip the tick instead
    std::unique_lock<std::mutex> lk(mtx_, std::try_to_lock);
    if (!lk.owns_lock() || !status_pub_ || !status_pub_->is_activated()) {return;}
    std_msgs::msg::String s;
    char buf[128];
    std::snprintf(buf, sizeof buf, "%s inliers %.2f tight %.2f scans %d", core_ ? gn_loc::name(core_->state()) : "WAITING_MAP", ratio_, tight_, n_);
    s.data = buf;
    status_pub_->publish(s);
  }

  std::string base_, odom_, global_;
  double tol_, ratio_ = 0, tight_ = 0;
  bool reloc_sent_ = false;
  bool publish_pose_;
  int n_ = 0;
  std::atomic<bool> active_{false};
  std::unique_ptr<bond::Bond> bond_;
  gn_loc::Params prm_;
  std::mutex mtx_;   // the TransformListener thread delivers MessageFilter callbacks, so scan, map and initialpose can overlap
  std::unique_ptr<gn_loc::Core> core_;
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> pending_init_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_sub_;
  std::unique_ptr<message_filters::Subscriber<LaserScan, rclcpp_lifecycle::LifecycleNode>> scan_sub_;
  std::unique_ptr<tf2_ros::MessageFilter<LaserScan>> scan_filter_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr reloc_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<GnLocNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
