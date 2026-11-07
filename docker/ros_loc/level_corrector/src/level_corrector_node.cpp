// level_corrector, shadow mode: every 1/rate_hz s it registers the latest FAST-LIO body cloud against the levelled 3D map, seeded with gn_loc's map->odom and FAST-LIO's pose,
// and publishes the roll/pitch correction C that odom <- camera_init would need (L' = C * L, L = the static level planar_odom publishes). It publishes only: no TF, planar_odom does
// not read it. Out: ~/correction (QuaternionStamped, C), ~/status (String, every tick: state, fit, measured tilt, C, CPU ms). When a gate fails C stays where it was.
#include <time.h>

#include <deque>
#include <memory>

#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "level_corrector/map_io.hpp"
#include "level_corrector/registration.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace level_corrector;

static double cpu_s() { timespec t; clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t); return t.tv_sec + 1e-9 * t.tv_nsec; }
static double secs(const builtin_interfaces::msg::Time& t) { return t.sec + 1e-9 * t.nanosec; }
static Eigen::Isometry3d iso(const geometry_msgs::msg::Pose& p) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z).normalized().toRotationMatrix();
  T.translation() = Vec3(p.position.x, p.position.y, p.position.z);
  return T;
}

class LevelCorrector : public rclcpp::Node {
 public:
  LevelCorrector()
      : Node("level_corrector"), gates_(), filter_(declare_parameter("tau_s", 10.0), declare_parameter("slew_deg_s", 0.05), declare_parameter("max_jump_deg", 2.0), static_cast<int>(declare_parameter("jump_limit", 15))),
        tf_buf_(get_clock()), tf_(tf_buf_) {
    const auto dir = declare_parameter<std::string>("map_dir", "/map");
    const auto lv = declare_parameter("level", std::vector<double>{0.0, 0.0, 0.0, 1.0});
    if (lv.size() != 4) throw std::runtime_error("level must be [qx, qy, qz, qw]");
    L_ = Eigen::Quaterniond(lv[3], lv[0], lv[1], lv[2]).normalized().toRotationMatrix();
    gates_.min_inliers = declare_parameter("min_inliers", gates_.min_inliers);
    gates_.min_ratio = declare_parameter("min_ratio", gates_.min_ratio);
    gates_.max_rms = declare_parameter("max_rms", gates_.max_rms);
    gates_.max_dxy = declare_parameter("max_dxy", gates_.max_dxy);
    gates_.max_dyaw_deg = declare_parameter("max_dyaw_deg", gates_.max_dyaw_deg);
    const double rate = declare_parameter("rate_hz", 0.5);
    const double t0 = cpu_s();
    reg_ = std::make_unique<MapRegistrar>(read_ply_xyz(dir + "/map.ply", read_t_map_world(dir + "/map.yaml")), declare_parameter("map_voxel", 0.1), declare_parameter("scan_voxel", 0.25));
    RCLCPP_INFO(get_logger(), "map %zu voxels, prepared in %.2f s CPU; rate %.2f Hz", reg_->map_size(), cpu_s() - t0, rate);
    check_only_ = declare_parameter("check_only", false);   // nav_live.sh: load the map with the real loader and leave (a bad map throws above, before this line)
    if (check_only_) return;
    pub_c_ = create_publisher<geometry_msgs::msg::QuaternionStamped>("~/correction", 1);
    pub_s_ = create_publisher<std_msgs::msg::String>("~/status", 10);
    sub_o_ = create_subscription<nav_msgs::msg::Odometry>("/Odometry", 100, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { on_odom(*m); });
    sub_c_ = create_subscription<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", rclcpp::SensorDataQoS().keep_last(1), [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { cloud_ = m; });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] { tick(); });
  }

 public:
  bool check_only() const { return check_only_; }

 private:
  void on_odom(const nav_msgs::msg::Odometry& m) {
    const double t = secs(m.header.stamp);
    const Vec3 p(m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z);
    if (!odom_.empty() && restarted(odom_.back().t, odom_.back().p, t, p)) {   // FAST-LIO restarted: new camera_init
      restart_ = true; odom_.clear();
      filter_.reset(); publish_c(now());   // the old C belonged to the old camera_init: plain level until the first valid measurement of the new one
    }
    odom_.push_back({t, p, iso(m.pose.pose).linear()});
    if (odom_.size() > 50) odom_.pop_front();
  }

  void publish_c(const builtin_interfaces::msg::Time& stamp) {
    geometry_msgs::msg::QuaternionStamped q; q.header.stamp = stamp; q.header.frame_id = "odom";
    const Eigen::Quaterniond Q(filter_.C()); q.quaternion.x = Q.x(); q.quaternion.y = Q.y(); q.quaternion.z = Q.z(); q.quaternion.w = Q.w();
    pub_c_->publish(q);
  }

  void say(const std::string& s) { std_msgs::msg::String m; m.data = s; pub_s_->publish(m); }

  // Every cycle, whether it could measure or not, C goes out again: it is the heartbeat planar_odom needs to keep applying it (it drops a C that goes silent: the corrector may be gone).
  void tick() {
    cycle();
    if (filter_.primed()) publish_c(now());
  }

  void cycle() {
    const auto cloud = std::move(cloud_); cloud_.reset();
    if (!cloud) return say("state=no_cloud");
    if (odom_.empty()) return say("state=no_odom");
    const double ts = secs(cloud->header.stamp), c0 = cpu_s();
    const Odo* o = &odom_.back();
    for (const auto& x : odom_) if (std::abs(x.t - ts) < std::abs(o->t - ts)) o = &x;
    if (std::abs(o->t - ts) > 0.15) return say("state=odom_far");
    Eigen::Isometry3d T_mo;
    try { T_mo = tf2::transformToEigen(tf_buf_.lookupTransform("map", "odom", tf2::TimePointZero)); } catch (const tf2::TransformException& e) { return say("state=no_map_odom"); }

    std::vector<Eigen::Vector3f> pts;
    pts.reserve(cloud->width * cloud->height);
    for (sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x"), y(*cloud, "y"), z(*cloud, "z"); x != x.end(); ++x, ++y, ++z) {
      const Eigen::Vector3f q(*x, *y, *z);
      if (q.allFinite() && q.norm() > 0.8f) pts.push_back(q);
    }
    const Aligned a = reg_->align(pts, seed_pose(T_mo, filter_.C(), L_, o->R, o->p));
    const char* bad = gate_reason(a.fit, gates_);
    const Vec3 up = a.T.linear().transpose() * Vec3::UnitZ();                       // the map's up in the body
    const Mat3 Cm = tilt_rot(L_ * o->R * up, Vec3::UnitZ());
    const char* st = bad ? bad : "ok";
    if (!bad) {
      const auto r = filter_.update(Cm, ts);
      if (r == Filter::Jump) st = "jump"; else restart_ = false;
    }
    auto xy = [](const Mat3& R) -> Eigen::Vector2d { const Vec3 u = R.transpose() * Vec3::UnitZ(); return Eigen::Vector2d(std::atan2(u.x(), u.z()), std::atan2(u.y(), u.z())) * 180 / M_PI; };
    const Eigen::Vector2d m = xy(Cm.transpose()), c = xy(filter_.C().transpose());
    char b[400];
    std::snprintf(b, sizeof b, "state=%s t=%.3f n=%d src=%d rms=%.4f dxy=%.3f dyaw=%.2f conv=%d meas_pitch=%+.3f meas_roll=%+.3f C_pitch=%+.3f C_roll=%+.3f primed=%d restart=%d npts=%zu cpu_ms=%.1f",
                  st, ts, a.fit.inliers, a.fit.source_points, a.fit.rms, a.fit.dxy, a.fit.dyaw_deg, a.converged, m.x(), m.y(), c.x(), c.y(), filter_.primed(), restart_, pts.size(), 1e3 * (cpu_s() - c0));
    say(b);
  }

  struct Odo { double t; Vec3 p; Mat3 R; };
  Gates gates_;
  Filter filter_;
  Mat3 L_;
  std::unique_ptr<MapRegistrar> reg_;
  tf2_ros::Buffer tf_buf_;
  tf2_ros::TransformListener tf_;
  std::deque<Odo> odom_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_;
  bool restart_ = false, check_only_ = false;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr pub_c_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_s_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_o_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_c_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<LevelCorrector>();
  if (!node->check_only()) rclcpp::spin(node);
  rclcpp::shutdown();
}
