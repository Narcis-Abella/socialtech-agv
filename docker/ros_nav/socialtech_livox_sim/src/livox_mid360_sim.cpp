// Simulated Livox Mid-360: every point of the closed-form scan model (per-unit constants) is ray cast from the sensor pose at the point's own time,
#include <algorithm>
// so the output carries the real motion distortion and per-point offset_time. No-return slots are emitted as zero points (as the driver does).
#include <chrono>
#include <cstring>
#include <random>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "socialtech_livox_sim/caster.hpp"

using livox_ros_driver2::msg::CustomMsg;
using livox_ros_driver2::msg::CustomPoint;
using sensor_msgs::msg::PointCloud2;

class LivoxSim : public rclcpp::Node
{
public:
  LivoxSim()
  : Node("livox_mid360_sim"), mod_(livoxsim::load_model(declare_parameter<std::string>("model_file", ""))),
    scene_(declare_parameter<std::string>("scene_file", ""))
  {
    frame_id_ = declare_parameter("frame_id", std::string("livox_mid360_link"));
    sensor_ = declare_parameter("sensor_xyz", std::vector<double>{0.0, 0.0, 0.28});     // pose frame (base_footprint) -> sensor
    pose_off_ = declare_parameter("pose_offset", std::vector<double>{0.0, 0.0, 0.0});    // world <- pose source frame (x, y, yaw)
    auto static_pose = declare_parameter("static_pose", std::vector<double>{});
    rmin_ = declare_parameter("range_min", 0.1); rmax_ = declare_parameter("range_max", 40.0);
    sigma_ = declare_parameter("range_sigma", 0.02); refl_ = declare_parameter("reflectivity", 100);
    rng_.seed(declare_parameter("seed", 1));
    jitter_ = declare_parameter("jitter_std", 0.0);                                         // s; reported packet time vs firing (real driver ~0.0005, AR(1) 0.65), off by default
    const auto custom_topic = declare_parameter("custom_topic", std::string("/livox/lidar")), pc2_topic = declare_parameter("pc2_topic", std::string(""));
    if (!custom_topic.empty()) {custom_pub_ = create_publisher<CustomMsg>(custom_topic, 10);}
    if (!pc2_topic.empty()) {pc2_pub_ = create_publisher<PointCloud2>(pc2_topic, 10);}
    if (static_pose.size() == 3) {static_ = static_pose;} else {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(declare_parameter("pose_topic", std::string("/odom_gt_raw")), 50, [this](nav_msgs::msg::Odometry::SharedPtr m) {on_odom(*m);});
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]() {tick();});          // wall timer + node clock reads: also works with use_sim_time
    RCLCPP_INFO(get_logger(), "model: f_z %.4f Hz, f_a %.4f Hz; scene: %zu triangles", mod_.f_z, mod_.f_a, scene_.triangles());
  }

private:
  void on_odom(const nav_msgs::msg::Odometry & m)
  {
    const auto & q = m.pose.pose.orientation;
    const auto & p = m.pose.pose.position;
    const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), c = std::cos(pose_off_[2]), s = std::sin(pose_off_[2]);
    track_.push({rclcpp::Time(m.header.stamp).seconds(), pose_off_[0] + c * p.x - s * p.y, pose_off_[1] + s * p.x + c * p.y, yaw + pose_off_[2]});
  }

  // The point stream is continuous (slot clock); a frame is the whole packets that start inside its 100 ms, like the driver (208 or 209 packets = 19968 / 20064 points)
  static uint64_t first_packet(uint64_t n) {return (625 * n + 2) / 3;}        // ceil(n * 0.1 s / 480 us)

  void tick()
  {
    if (!static_.empty()) {track_.push({now().seconds(), static_[0], static_[1], static_[2]});}
    if (track_.empty()) {return;}
    if (t_stream_ < 0) {t_stream_ = track_.newest();}
    while (t_stream_ + double(first_packet(frame_ + 1)) * livoxsim::kPacket * livoxsim::kSlot <= track_.newest()) {emit(frame_++);}
  }

  void emit(uint64_t n)
  {
    const auto c0 = std::chrono::steady_clock::now();
    const uint64_t p0 = first_packet(n), p1 = first_packet(n + 1), k0 = p0 * livoxsim::kPacket, np = (p1 - p0) * livoxsim::kPacket;
    const double t0 = t_stream_ + double(k0) * livoxsim::kSlot;
    std::normal_distribution<float> noise(0.0f, float(sigma_)), innov(0.0f, 1.0f);
    CustomMsg msg; msg.header.frame_id = frame_id_; msg.header.stamp = rclcpp::Time(int64_t(t0 * 1e9)); msg.timebase = uint64_t(t0 * 1e9); msg.lidar_id = 0; msg.points.resize(np);
    const float sx = float(sensor_[0]), sy = float(sensor_[1]), sz = float(sensor_[2]); size_t nvalid = 0; double jit = 0;
    for (uint64_t i = 0; i < np; ++i) {
      if (i % livoxsim::kPacket == 0) {jit_ = 0.65 * jit_ + 0.76 * jitter_ * innov(rng_); jit = jit_;}   // one reported-time error per packet; sqrt(1 - 0.65^2) = 0.76
      const double off = double(i) * livoxsim::kSlot;
      const livoxsim::Pose p = track_.at(t0 + off);
      const float c = float(std::cos(p.yaw)), s = float(std::sin(p.yaw));
      double az, ze; mod_.direction(k0 + i, az, ze);
      const float sze = float(std::sin(ze)), ds[3] = {sze * float(std::cos(az)), sze * float(std::sin(az)), float(std::cos(ze))};
      const float o[3] = {float(p.x) + c * sx - s * sy, float(p.y) + s * sx + c * sy, sz}, d[3] = {c * ds[0] - s * ds[1], s * ds[0] + c * ds[1], ds[2]};
      float r = scene_.cast(o, d, float(rmin_), float(rmax_));
      CustomPoint & q = msg.points[i];
      q.offset_time = uint32_t(std::max(0.0, off + jit) * 1e9); q.line = uint8_t((k0 + i) & 3); q.tag = 0;
      if (r >= 0 && (r += noise(rng_)) >= float(rmin_)) {q.x = r * ds[0]; q.y = r * ds[1]; q.z = r * ds[2]; q.reflectivity = uint8_t(refl_); ++nvalid;}
      else {q.x = q.y = q.z = 0.0f; q.reflectivity = 0;}
    }
    msg.point_num = uint32_t(msg.points.size());
    if (pc2_pub_) {pc2_pub_->publish(to_pc2(msg));}
    if (custom_pub_) {custom_pub_->publish(msg);}
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - c0).count();
    sum_ms_ += ms; max_ms_ = std::max(max_ms_, ms); valid_ += nvalid; total_ += np;
    if (++n_stat_ == 100) {
      RCLCPP_INFO(get_logger(), "100 frames: %.2f ms/frame (max %.2f), %.2f Mrays/s, valid %.0f %%", sum_ms_ / 100, max_ms_, double(total_) / (sum_ms_ * 1e3), 100.0 * double(valid_) / double(total_));
      sum_ms_ = max_ms_ = 0; valid_ = total_ = 0; n_stat_ = 0;
    }
  }

  // Driver layout of xfer_format 0 (PointXYZRTLT, 26 bytes): x y z float32, intensity float32, tag u8, line u8, timestamp float64 [ns, absolute]
  PointCloud2 to_pc2(const CustomMsg & m) const
  {
    PointCloud2 pc; pc.header = m.header; pc.height = 1; pc.width = m.point_num; pc.is_bigendian = false; pc.is_dense = true; pc.point_step = 26; pc.row_step = pc.point_step * pc.width;
    auto field = [&pc](const char * n, uint32_t off, uint8_t type) {sensor_msgs::msg::PointField f; f.name = n; f.offset = off; f.datatype = type; f.count = 1; pc.fields.push_back(f);};
    field("x", 0, 7); field("y", 4, 7); field("z", 8, 7); field("intensity", 12, 7); field("tag", 16, 2); field("line", 17, 2); field("timestamp", 18, 8);
    pc.data.resize(size_t(pc.row_step));
    for (size_t i = 0; i < m.points.size(); ++i) {
      uint8_t * w = pc.data.data() + 26 * i; const CustomPoint & q = m.points[i];
      const float in = q.reflectivity; const double ts = double(m.timebase) + double(q.offset_time);
      std::memcpy(w, &q.x, 4); std::memcpy(w + 4, &q.y, 4); std::memcpy(w + 8, &q.z, 4); std::memcpy(w + 12, &in, 4); w[16] = q.tag; w[17] = q.line; std::memcpy(w + 18, &ts, 8);
    }
    return pc;
  }

  livoxsim::Model mod_; livoxsim::Scene scene_; livoxsim::PoseTrack track_;
  std::string frame_id_; std::vector<double> sensor_, pose_off_, static_; double rmin_, rmax_, sigma_; int refl_;
  std::mt19937_64 rng_; double t_stream_ = -1, jitter_ = 0, jit_ = 0; uint64_t frame_ = 0;
  double sum_ms_ = 0, max_ms_ = 0; uint64_t valid_ = 0, total_ = 0; int n_stat_ = 0;
  rclcpp::Publisher<CustomMsg>::SharedPtr custom_pub_; rclcpp::Publisher<PointCloud2>::SharedPtr pc2_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxSim>());
  rclcpp::shutdown();
  return 0;
}
