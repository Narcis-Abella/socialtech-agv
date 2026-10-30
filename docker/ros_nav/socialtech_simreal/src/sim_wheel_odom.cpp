// What the base's wheel odometry reports: the true twist of Gazebo (/odom_gt_raw) -> scale error, quantisation, noise and delay (models.hpp WheelOdomModel) -> /odom, with the pose integrated from the reported twist.
#include <cmath>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "socialtech_simreal/models.hpp"

class SimWheelOdom : public rclcpp::Node
{
public:
  SimWheelOdom()
  : rclcpp::Node("sim_wheel_odom")
  {
    simreal::WheelParams p;
    const int seed = declare_parameter("seed", -1);
    p.dt = declare_parameter("input_dt", p.dt);
    p.delay = declare_parameter("delay", p.delay);
    simreal::Rng rng(seed < 0 ? -1 : seed + 1);
    p.draw_scales(rng);   // one scale error per run
    p.k_v = declare_parameter("k_v", p.k_v); p.k_w_moving = declare_parameter("k_w_moving", p.k_w_moving); p.k_w_inplace = declare_parameter("k_w_inplace", p.k_w_inplace);
    dt_ = p.dt;
    model_ = std::make_unique<simreal::WheelOdomModel>(p, seed < 0 ? -1 : seed + 2);
    RCLCPP_INFO(get_logger(), "wheel odometry: reported/actual linear %.3f, angular %.3f moving / %.3f in place; quantisation %.3f m/s, %.3f rad/s", p.k_v, p.k_w_moving, p.k_w_inplace, p.q_v, p.q_w);
    pose_cov_.fill(0.0); twist_cov_.fill(0.0);
    pose_cov_[0] = pose_cov_[7] = 0.005; pose_cov_[14] = pose_cov_[21] = pose_cov_[28] = 1e6; pose_cov_[35] = 0.08;
    twist_cov_[0] = 0.001; twist_cov_[7] = 0.0001; twist_cov_[14] = twist_cov_[21] = twist_cov_[28] = 1e6; twist_cov_[35] = 0.05;
    sub_ = create_subscription<nav_msgs::msg::Odometry>(declare_parameter("in_topic", "/odom_gt_raw"), 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {onTruth(*m);});
    pub_ = create_publisher<nav_msgs::msg::Odometry>(declare_parameter("out_topic", "/odom"), 10);
  }

private:
  void onTruth(const nav_msgs::msg::Odometry & m)
  {
    const auto [v, w] = model_->step(m.twist.twist.linear.x, m.twist.twist.angular.z);
    th_ += w * dt_; x_ += v * std::cos(th_) * dt_; y_ += v * std::sin(th_) * dt_;
    nav_msgs::msg::Odometry o;
    o.header = m.header; o.header.frame_id = "odom"; o.child_frame_id = "base_footprint";
    o.pose.pose.position.x = x_; o.pose.pose.position.y = y_; o.pose.pose.orientation.z = std::sin(th_ / 2); o.pose.pose.orientation.w = std::cos(th_ / 2);
    o.twist.twist.linear.x = v; o.twist.twist.angular.z = w; o.pose.covariance = pose_cov_; o.twist.covariance = twist_cov_;
    pub_->publish(o);
  }
  double dt_, x_ = 0, y_ = 0, th_ = 0;
  std::array<double, 36> pose_cov_, twist_cov_;
  std::unique_ptr<simreal::WheelOdomModel> model_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimWheelOdom>());
  rclcpp::shutdown();
  return 0;
}
