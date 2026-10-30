// The real base between Nav2 and Gazebo: /cmd_vel (Nav2's output) -> dead time, lag, in-place deadband (models.hpp BaseModel) -> /cmd_vel_gz (the bridge's input to Gazebo's DiffDrive).
#include <chrono>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "socialtech_simreal/models.hpp"

class SimBaseModel : public rclcpp::Node
{
public:
  SimBaseModel()
  : rclcpp::Node("sim_base_model")
  {
    simreal::BaseParams p;
    const int seed = declare_parameter("seed", -1);
    p.td_v = declare_parameter("td_v", p.td_v); p.tau_v = declare_parameter("tau_v", p.tau_v); p.gain_v = declare_parameter("gain_v", p.gain_v);
    p.td_w = declare_parameter("td_w", p.td_w); p.tau_w = declare_parameter("tau_w", p.tau_w); p.gain_w = declare_parameter("gain_w", p.gain_w);
    p.kick = declare_parameter("kick", p.kick);
    const double db = declare_parameter("deadband", -1.0);   // < 0: one draw per run in [0.03, 0.18] rad/s (the three real sessions gave 0.06, 0.08 and 0.18)
    simreal::Rng rng(seed);
    if (db < 0) {p.draw_deadband(rng);} else {p.deadband = db;}
    timeout_ = declare_parameter("timeout", 1.0);
    model_ = std::make_unique<simreal::BaseModel>(p);
    RCLCPP_INFO(get_logger(), "base model: v dead time %.0f ms lag %.0f ms gain %.3f; w dead time %.0f ms lag %.0f ms gain %.3f, in-place deadband %.3f rad/s, kick %.3f", p.td_v * 1e3, p.tau_v * 1e3, p.gain_v, p.td_w * 1e3, p.tau_w * 1e3, p.gain_w, p.deadband, p.kick);
    sub_ = create_subscription<geometry_msgs::msg::Twist>(declare_parameter("in_topic", "/cmd_vel"), 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {v_ = m->linear.x; w_ = m->angular.z; last_ = now();});
    pub_ = create_publisher<geometry_msgs::msg::Twist>(declare_parameter("out_topic", "/cmd_vel_gz"), 10);
    timer_ = create_timer(std::chrono::milliseconds(10), [this]() {
        const bool fresh = last_.nanoseconds() != 0 && (now() - last_).seconds() < timeout_;
        const auto [v, w] = model_->step(fresh ? v_ : 0.0, fresh ? w_ : 0.0);
        geometry_msgs::msg::Twist o; o.linear.x = v; o.angular.z = w; pub_->publish(o);
      });
  }

private:
  double v_ = 0, w_ = 0, timeout_;
  rclcpp::Time last_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<simreal::BaseModel> model_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimBaseModel>());
  rclcpp::shutdown();
  return 0;
}
