// /imu_raw (Gazebo's IMU) -> the ICM-40609-D noise measured by a 12 h Allan variance (models.hpp ImuModel) -> /imu/data. Static noise = lower bound: the vibration of a moving robot is not modelled.
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "socialtech_simreal/models.hpp"

class SimImuModel : public rclcpp::Node
{
public:
  SimImuModel()
  : rclcpp::Node("sim_imu_model")
  {
    simreal::ImuParams p;
    const int seed = declare_parameter("seed", -1);
    dt_ = 1.0 / declare_parameter("rate_hz", 100.0);
    model_ = std::make_unique<simreal::ImuModel>(p, seed < 0 ? -1 : seed + 4);
    sub_ = create_subscription<sensor_msgs::msg::Imu>(declare_parameter("in_topic", "/imu_raw"), 10, [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
        auto o = *m; double g[3] = {o.angular_velocity.x, o.angular_velocity.y, o.angular_velocity.z}, a[3] = {o.linear_acceleration.x, o.linear_acceleration.y, o.linear_acceleration.z};
        model_->step(g, a, dt_);
        o.angular_velocity.x = g[0]; o.angular_velocity.y = g[1]; o.angular_velocity.z = g[2]; o.linear_acceleration.x = a[0]; o.linear_acceleration.y = a[1]; o.linear_acceleration.z = a[2];
        pub_->publish(o);
      });
    pub_ = create_publisher<sensor_msgs::msg::Imu>(declare_parameter("out_topic", "/imu/data"), 10);
  }

private:
  double dt_;
  std::unique_ptr<simreal::ImuModel> model_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimImuModel>());
  rclcpp::shutdown();
  return 0;
}
