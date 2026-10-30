// /scan_raw (true ranges of Gazebo's 2D lidar) -> the noise of the real /scan (models.hpp ScanModel, calibrated on the Mid-360 + pointcloud_to_laserscan chain) -> /scan
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "socialtech_simreal/models.hpp"

class SimScanModel : public rclcpp::Node
{
public:
  SimScanModel()
  : rclcpp::Node("sim_scan_model")
  {
    simreal::ScanParams p;
    const int seed = declare_parameter("seed", -1);
    p.outlier_scale = declare_parameter("outlier_scale", p.outlier_scale);   // 0: no surface jumps
    p.noise_scale = declare_parameter("noise_scale", p.noise_scale);
    model_ = std::make_unique<simreal::ScanModel>(p, seed < 0 ? -1 : seed + 3);
    RCLCPP_INFO(get_logger(), "scan model: gaussian core %.0f-%.0f mm, dropout %.0f-%.0f %% by range, surface jumps x%.2f, noise x%.2f", p.core_sigma.at(2.5) * 1e3, p.core_sigma.at(10) * 1e3, p.dropout.at(1.5) * 100, p.dropout.at(16) * 100, p.outlier_scale, p.noise_scale);
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(declare_parameter("in_topic", "/scan_raw"), 10, [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {
        auto o = *m; model_->apply(o.ranges, o.range_min, o.range_max); pub_->publish(o);
      });
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(declare_parameter("out_topic", "/scan"), 10);
  }

private:
  std::unique_ptr<simreal::ScanModel> model_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimScanModel>());
  rclcpp::shutdown();
  return 0;
}
