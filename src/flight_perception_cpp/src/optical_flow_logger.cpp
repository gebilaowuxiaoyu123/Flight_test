// 把 PX4 的光流/测距/EKF 速度写成 CSV，用于验证 v ≈ ω·h。

#include <chrono>
#include <cmath>
#include <fstream>
#include <mutex>
#include <string>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_optical_flow.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
class OpticalFlowLogger : public rclcpp::Node
{
public:
  OpticalFlowLogger()
  : rclcpp::Node("optical_flow_logger")
  {
    path_ = declare_parameter<std::string>("csv_path", "/tmp/optical_flow_cpp.csv");
    file_.open(path_, std::ios::out | std::ios::trunc);
    if (file_.is_open()) {
      file_ << "t,flow_x,flow_y,dt,distance,quality,ekf_vx,ekf_vy,flow_vx,flow_vy\n";
    } else {
      RCLCPP_ERROR(get_logger(), "cannot open %s", path_.c_str());
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    flow_sub_ = create_subscription<px4_msgs::msg::VehicleOpticalFlow>(
      "/fmu/out/vehicle_optical_flow", qos,
      [this](px4_msgs::msg::VehicleOpticalFlow::SharedPtr msg) {onFlow(msg);});
    pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos,
      [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {onPosition(msg);});
    RCLCPP_INFO(get_logger(), "logging to %s", path_.c_str());
  }

  ~OpticalFlowLogger() override
  {
    if (file_.is_open()) {
      file_.flush();
    }
  }

private:
  void onPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    vx_ = msg->vx;
    vy_ = msg->vy;
  }

  void onFlow(const px4_msgs::msg::VehicleOpticalFlow::SharedPtr msg)
  {
    double vx = 0.0;
    double vy = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      vx = vx_;
      vy = vy_;
    }

    const double dt = msg->integration_timespan_us * 1e-6;
    const double flow_x = msg->pixel_flow[0];
    const double flow_y = msg->pixel_flow[1];
    const double distance = msg->distance_m;
    double flow_vx = 0.0;
    double flow_vy = 0.0;
    if (dt > 0.0 && std::isfinite(distance)) {
      flow_vx = flow_x / dt * distance;
      flow_vy = flow_y / dt * distance;
    }

    const double t = static_cast<double>(msg->timestamp) * 1e-6;
    std::lock_guard<std::mutex> lock(mutex_);
    if (file_.is_open()) {
      file_ << t << ',' << flow_x << ',' << flow_y << ',' << dt << ','
            << (std::isfinite(distance) ? distance : 0.0) << ','
            << static_cast<int>(msg->quality) << ','
            << vx << ',' << vy << ',' << flow_vx << ',' << flow_vy << '\n';
    }
  }

  std::string path_;
  std::ofstream file_;
  std::mutex mutex_;
  double vx_{0.0};
  double vy_{0.0};
  rclcpp::Subscription<px4_msgs::msg::VehicleOpticalFlow>::SharedPtr flow_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpticalFlowLogger>());
  rclcpp::shutdown();
  return 0;
}
