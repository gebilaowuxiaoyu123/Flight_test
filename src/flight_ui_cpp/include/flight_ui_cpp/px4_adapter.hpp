#pragma once

// PX4 消息字段只在这个文件里出现；PX4 改字段名时只改这里。

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

namespace flight_ui_cpp
{

class Px4Adapter
{
public:
  explicit Px4Adapter(rclcpp::Node * node);

  // PX4 校验 manual_control 时间戳用的是它自己的开机时间，
  // 所以这里跟随 /fmu/out/timesync_status 插值，而不是用 ROS 墙钟。
  uint64_t nowUs();

  void publishManual(double roll, double pitch, double yaw, double throttle, bool valid, bool deadman);

  void sendCommand(
    uint16_t command, float p1 = 0.0F, float p2 = 0.0F, float p3 = 0.0F, float p4 = 0.0F,
    double p5 = 0.0, double p6 = 0.0, float p7 = 0.0F);

  void arm(bool arm);
  void setMode(const std::string & mode);
  void takeoff(double altitude_m);
  void land();
  void kill();

private:
  void onTimesync(const px4_msgs::msg::TimesyncStatus::SharedPtr msg);
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

  rclcpp::Node * node_;
  rclcpp::Publisher<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;

  // 【C++ 关键点】rclcpp 不会替你保活 Subscription：不存成成员，
  // 临时对象析构时订阅就被撤销，回调永远不触发。rclpy 有内部引用，所以 Python 版没这个坑。
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;

  std::mutex mutex_;
  int64_t px4_time_us_{0};
  bool px4_time_valid_{false};
  std::chrono::steady_clock::time_point last_recv_{};
  bool reference_alt_valid_{false};
  double reference_alt_{0.0};
  double z_{0.0};
};

}  // namespace flight_ui_cpp
