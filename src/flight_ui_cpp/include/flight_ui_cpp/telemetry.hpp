#pragma once

#include <mutex>
#include <vector>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include "flight_ui_cpp/types.hpp"

namespace flight_ui_cpp
{

// 订阅 PX4 的遥测话题，聚合成一个可以随时加锁读取的快照
class Telemetry
{
public:
  explicit Telemetry(rclcpp::Node * node);

  TelemetrySnapshot snapshot() const;

private:
  void onStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void onPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void onBattery(const px4_msgs::msg::BatteryStatus::SharedPtr msg);
  void onLanded(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);

  mutable std::mutex mutex_;
  TelemetrySnapshot state_;
  // 同上：订阅对象必须保活
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
};

}  // namespace flight_ui_cpp
