#include "flight_ui_cpp/telemetry.hpp"

#include <unordered_map>

namespace flight_ui_cpp
{
namespace
{
const std::unordered_map<int, std::string> kNavStateNames{
  {0, "MANUAL"}, {1, "ALTCTL"}, {2, "POSCTL"}, {3, "AUTO_MISSION"},
  {4, "AUTO_LOITER"}, {5, "AUTO_RTL"}, {6, "POSITION_SLOW"}, {10, "ACRO"},
  {12, "DESCEND"}, {13, "TERMINATION"}, {14, "OFFBOARD"}, {15, "STABILIZED"},
  {17, "AUTO_TAKEOFF"}, {18, "AUTO_LAND"}, {19, "AUTO_FOLLOW"},
  {20, "AUTO_PRECLAND"}, {21, "ORBIT"}, {22, "AUTO_VTOL_TAKEOFF"},
};
}  // namespace

Telemetry::Telemetry(rclcpp::Node * node)
{
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  // PX4 在 MESSAGE_VERSION > 0 时给话题加版本后缀；v1.16 是 *_v1。
  // 同时订阅 v1 和 v0，跨版本都能用。
  subs_.push_back(node->create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status_v1", qos,
    [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) {onStatus(msg);}));
  subs_.push_back(node->create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status", qos,
    [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) {onStatus(msg);}));
  subs_.push_back(node->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/fmu/out/vehicle_local_position", qos,
    [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {onPosition(msg);}));
  subs_.push_back(node->create_subscription<px4_msgs::msg::BatteryStatus>(
    "/fmu/out/battery_status", qos,
    [this](px4_msgs::msg::BatteryStatus::SharedPtr msg) {onBattery(msg);}));
  subs_.push_back(node->create_subscription<px4_msgs::msg::VehicleLandDetected>(
    "/fmu/out/vehicle_land_detected", qos,
    [this](px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {onLanded(msg);}));
}

void Telemetry::onStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.armed = (msg->arming_state == 2);
  state_.nav_state = static_cast<int>(msg->nav_state);
  const auto it = kNavStateNames.find(state_.nav_state);
  state_.mode = (it != kNavStateNames.end()) ? it->second : "STATE_" + std::to_string(state_.nav_state);
}

void Telemetry::onPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.xy_valid = msg->xy_valid;
  state_.z_valid = msg->z_valid;
  state_.alt = msg->z_valid ? static_cast<double>(-msg->z) : 0.0;
  state_.vx = msg->vx;
  state_.vy = msg->vy;
  state_.vz = msg->vz;
}

void Telemetry::onBattery(const px4_msgs::msg::BatteryStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.voltage = msg->voltage_v;
  state_.battery = (msg->connected && msg->remaining >= 0.0F)
    ? static_cast<double>(msg->remaining * 100.0F) : -1.0;
}

void Telemetry::onLanded(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.landed = msg->landed;
}

TelemetrySnapshot Telemetry::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

}  // namespace flight_ui_cpp
