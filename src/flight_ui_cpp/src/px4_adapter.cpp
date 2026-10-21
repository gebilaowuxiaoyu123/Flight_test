#include "flight_ui_cpp/px4_adapter.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace flight_ui_cpp
{
namespace
{
constexpr char kManualControlTopic[] = "/fmu/in/manual_control_input";
constexpr char kVehicleCommandTopic[] = "/fmu/in/vehicle_command";

constexpr uint16_t kCmdNavLand = 21;
constexpr uint16_t kCmdNavTakeoff = 22;
constexpr uint16_t kCmdDoSetMode = 176;
constexpr uint16_t kCmdDoFlightTermination = 185;
constexpr uint16_t kCmdComponentArmDisarm = 400;

const std::unordered_map<std::string, uint8_t> kModeIds{
  {"MANUAL", 1}, {"ALTCTL", 2}, {"POSCTL", 3}, {"AUTO", 4},
  {"ACRO", 5}, {"OFFBOARD", 6}, {"STABILIZED", 7},
};

double clampUnit(double v)
{
  return std::max(-1.0, std::min(1.0, v));
}
}  // namespace

Px4Adapter::Px4Adapter(rclcpp::Node * node)
: node_(node)
{
  // /fmu/in 用 reliable，和 PX4 官方 uXRCE 示例保持一致
  const auto qos_in = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  manual_pub_ = node_->create_publisher<px4_msgs::msg::ManualControlSetpoint>(
    kManualControlTopic, qos_in);
  command_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>(
    kVehicleCommandTopic, qos_in);

  const auto qos_out = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  timesync_sub_ = node_->create_subscription<px4_msgs::msg::TimesyncStatus>(
    "/fmu/out/timesync_status", qos_out,
    [this](px4_msgs::msg::TimesyncStatus::SharedPtr msg) {onTimesync(msg);});
  local_pos_sub_ = node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/fmu/out/vehicle_local_position", qos_out,
    [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {onLocalPosition(msg);});
}

void Px4Adapter::onTimesync(const px4_msgs::msg::TimesyncStatus::SharedPtr msg)
{
  if (msg->timestamp == 0) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  px4_time_us_ = static_cast<int64_t>(msg->timestamp);
  px4_time_valid_ = true;
  // 记录“这帧是本地哪个时刻收到的”，之后用它推算 PX4 当前时间
  last_recv_ = std::chrono::steady_clock::now();
}

void Px4Adapter::onLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  if (!std::isfinite(msg->ref_alt)) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  reference_alt_ = msg->ref_alt;
  reference_alt_valid_ = true;
  z_ = msg->z;
}

uint64_t Px4Adapter::nowUs()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!px4_time_valid_) {
    // 第一帧 timesync 之前退回 ROS 时间
    return static_cast<uint64_t>(node_->get_clock()->now().nanoseconds() / 1000);
  }
  const auto elapsed = std::chrono::steady_clock::now() - last_recv_;
  const auto elapsed_us =
    std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
  // 故意减 100 ms：PX4 会拒绝 timestamp_sample 比它自己时钟还新的样本
  const int64_t now = px4_time_us_ + elapsed_us - 100000;
  return static_cast<uint64_t>(std::max<int64_t>(now, 0));
}

void Px4Adapter::publishManual(
  double roll, double pitch, double yaw, double throttle, bool valid, bool deadman)
{
  px4_msgs::msg::ManualControlSetpoint msg{};
  msg.timestamp = nowUs();
  msg.timestamp_sample = msg.timestamp;
  msg.valid = valid && deadman;
  msg.data_source = px4_msgs::msg::ManualControlSetpoint::SOURCE_MAVLINK_0;
  msg.roll = static_cast<float>(clampUnit(roll));
  msg.pitch = static_cast<float>(clampUnit(pitch));
  msg.yaw = static_cast<float>(clampUnit(yaw));
  msg.throttle = static_cast<float>(clampUnit(throttle));
  msg.flaps = 0.0F;
  msg.aux1 = msg.aux2 = msg.aux3 = msg.aux4 = msg.aux5 = msg.aux6 = 0.0F;
  msg.sticks_moving =
    std::fabs(msg.roll) > 0.01F || std::fabs(msg.pitch) > 0.01F ||
    std::fabs(msg.yaw) > 0.01F || std::fabs(msg.throttle) > 0.01F;
  msg.buttons = 0;
  manual_pub_->publish(msg);
}

void Px4Adapter::sendCommand(
  uint16_t command, float p1, float p2, float p3, float p4,
  double p5, double p6, float p7)
{
  px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp = nowUs();
  msg.command = command;
  msg.param1 = p1;
  msg.param2 = p2;
  msg.param3 = p3;
  msg.param4 = p4;
  msg.param5 = p5;
  msg.param6 = p6;
  msg.param7 = p7;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.confirmation = 0;
  msg.from_external = true;
  command_pub_->publish(msg);
}

void Px4Adapter::arm(bool arm_on)
{
  sendCommand(kCmdComponentArmDisarm, arm_on ? 1.0F : 0.0F);
}

void Px4Adapter::setMode(const std::string & mode)
{
  std::string upper;
  upper.reserve(mode.size());
  for (char c : mode) {
    upper.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
  }
  const auto it = kModeIds.find(upper);
  if (it == kModeIds.end()) {
    throw std::invalid_argument("unknown mode: " + mode);
  }
  // param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, param2 = PX4 main mode
  sendCommand(kCmdDoSetMode, 1.0F, static_cast<float>(it->second));
}

void Px4Adapter::takeoff(double altitude_m)
{
  double target_alt = altitude_m;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (reference_alt_valid_) {
      // NAV_TAKEOFF.param7 是 AMSL 绝对高度，不是离地高度
      target_alt = reference_alt_ - z_ + altitude_m;
    }
  }
  sendCommand(kCmdNavTakeoff, 0.0F, 0.0F, 0.0F, 0.0F, 0.0, 0.0,
    static_cast<float>(target_alt));
}

void Px4Adapter::land()
{
  sendCommand(kCmdNavLand);
}

void Px4Adapter::kill()
{
  sendCommand(kCmdComponentArmDisarm, 0.0F);
  sendCommand(kCmdDoFlightTermination, 1.0F);
}

}  // namespace flight_ui_cpp
