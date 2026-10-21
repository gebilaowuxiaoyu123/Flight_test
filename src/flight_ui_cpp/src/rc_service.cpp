#include "flight_ui_cpp/rc_service.hpp"

#include <json/writer.h>

#include <thread>

namespace flight_ui_cpp
{
namespace
{
constexpr double kStaleSeconds = 0.3;   // 超过这个时间没收到摇杆帧就给零输入
constexpr double kOwnerSeconds = 0.6;   // 现有控制端停发这么久才让位
constexpr double kArmRetrySeconds = 10.0;

}  // namespace

RcService::RcService(Px4Adapter & adapter, Telemetry & telemetry)
: adapter_(adapter), telemetry_(telemetry)
{
}

void RcService::tick()
{
  Sticks sticks;
  bool deadman = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sticks = sticks_;
    const auto age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_update_).count();
    deadman = sticks_.deadman && age <= kStaleSeconds;
  }
  if (!deadman) {
    adapter_.publishManual(0.0, 0.0, 0.0, -1.0, false, false);
    return;
  }
  adapter_.publishManual(
    sticks.roll, sticks.pitch, sticks.yaw, sticks.throttle, true, true);
}

bool RcService::setSticks(const Sticks & sticks, uint64_t client_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = std::chrono::steady_clock::now();
  if (owner_ != 0 && owner_ != client_id) {
    const auto age = std::chrono::duration<double>(now - last_update_).count();
    if (age < kOwnerSeconds) {
      return false;
    }
  }
  owner_ = client_id;
  sticks_ = sticks;
  last_update_ = now;
  return true;
}

void RcService::releaseClient(uint64_t client_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (owner_ == client_id) {
    owner_ = 0;
    sticks_.deadman = false;
    sticks_.throttle = -1.0;
  }
}

Sticks RcService::sticks() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return sticks_;
}

std::pair<bool, std::string> RcService::arm(uint64_t client_id)
{
  (void)client_id;
  const auto s = sticks();
  if (!s.deadman) {
    return {false, "deadman not held"};
  }
  if (s.throttle > -0.8) {
    return {false, "throttle must be at minimum"};
  }
  if (sequence_running_.exchange(true)) {
    return {false, "another command is running"};
  }
  std::thread([this]() {
      armSequence();
      sequence_running_ = false;
    }).detach();
  return {true, "arm command sent"};
}

void RcService::armSequence()
{
  ensureManualMode();
  if (!armWithRetry(kArmRetrySeconds)) {
    adapter_.arm(false);
  }
}

std::pair<bool, std::string> RcService::disarm(uint64_t client_id)
{
  (void)client_id;
  adapter_.arm(false);
  return {true, "disarm command sent"};
}

std::pair<bool, std::string> RcService::setMode(
  const std::string & mode, uint64_t client_id)
{
  (void)client_id;
  try {
    adapter_.setMode(mode);
  } catch (const std::exception & exc) {
    return {false, exc.what()};
  }
  return {true, "mode " + mode + " requested"};
}

std::pair<bool, std::string> RcService::takeoff(double altitude_m, uint64_t client_id)
{
  (void)client_id;
  const auto s = sticks();
  if (!s.deadman) {
    return {false, "deadman not held"};
  }
  if (s.throttle > -0.8) {
    return {false, "throttle must be at minimum"};
  }
  if (sequence_running_.exchange(true)) {
    return {false, "another command is running"};
  }
  std::thread([this, altitude_m]() {
      takeoffSequence(altitude_m);
      sequence_running_ = false;
    }).detach();
  return {true, "takeoff to " + std::to_string(altitude_m) + " m requested"};
}

void RcService::takeoffSequence(double altitude_m)
{
  // PX4 会丢弃和 ARM/切模式同一周期发出的 NAV_TAKEOFF，所以分成几步
  ensureManualMode();
  if (!armWithRetry(kArmRetrySeconds)) {
    return;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  adapter_.setMode("POSCTL");
  std::this_thread::sleep_for(std::chrono::milliseconds(800));
  adapter_.takeoff(altitude_m);
}

void RcService::ensureManualMode()
{
  if (!isManualNavState(telemetry_.snapshot().nav_state)) {
    adapter_.setMode("POSCTL");
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
  }
}

bool RcService::armWithRetry(double deadline_s)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(static_cast<int>(deadline_s * 1000.0));
  while (std::chrono::steady_clock::now() < deadline) {
    adapter_.arm(true);
    const auto armed_deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(1000);
    while (std::chrono::steady_clock::now() < armed_deadline) {
      if (telemetry_.snapshot().armed) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (telemetry_.snapshot().armed) {
      return true;
    }
    ensureManualMode();
  }
  return false;
}

std::pair<bool, std::string> RcService::land(uint64_t client_id)
{
  (void)client_id;
  adapter_.land();
  return {true, "land command sent"};
}

std::pair<bool, std::string> RcService::kill(uint64_t client_id)
{
  (void)client_id;
  adapter_.kill();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sticks_.deadman = false;
    sticks_.throttle = -1.0;
  }
  return {true, "kill command sent"};
}

Json::Value RcService::telemetryJson() const
{
  const auto t = telemetry_.snapshot();
  Json::Value root(Json::objectValue);
  root["armed"] = t.armed;
  root["mode"] = t.mode;
  root["nav_state"] = t.nav_state;
  root["alt"] = t.alt;
  root["vx"] = t.vx;
  root["vy"] = t.vy;
  root["vz"] = t.vz;
  root["battery"] = t.battery;
  root["voltage"] = t.voltage;
  root["xy_valid"] = t.xy_valid;
  root["z_valid"] = t.z_valid;
  root["landed"] = t.landed;
  return root;
}

}  // namespace flight_ui_cpp
