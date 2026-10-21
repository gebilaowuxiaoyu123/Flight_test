#pragma once

#include <json/value.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <utility>

#include "flight_ui_cpp/px4_adapter.hpp"
#include "flight_ui_cpp/telemetry.hpp"
#include "flight_ui_cpp/types.hpp"

namespace flight_ui_cpp
{

// 对应 Python 版的 RosWorker：持有摇杆状态、跑 50 Hz 定时发布、执行起飞/降落序列。
class RcService
{
public:
  RcService(Px4Adapter & adapter, Telemetry & telemetry);

  // 50 Hz 定时器回调
  void tick();

  // ---- 摇杆 ----
  // 同一时刻只接受一个控制端：现有控制端 0.6 s 没发帧就让位，
  // 避免遗留的浏览器标签页抢摇杆。
  bool setSticks(const Sticks & sticks, uint64_t client_id);
  void releaseClient(uint64_t client_id);
  Sticks sticks() const;

  // ---- 遥控命令（返回 {是否受理, 说明}）----
  std::pair<bool, std::string> arm(uint64_t client_id);
  std::pair<bool, std::string> disarm(uint64_t client_id);
  std::pair<bool, std::string> setMode(const std::string & mode, uint64_t client_id);
  std::pair<bool, std::string> takeoff(double altitude_m, uint64_t client_id);
  std::pair<bool, std::string> land(uint64_t client_id);
  std::pair<bool, std::string> kill(uint64_t client_id);

  Json::Value telemetryJson() const;

private:
  // 解锁重试：PX4 可能还在跑 arming checks（EKF 收敛、气压计预热）
  void ensureManualMode();
  bool armWithRetry(double deadline_s);
  void armSequence();
  void takeoffSequence(double altitude_m);

  Px4Adapter & adapter_;
  Telemetry & telemetry_;

  mutable std::mutex mutex_;
  Sticks sticks_;
  uint64_t owner_{0};
  std::chrono::steady_clock::time_point last_update_{};

  // 防止连点按钮同时跑多个序列
  std::atomic<bool> sequence_running_{false};
};

}  // namespace flight_ui_cpp
