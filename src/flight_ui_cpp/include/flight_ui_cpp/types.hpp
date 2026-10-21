#pragma once

#include <string>

namespace flight_ui_cpp
{

// 摇杆状态：范围 [-1, 1]，throttle 的 -1 表示最低
struct Sticks
{
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
  double throttle{-1.0};
  bool deadman{false};
};

// 给 HUD 用的遥测快照（加锁拷贝，避免读到一半被改）
struct TelemetrySnapshot
{
  bool armed{false};
  std::string mode{"UNKNOWN"};
  int nav_state{-1};
  double alt{0.0};
  double vx{0.0};
  double vy{0.0};
  double vz{0.0};
  double battery{-1.0};
  double voltage{0.0};
  bool xy_valid{false};
  bool z_valid{false};
  bool landed{true};
};

// 4021 机架没有 GPS，这些模式不依赖 GPS 位置估计
inline bool isManualNavState(int nav_state)
{
  switch (nav_state) {
    case 0:   // MANUAL
    case 1:   // ALTCTL
    case 2:   // POSCTL
    case 5:   // ACRO
    case 10:  // STABILIZED
    case 15:  // DESCEND
      return true;
    default:
      return false;
  }
}

}  // namespace flight_ui_cpp
