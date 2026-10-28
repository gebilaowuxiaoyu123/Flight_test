// mode_tables.cpp —— 两张模式表：读方向（nav_state → 名字）和写方向（名字 → 主模式号）
// 对外：modeNameOf / mainModeOf
// 注意：这两套编号不是同一套，数值不能互相套用

#include "flight_ui_cpp/mode_tables.hpp"

// PX4 的 nav_state 数字 → 人看的模式名（和 Python 版 telemetry.py 完全一致）
std::string modeNameOf(int nav_state)
{
    switch (nav_state) {
        case 0:  return "MANUAL";
        case 1:  return "ALTCTL";
        case 2:  return "POSCTL";
        case 3:  return "AUTO_MISSION";
        case 4:  return "AUTO_LOITER";
        case 5:  return "AUTO_RTL";
        case 6:  return "POSITION_SLOW";
        case 10: return "ACRO";
        case 12: return "DESCEND";
        case 13: return "TERMINATION";
        case 14: return "OFFBOARD";
        case 15: return "STABILIZED";
        case 17: return "AUTO_TAKEOFF";
        case 18: return "AUTO_LAND";
        case 19: return "AUTO_FOLLOW";
        case 20: return "AUTO_PRECLAND";
        case 21: return "ORBIT";
        case 22: return "AUTO_VTOL_TAKEOFF";
        default: return "STATE_" + std::to_string(nav_state);
    }
}

int mainModeOf(const std::string & name)
{
    if (name == "MANUAL")     return 1;
    if (name == "ALTCTL")     return 2;
    if (name == "POSCTL")     return 3;
    if (name == "AUTO")       return 4;
    if (name == "ACRO")       return 5;
    if (name == "OFFBOARD")   return 6;
    if (name == "STABILIZED") return 7;
    return 0;
}
