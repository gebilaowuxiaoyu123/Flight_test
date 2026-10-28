// config.cpp —— 把 config/flight_ui.yaml 读进 FlightUiConfig
// 对外：loadConfig() / config()
// 说明：全局一份；HTTP 线程、WS 会话、定时器都要读，逐层传参不划算

#include "flight_ui_cpp/config.hpp"

#include <algorithm>

namespace {

FlightUiConfig g_config;

}  // namespace

const FlightUiConfig & config()
{
    return g_config;
}

void loadConfig(rclcpp::Node & node)
{
    g_config.port = node.declare_parameter<int>("port", g_config.port);
    g_config.takeoff_altitude_m =
        node.declare_parameter<double>("takeoff_altitude_m", g_config.takeoff_altitude_m);
    g_config.arm_retry_sec =
        node.declare_parameter<double>("arm_retry_sec", g_config.arm_retry_sec);
    g_config.control_rate_hz =
        node.declare_parameter<double>("control_rate_hz", g_config.control_rate_hz);
    g_config.telemetry_rate_hz =
        node.declare_parameter<double>("telemetry_rate_hz", g_config.telemetry_rate_hz);

    // 频率写错会让定时器周期变成 0 或负 → 直接崩，这里夹一下
    g_config.control_rate_hz = std::clamp(g_config.control_rate_hz, 1.0, 500.0);
    g_config.telemetry_rate_hz = std::clamp(g_config.telemetry_rate_hz, 0.1, 50.0);
    g_config.arm_retry_sec = std::clamp(g_config.arm_retry_sec, 1.0, 60.0);
    g_config.takeoff_altitude_m = std::clamp(g_config.takeoff_altitude_m, 0.3, 20.0);
}
