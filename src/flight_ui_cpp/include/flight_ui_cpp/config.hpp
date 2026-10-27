#pragma once

#include <rclcpp/rclcpp.hpp>

// 后端的所有可调参数都在这里声明，默认值只是"没配也能跑"的兜底。
// 实际值来自 config/flight_ui.yaml（launch 里作为参数文件传进来）。
struct FlightUiConfig {
    int port = 8000;                    // HTTP / WebSocket 监听端口
    double takeoff_altitude_m = 2.5;    // 网页点"起飞"不写高度时用这个
    double arm_retry_sec = 10.0;        // 解锁重试窗口（PX4 冷启动要等 EKF 收敛）
    double control_rate_hz = 50.0;      // 杆量发布频率，PX4 要求 >= 10 Hz
    double telemetry_rate_hz = 1.0;     // 终端里 1 Hz 那行遥测日志
};

// 全局一份：HTTP 线程 → WebSocket 会话这条链上都要读，
// 逐层往下传参只会让 6 个文件互相认识，不如集中放这里。
const FlightUiConfig & config();

// 在 main 里调一次，把 ROS 参数读进上面的结构体
void loadConfig(rclcpp::Node & node);
