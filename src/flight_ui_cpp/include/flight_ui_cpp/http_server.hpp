#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>

#include "flight_ui_cpp/shared_state.hpp"

// 每个 accept 出来的 fd 一个线程：先读 HTTP 请求头，
// 是 WebSocket 升级就交给 serveWebSocket，否则走静态文件 / /health。
void handleConnection(int fd, SharedState & state, const std::string & web_root,
                      rclcpp::Node * node,
                      const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr & cmd_pub);
