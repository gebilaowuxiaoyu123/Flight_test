#pragma once

#include <cstdint>

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>

#include "flight_ui_cpp/shared_state.hpp"

namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
namespace websocket = beast::websocket;
using tcp = net::ip::tcp;

// 一个 WebSocket 会话：收前端 JSON（摇杆/命令），回遥测和 ack。
void serveWebSocket(beast::tcp_stream stream,
                    http::request<http::string_body> req,
                    SharedState & state,
                    rclcpp::Node * node,
                    const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr & cmd_pub,
                    uint64_t client_id);
