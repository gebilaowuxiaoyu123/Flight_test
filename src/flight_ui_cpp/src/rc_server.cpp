#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>

#include <chrono>

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>

#include <json/json.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <atomic>

namespace beast = boost::beast;         // from <boost/beast.hpp>
namespace http = beast::http;           // from <boost/beast/http.hpp>
namespace net = boost::asio;            // from <boost/asio.hpp>
using tcp = net::ip::tcp;               // from <boost/asio/ip/tcp

std::string jsonString(const Json::Value & value)
{
    Json::StreamWriterBuilder builder;
    builder["indentation"] = ""; // No indentation
    return Json::writeString(builder, value);
}

Json::Value buildJsonResponse(bool armed, int nav_state, bool z_valid, double z, float voltage_v)
{
    Json::Value root(Json::objectValue);
    root["status"] = "ok";
    root["ros"] = true;
    root["telemetry"]["armed"] = armed;
    root["telemetry"]["nav_state"] = nav_state;
    root["telemetry"]["z_valid"] = z_valid;
    root["telemetry"]["z"] = z;
    root["telemetry"]["voltage_v"] = voltage_v;
    return root;
}

void handleConnection(int fd, bool armed, int nav_state, bool z_valid, double z, float voltage_v)
{
    try
    {
        // tcp_stream 不能直接从 int fd 构造：需要 io_context + socket.assign 接管 fd
        net::io_context ioc(1);
        tcp::socket socket(ioc);
        beast::error_code ec;
        socket.assign(tcp::v4(), fd, ec);
        if (ec) {
            ::close(fd);
            return;
        }
        beast::tcp_stream stream(std::move(socket));
        beast::flat_buffer buffer;

        http::request<http::string_body> req;
        http::read(stream, buffer, req);

        Json::Value jsonResponse = buildJsonResponse(armed, nav_state, z_valid, z, voltage_v);
        std::string responseBody = jsonString(jsonResponse);

        http::response<http::string_body> res{http::status::ok, req.version()};
        res.set(http::field::server, "FlightUI");
        res.set(http::field::content_type, "application/json");
        res.keep_alive(false);   // 回完就关，语义明确
        res.body() = responseBody;
        res.prepare_payload();

        http::write(stream, res);
    }
    catch (const std::exception & e)
    {
        std::cerr << "Error handling connection: " << e.what() << std::endl;
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("flight_ui");
    auto logger = rclcpp::get_logger("flight_ui");

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    bool armed = false;
    int nav_state = -1;
    bool z_valid = false;
    double z=0.0;
    float voltage_v =0.0F;

    auto status_sub = node->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1", qos,
        [&](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
            armed = msg->arming_state == 2; // PX4 arming state: 2 means armed
            nav_state = msg->nav_state;
        });

    auto pos_sub = node->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", qos,
        [&](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
            z_valid = msg->z_valid;
            z = msg->z;
        });

    auto battery_sub = node->create_subscription<px4_msgs::msg::BatteryStatus>(
        "/fmu/out/battery_status", qos,
        [&](const px4_msgs::msg::BatteryStatus::SharedPtr msg) {
            voltage_v = msg->voltage_v;
        });

    const auto qos_in = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto manual_pub = node->create_publisher<px4_msgs::msg::ManualControlSetpoint>(
        "/fmu/in/manual_control_input", qos_in);

    bool deadman = true;
    double throttle_stick = 0.0;
    
    auto timer = node->create_wall_timer(std::chrono::milliseconds(1000), [&, logger]() {
        RCLCPP_INFO(logger, "Armed: %s, Nav State: %d, Z Valid: %s, Z: %.2f, Voltage: %.2f V",
                    armed ? "Yes" : "No", nav_state,
                    z_valid ? "Yes" : "No", z, voltage_v);
    });

    auto control_timer = node->create_wall_timer(std::chrono::milliseconds(20), [&, logger]() {
        
            px4_msgs::msg::ManualControlSetpoint msg;
            msg.timestamp = static_cast<uint64_t>(node->get_clock()->now().nanoseconds() / 1000);
            msg.timestamp_sample = msg.timestamp;
            msg.valid = deadman;
            msg.data_source = px4_msgs::msg::ManualControlSetpoint::SOURCE_MAVLINK_0; // Assuming 1 is the data source for manual control
            msg.roll = 0.0F;
            msg.pitch = 0.0F;
            msg.yaw = 0.0F;
            msg.throttle = static_cast<float>(throttle_stick);
            msg.flaps = 0.0F;
            msg.sticks_moving = false;
            msg.buttons = 0;

            manual_pub->publish(msg);
        
    });

    // -- HTTP 服务 --
    const int port = 8000;   // 和 Python 版、verify.sh、ws_smoke_test.py 保持一致

    const int listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    int yes = 1;
    ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (::bind(listen_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        RCLCPP_FATAL(logger, "bind(:%d) 失败，端口被调用？", port);
        rclcpp::shutdown();
        return 1;
    }
    ::listen(listen_fd, 16);

    std::atomic<bool> running{true};
    std::thread http_thread([&]() {
        while (running) {
            const int fd = ::accept(listen_fd, nullptr, nullptr);
            if (fd < 0) {
                break;
            }
            std::thread(handleConnection, fd, armed, nav_state, z_valid, z, voltage_v).detach();
        }
    });
    RCLCPP_INFO(logger, "HTTP 服务已启动，监听端口 %d", port);

    rclcpp::spin(node);

    running = false;
    ::shutdown(listen_fd, SHUT_RDWR);   // 让阻塞在 accept 的线程立刻返回
    ::close(listen_fd);
    if (http_thread.joinable()) {
        http_thread.join();
    }
    rclcpp::shutdown();
    return 0;
}