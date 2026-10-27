// rc_server 主程序：装配节点、订阅遥测、起 50 Hz 杆量发布和 HTTP 服务。
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <json/json.h>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <std_msgs/msg/float32.hpp>

#include "flight_ui_cpp/command_sequence.hpp"
#include "flight_ui_cpp/http_server.hpp"
#include "flight_ui_cpp/mode_tables.hpp"
#include "flight_ui_cpp/shared_state.hpp"

int main(int argc, char ** argv)   // 装配 + spin
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("flight_ui");
    auto logger = rclcpp::get_logger("flight_ui");

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // 跨线程共享的数据全放这里，不再有裸变量
    SharedState state;

    // 前端文件装在哪：查 ament 索引，换机器/换目录都不用改代码
    const std::string web_root =
        ament_index_cpp::get_package_share_directory("flight_ui_cpp") + "/web";
    RCLCPP_INFO(logger, "静态资源目录：%s", web_root.c_str());

    auto status_sub = node->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1", qos,
        [&](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
            state.setStatus(msg->arming_state == 2, // 2 = ARMED
                            msg->nav_state);
        });

    auto pos_sub = node->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", qos,
        [&](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
            state.setPosition(msg->z_valid, msg->z, msg->ref_alt);
        });

    auto battery_sub = node->create_subscription<px4_msgs::msg::BatteryStatus>(
        "/fmu/out/battery_status", qos,
        [&](const px4_msgs::msg::BatteryStatus::SharedPtr msg) {
            state.setBattery(msg->voltage_v, msg->remaining);
        });

    // 感知包（flight_perception_cpp 的 min_depth）输出正前方最近障碍距离。
    // 没跑感知时这条订阅收不到东西，遥测里 obstacle_m 就是 -1。
    auto obstacle_sub = node->create_subscription<std_msgs::msg::Float32>(
        "/obstacle_distance", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        [&](const std_msgs::msg::Float32::SharedPtr msg) {
            state.setObstacle(msg->data);
        });

    const auto qos_in = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto manual_pub = node->create_publisher<px4_msgs::msg::ManualControlSetpoint>(
        "/fmu/in/manual_control_input", qos_in);

    // 命令发布者：和摇杆是两个不同的话题（连续量 vs 一次性指令）
    auto cmd_pub = node->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", qos_in);

    // 1 Hz 日志：读一份快照打印（别在这里直接读成员变量）
    auto timer = node->create_wall_timer(std::chrono::milliseconds(1000), [&, logger]() {
        const Json::Value t = state.getTelemetry();
        RCLCPP_INFO(logger, "armed=%s nav_state=%d alt=%.2f voltage=%.2f",
                    t["armed"].asBool() ? "Yes" : "No",
                    t["nav_state"].asInt(),
                    t["alt"].asDouble(),
                    t["voltage_v"].asDouble());
    });

    // 50 Hz：先从 state 取一份加锁快照，再在锁外发布
    auto control_timer = node->create_wall_timer(std::chrono::milliseconds(20), [&]() {
        const auto c = state.control();
        const bool armed = state.armed();

        // valid 表示"有人在管这架飞机"。PX4 用 valid=false 判定遥控丢失，
        // 会把未经使能的帧当成"没有遥控器"→ 直接拒绝解锁。
        // 所以只要还有网页连着就报 valid=true；deadman 只决定杆量传不传下去：
        //   没按使能 → 杆量回中；油门给"最低"（没解锁）或"中位悬停"（已解锁）。
        const bool has_pilot = c.any_client;
        const float roll  = c.valid ? static_cast<float>(c.roll)  : 0.0F;
        const float pitch = c.valid ? static_cast<float>(c.pitch) : 0.0F;
        const float yaw   = c.valid ? static_cast<float>(c.yaw)   : 0.0F;
        const float thr   = c.valid ? static_cast<float>(c.throttle)
                                    : (armed ? 0.0F : -1.0F);

        px4_msgs::msg::ManualControlSetpoint msg;
        msg.timestamp = static_cast<uint64_t>(node->get_clock()->now().nanoseconds() / 1000);
        msg.timestamp_sample = msg.timestamp;
        msg.valid = has_pilot;
        msg.data_source = px4_msgs::msg::ManualControlSetpoint::SOURCE_MAVLINK_0; // 2
        msg.roll = roll;
        msg.pitch = pitch;
        msg.yaw = yaw;
        msg.throttle = thr;
        msg.flaps = 0.0F;
        msg.sticks_moving = std::abs(roll) > 0.02F || std::abs(pitch) > 0.02F;
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
            std::thread(handleConnection, fd, std::ref(state), web_root,
                        node.get(), cmd_pub).detach();   // 每连接一个线程
        }
    });
    RCLCPP_INFO(logger, "HTTP 服务已启动，监听端口 %d", port);
    RCLCPP_INFO(logger, "网页遥控器： http://127.0.0.1:%d/", port);

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
