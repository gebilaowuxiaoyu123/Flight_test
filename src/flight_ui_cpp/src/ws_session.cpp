#include "flight_ui_cpp/ws_session.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include <rclcpp/logging.hpp>

#include "flight_ui_cpp/command_sequence.hpp"
#include "flight_ui_cpp/config.hpp"
#include "flight_ui_cpp/http_util.hpp"
#include "flight_ui_cpp/mode_tables.hpp"

void serveWebSocket(beast::tcp_stream stream,
    http::request<http::string_body> req,
    SharedState & state,
    rclcpp::Node * node,
    const rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr & cmd_pub,
    uint64_t client_id
){
    // 接管 HTTP 连接，升级成 WebSocket
    websocket::stream<beast::tcp_stream> ws(std::move(stream));

    // 超时：客户端拔网线时 read 会永远阻塞，线程就泄漏了
    ws.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));

    ws.accept(req);        // 握手：校验请求头 + 回 101

    // 用文本帧。不设的话默认二进制帧，浏览器收到 Blob，前端 JSON.parse 会失败
    ws.text(true);

    state.attachClient(client_id);
    RCLCPP_INFO(rclcpp::get_logger("flight_ui"),
                "[WS] 浏览器 #%lu 已连接", static_cast<unsigned long>(client_id));

    // ---- 写串行化 + 独立遥测推送 ----
    // Beast 同步流规则：同一连接允许 1 读 + 1 写并发，但绝不能 2 个写者。
    // 所以读循环回 ack、推送线程发遥测，全部走这把锁。
    std::mutex write_mu;
    std::atomic<bool> alive{true};

    auto send_text = [&](const std::string & payload) -> bool {
        std::lock_guard<std::mutex> lock(write_mu);
        if (!alive) return false;
        beast::error_code wec;
        ws.write(net::buffer(payload), wec);
        if (wec) { alive = false; return false; }
        return true;
    };

    // 10 Hz 推遥测，与客户端是否发帧无关（隐藏标签页 / 调试时也能看到实时状态）
    std::thread pusher([&]() {
        while (alive) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (!alive) break;
            if (!send_text(jsonString(state.getTelemetryFor(client_id)))) break;
        }
    });

    // 进入 WS 循环：读一帧 JSON 更新摇杆或执行命令
    for (;;)
    {
        beast::flat_buffer buffer;              // 每轮新建，天然不会有上一帧残留
        beast::error_code ec;
        ws.read(buffer, ec);                    // 用 error_code 版：断开时返回错误码而不是抛异常
        if (ec) {
            break;                              // 客户端断开/超时，正常收场
        }

        const std::string msg = beast::buffers_to_string(buffer.data());

        Json::CharReaderBuilder builder;
        Json::Value root;
        std::istringstream ss(msg);
        if (Json::parseFromStream(builder, ss, &root, nullptr)) {
            // 只有 sticks 帧才动摇杆状态。
            // 别的命令（arm/mode/takeoff…）没有 deadman 字段，
            // 若不判断类型会读到默认值 false，等于把使能松开 → 解锁必然被拒。
                        const std::string type = root.get("type", "").asString();

            // 命令类消息先确认 ROS 链路是通的。
            // 没起 MicroXRCEAgent 时 /fmu/* 一个话题都没有，命令发出去没人收，
            // 用户看到的现象就是"点了 ARM/TAKEOFF 毫无反应"。
            if (type != "sticks" && !type.empty() && state.statusAgeSec() > 1.5) {
                Json::Value ack(Json::objectValue);
                ack["type"] = "ack";
                ack["action"] = type;
                ack["ok"] = false;
                ack["message"] = "没有遥测：ROS 链路没通，先跑 flight link（起 MicroXRCEAgent）";
                send_text(jsonString(ack));
                RCLCPP_WARN(rclcpp::get_logger("flight_ui"),
                            "[cmd] 收到 %s 但 %.1f s 没遥测，已忽略：ROS 链路没通",
                            type.c_str(), state.statusAgeSec());
                continue;
            }

            if (type == "sticks") {
                state.setSticks(client_id,
                                root.get("deadman", false).asBool(),
                                root.get("roll", 0.0).asDouble(),
                                root.get("pitch", 0.0).asDouble(),
                                root.get("yaw", 0.0).asDouble(),
                                root.get("throttle", -1.0).asDouble());
            } else if (type == "disarm") {
                // 400 = COMPONENT_ARM_DISARM，param1 = 0 表示上锁
                sendCommand(cmd_pub, nowUs(node), 400, 0.0F);

                // 回执：前端靠 type=="ack" 在右下角打印结果
                Json::Value ack(Json::objectValue);
                ack["type"] = "ack";
                ack["action"] = "disarm";
                ack["ok"] = true;
                ack["message"] = "disarm command sent";
                send_text(jsonString(ack));
            } else if (type == "mode") {
                const std::string wanted = root.get("mode", "").asString();
                const int main_mode = mainModeOf(wanted);

                // ack 先声明好，两条路（成功/失败）都往同一个变量里填
                Json::Value ack(Json::objectValue);
                ack["type"] = "ack";
                ack["action"] = "mode";
                if (main_mode == 0) {
                    ack["ok"] = false;
                    ack["message"] = "unknown mode: " + wanted;
                } else {
                    // command 176 = DO_SET_MODE
                    // param1 = 1  → 启用"自定义模式"，告诉 PX4 我要用 param2 那个号
                    // param2      → 主模式号
                    sendCommand(cmd_pub, nowUs(node), 176,
                                1.0F, static_cast<float>(main_mode));
                    ack["ok"] = true;
                    ack["message"] = "switch to " + wanted + " requested";
                }

                send_text(jsonString(ack));
            } else if (type == "arm" || type == "takeoff") {
                const bool want_takeoff = (type == "takeoff");
                const double altitude =
                    root.get("altitude", config().takeoff_altitude_m).asDouble();
                const auto control = state.control();

                Json::Value ack(Json::objectValue);
                ack["type"] = "ack";
                ack["action"] = type;

                // 解锁/起飞是"命令"，不该因为没按使能就什么都不做
                // （旧逻辑直接回 "deadman not held"，用户按了起飞却毫无反应）。
                // PX4 只要求两件事：有有效的遥控信号 + 油门在最低。
                // 没按使能 → 这里把杆量临时置成"回中 + 油门最低"，既满足解锁条件，
                // 又不会让飞机带着偏杆起飞。
                if (control.valid && control.throttle > -0.8) {
                    // 按着使能却把油门推上去了：这时候解锁会直接蹿起来，拒掉
                    ack["ok"] = false;
                    ack["message"] = "throttle must be at minimum";
                } else {
                    if (!control.valid) {
                        state.setSticks(client_id, true, 0.0, 0.0, 0.0, -1.0);
                    }
                    ack["ok"] = true;
                    ack["message"] = want_takeoff ? "takeoff requested" : "arm requested";
                    // 解锁/起飞要重试最长 10 秒，绝不能堵在这个读循环里
                    if (want_takeoff) {
                        std::thread(takeoffSequence, std::ref(state), node,
                                    cmd_pub, altitude).detach();
                    } else {
                        std::thread(armSequence, std::ref(state), node,
                                    cmd_pub).detach();
                    }
                }

                send_text(jsonString(ack));
            } else if (type == "land") {
                landNow(node, cmd_pub);
                Json::Value ack(Json::objectValue);
                ack["type"] = "ack";
                ack["action"] = "land";
                ack["ok"] = true;
                ack["message"] = "land command sent";
                send_text(jsonString(ack));
            } else if (type == "kill") {
                killNow(node, cmd_pub);
                state.setSticks(client_id, false, 0.0, 0.0, 0.0, -1.0);
                Json::Value ack(Json::objectValue);
                ack["type"] = "ack";
                ack["action"] = "kill";
                ack["ok"] = true;
                ack["message"] = "kill command sent";
                send_text(jsonString(ack));
            }
        }

    }

    alive = false;
    if (pusher.joinable()) {
        pusher.join();
    }
    state.releaseClient(client_id);      // 断开就把控制权交出来
    RCLCPP_INFO(rclcpp::get_logger("flight_ui"),
                "[WS] 浏览器 #%lu 已断开", static_cast<unsigned long>(client_id));
}
